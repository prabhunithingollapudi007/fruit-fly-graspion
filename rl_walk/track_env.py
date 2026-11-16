"""Trajectory-tracking env on the same plant as env.GoalEnv: follow a reference point that moves along a circle or
a figure-8 (hw_fig8's formulas, so a deployed policy and the lab's MPC track the same thing).

- reference: time-indexed, as in hw_fig8: the error is the distance from the tag to the reference point of the moment.
  Circles: radius 0.10-0.22 m, 3-5.5 cm/s, both directions. Figure-8s: half width 0.15-0.28 m, half height
  0.08-0.17 m, peak speed 4.5-6.2 cm/s (the MPC's benchmark 0.25 x 0.15 m in 40 s peaks at 6.1), both directions.
  The robot's fastest measured cells do 7-8 cm/s, so these stay inside what it can do.
- observation (no absolute position, so one policy serves any reference): the reference point now and 0.3, 0.6,
  1.0 and 1.5 s ahead in the robot's frame, the reference heading, speed and turn rate 0.3 s ahead, the last commands.
- reward per tick: 1 - error / 5 cm (so positive inside 5 cm), -switch_cost when the command changes; -20 and the
  episode ends when the error passes 25 cm or the tag leaves the arena. switch_cost 0.3: at 0.05 the policy changed
  its command on 80 % of ticks (0.5 cm median error in sim), at 0.3 on 36 % (0.7 cm), at 1.0 on 24 % (1.2 cm). Each
  change is an IR line, and drive-to-drive switching was never timed on the robot, so fewer changes is safer.
- start: at rest, within 4 cm and 0.5 rad of a random point of the reference. Episodes last 30 s.
"""
import gymnasium as gym
import numpy as np

from env import ARENA, NOMINAL, GreedyMap, PlantEnv

LOOK = (0.0, 0.3, 0.6, 1.0, 1.5)   # preview times [s]
EDGE = 0.05                        # the reference stays this far inside the arena [m]
LOST = 0.25                        # error that ends the episode [m]


class Figure8:
    """x = cx + A sin(d w t), y = cy + (B / 2) sin(2 d w t), A = ax, B = 2 ay; d = -1 runs it backwards."""

    def __init__(self, cx, cy, ax, ay, period, direction=1):
        self.cx, self.cy, self.A, self.B, self.T = cx, cy, ax, 2.0 * ay, period
        self.w = (1 if direction >= 0 else -1) * 2.0 * np.pi / period

    def at(self, t):
        s = self.w * np.asarray(t, float)
        x = self.cx + self.A * np.sin(s)
        y = self.cy + 0.5 * self.B * np.sin(2 * s)
        dx, dy = self.A * self.w * np.cos(s), self.B * self.w * np.cos(2 * s)
        ddx, ddy = -self.A * self.w ** 2 * np.sin(s), -2 * self.B * self.w ** 2 * np.sin(2 * s)
        v = np.hypot(dx, dy)
        return x, y, np.arctan2(dy, dx), v, (dx * ddy - dy * ddx) / np.maximum(v * v, 1e-12)


class Circle:
    """x = cx + R cos(d w t), y = cy + R sin(d w t); d = +1 counter-clockwise, -1 clockwise."""

    def __init__(self, cx, cy, r, period, direction=-1):
        self.cx, self.cy, self.R, self.T = cx, cy, r, period
        self.w = (1 if direction >= 0 else -1) * 2.0 * np.pi / period

    def at(self, t):
        s = self.w * np.asarray(t, float)
        th = s + np.sign(self.w) * np.pi / 2
        return (self.cx + self.R * np.cos(s), self.cy + self.R * np.sin(s), np.arctan2(np.sin(th), np.cos(th)),
                np.full_like(s, self.R * abs(self.w)), np.full_like(s, self.w))


def benchmark(shape):
    """The fixed references used for comparison: the MPC's figure-8 of run 20261112_152628 (50 x 30 cm in 40 s),
    and a circle of radius 15 cm in 20 s (4.7 cm/s), both at the arena's centre."""
    cx, cy = ARENA.mean(1)
    return Figure8(cx, cy, 0.25, 0.15, 40.0) if shape == "figure8" else Circle(cx, cy, 0.15, 20.0, -1)


def phase_near(ref, x, y, yaw):
    """The reference time whose point is closest to (x, y), heading mismatch penalised (as hw_fig8 starts a run)."""
    t = np.linspace(0.0, ref.T, 721, endpoint=False)
    rx, ry, rth = ref.at(t)[:3]
    d = np.hypot(rx - x, ry - y) + 0.05 * np.abs((yaw - rth + np.pi) % (2 * np.pi) - np.pi)
    return float(t[np.argmin(d)])


def make_track_obs(pose, ref, t, hist_LR):
    """The policy's input, shared by the env and deploy.py. t: the reference time of the pose."""
    x, y, th = pose
    c, s = np.cos(th), np.sin(th)
    rx, ry, rth, rv, rom = ref.at(t + np.array(LOOK))
    dx, dy = rx - x, ry - y
    body = np.clip(np.stack([c * dx + s * dy, -s * dx + c * dy], 1) / 0.1, -3, 3)
    dth = rth[1] - th
    return np.concatenate([body.ravel(), [np.cos(dth), np.sin(dth), rv[1] / 0.06, rom[1]],
                           (np.asarray(hist_LR, float) / 120.0).ravel()]).astype(np.float32)


class TrackEnv(PlantEnv):
    def __init__(self, shapes=("circle", "figure8"), randomize=True, t_max=30.0, n_hist=2, seed=None, ranges=None,
                 ref=None, switch_cost=0.3):
        """shapes: what reset() draws from; ref: a fixed reference instead (benchmark())."""
        super().__init__(randomize, n_hist, seed, ranges)
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (2 * len(LOOK) + 4 + 2 * n_hist,), np.float32)
        self.shapes, self.t_max, self.fixed_ref, self.switch_cost = tuple(shapes), t_max, ref, switch_cost

    def _random_ref(self):
        r = self.rng
        d = r.choice([-1, 1])
        (x0, x1), (y0, y1) = ARENA
        if r.choice(self.shapes) == "circle":
            R = r.uniform(0.10, 0.22)
            period = 2 * np.pi * R / r.uniform(0.03, 0.055)
            return Circle(r.uniform(x0 + R + EDGE, x1 - R - EDGE), r.uniform(y0 + R + EDGE, y1 - R - EDGE), R, period, d)
        ax, ay = r.uniform(0.15, 0.28), r.uniform(0.08, 0.17)
        period = 2 * np.pi * np.hypot(ax, 2 * ay) / r.uniform(0.045, 0.062)
        return Figure8(r.uniform(x0 + ax + EDGE, x1 - ax - EDGE), r.uniform(y0 + ay + EDGE, y1 - ay - EDGE), ax, ay,
                       period, d)

    def _obs(self):
        return make_track_obs(self.pose, self.ref, self.t0 + self.t, self.LR[self.hist])

    def error(self):
        rx, ry = self.ref.at(self.t0 + self.t)[:2]
        return float(np.hypot(rx - self.pose[0], ry - self.pose[1]))

    def reset(self, seed=None, options=None):
        self._reset_plant(seed)
        self.ref = self.fixed_ref or self._random_ref()
        self.t0 = self.rng.uniform(0, self.ref.T)
        rx, ry, rth = (float(v) for v in self.ref.at(self.t0)[:3])
        off, ang = self.rng.uniform(0, 0.04), self.rng.uniform(-np.pi, np.pi)
        self.pose = np.array([rx + off * np.cos(ang), ry + off * np.sin(ang), rth + self.rng.uniform(-0.5, 0.5)])
        return self._obs(), {}

    def step(self, action):
        changed = int(action) != self.hist[-1]
        self._advance(action)
        e = self.error()
        lost = e > LOST or self.outside()
        reward = max(1 - e / 0.05, -3.0) - self.switch_cost * changed - (20.0 if lost else 0.0)
        return self._obs(), reward, lost, self.t >= self.t_max and not lost, {"err": e, "lost": lost, "t": self.t}


class GreedyTrack(GreedyMap):
    """Baseline: every tick, the cell whose nominal arc ends nearest the reference point of that moment (and best
    aligned with its heading), from the pose predicted through the commands in flight. The horizon and heading weight
    were picked in the simulator (0.5 cm median there, against 1.5-2 cm at GreedyMap's 0.6 s and 0.03)."""

    def __init__(self, horizon=0.4, k_face=0.01, **kw):
        super().__init__(horizon=horizon, k_face=k_face, **kw)

    def __call__(self, env):
        pose = self.predicted_pose(env)
        ends = self._arc(np.broadcast_to(pose, (len(self.cells), 3)), self.cells, self.h)
        rx, ry, rth = (float(v) for v in env.ref.at(env.t0 + env.t + self.delay + self.h)[:3])
        d = np.hypot(ends[:, 0] - rx, ends[:, 1] - ry)
        d += self.k_face * np.abs((rth - ends[:, 2] + np.pi) % (2 * np.pi) - np.pi)
        return int(np.argmin(d))
