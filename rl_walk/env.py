"""Goal-reaching env for one GRASPion bot on the lab rig, with the plant identified on udZn9 (plant_check.py,
step_fit.py):

- action: one of the 81 measured grid cells (L, R) in {0, 55, 60, 70, ..., 120}^2; cell (0, 0) is stop.
  The twist (u1, u2, omega) of the tracked point is that cell's measured mean (grid_map_combined.json), so the
  policy never commands a drive without data. Battery ideal (full charge, first 10 min).
- timing: a command acts after a dead time (0.17 s nominal), through a first-order lag (tau 0.02 nominal, 0-0.06);
  control tick 0.14 s (0.12-0.16 randomised). Randomised dead time = a per-episode base (0.12-0.30: the 0.17 s was
  measured to frame arrival, the live loop adds tracker and policy time that is not measured yet) plus a jitter
  per command (+-0.05; the calibration steps scatter 0.10-0.26 s from one command to the next); commands never
  overtake each other. Measured only for rest <-> drive; assumed the same between two drive cells.
- noise: white twist noise held for one tick, sd (1.3 cm/s, 0.7 cm/s, 0.65 rad/s) at scale 1: set from the one-step
  residual of run 20261112_152628 and checked on that same run only (in-sample).
- map spread per episode (randomised): gains on speed and turn rate, a turn-rate bias, and an error per cell
  (sd 0.7 cm/s forward, 0.35 cm/s sideways, 0.3 rad/s: most cells have 1-3 segments; days differed by 0.74 cm/s).
- arena: where the tag centre stays clear of the reference markers, x 0.086-0.860, y 0.087-0.633 m; leaving it
  ends the episode (the rig's geofence).
"""
import json
import os

import gymnasium as gym
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ARENA = np.array([[0.086, 0.860], [0.087, 0.633]])
MARGIN = 0.08          # spawn and goal distance from the arena edge [m]
SUB = 0.01             # integration step [s]
NOISE_SD = np.array([0.013, 0.007, 0.65])
CELL_SD = np.array([0.007, 0.0035, 0.3])   # per-cell map error at cell_err = 1
NOMINAL = dict(delay=0.17, jitter=0.0, tau=0.02, tick=0.14, noise=1.0, gv=1.0, gw=1.0, wbias=0.0, cell_err=0.0)
RANDOM = dict(delay=(0.12, 0.30), jitter=(0.05, 0.05), tau=(0.0, 0.06), tick=(0.12, 0.16), noise=(0.7, 1.3),
              gv=(0.9, 1.1), gw=(0.9, 1.1), wbias=(-0.1, 0.1), cell_err=(1.0, 1.0))


def load_cells(path=os.path.join(HERE, "data", "grid_map_combined.json")):
    d = json.load(open(path))
    lv = np.array(d["levels"], float)
    tab = np.array(d["table"], float)            # (9, 9, 3): rows L, cols R
    LR = np.array([(l, r) for l in lv for r in lv])
    tw = tab.reshape(-1, 3).copy()
    tw[0] = 0.0                                  # (0, 0) = stop
    return LR, tw


def make_obs(pose, goal, hist_LR):
    """The policy's input, shared by the env and deploy.py: goal in the body frame, distance and bearing, position in
    the arena (-1..1), heading, and the last commands as drives / 120 (oldest first)."""
    x, y, th = pose
    d = np.asarray(goal) - [x, y]
    c, s = np.cos(th), np.sin(th)
    gb = np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1]])
    bearing = np.arctan2(gb[1], gb[0])
    pos = 2 * (np.array([x, y]) - ARENA[:, 0]) / (ARENA[:, 1] - ARENA[:, 0]) - 1
    return np.concatenate([gb / 0.5, [np.hypot(*gb) / 0.5, np.cos(bearing), np.sin(bearing)], pos, [c, s],
                           (np.asarray(hist_LR, float) / 120.0).ravel()]).astype(np.float32)


class GoalEnv(gym.Env):
    """Drive the tag to a goal within `goal_tol`; reward = progress [cm], -0.01 per tick, +10 on arrival,
    -10 on leaving the arena. Episodes last at most `t_max` seconds."""

    def __init__(self, randomize=True, goal_tol=0.03, t_max=30.0, n_hist=2, seed=None, ranges=None):
        """ranges: overrides of RANDOM entries (train.py's stress tests outside the training ranges)."""
        self.ranges = {**RANDOM, **(ranges or {})}
        self.LR, self.cells0 = load_cells()
        self.cells = self.cells0
        self.action_space = gym.spaces.Discrete(len(self.LR))
        self.n_hist = n_hist
        self.observation_space = gym.spaces.Box(-np.inf, np.inf, (9 + 2 * n_hist,), np.float32)
        self.randomize, self.goal_tol, self.t_max = randomize, goal_tol, t_max
        self.rng = np.random.default_rng(seed)

    # ------------------------------------------------------------------ helpers
    def _sample(self, lo_hi):
        return self.rng.uniform(*lo_hi)

    def _random_point(self):
        return np.array([self.rng.uniform(ARENA[0, 0] + MARGIN, ARENA[0, 1] - MARGIN),
                         self.rng.uniform(ARENA[1, 0] + MARGIN, ARENA[1, 1] - MARGIN)])

    def _obs(self):
        return make_obs(self.pose, self.goal, self.LR[self.hist])

    # ------------------------------------------------------------------ gym API
    def reset(self, seed=None, options=None):
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        p = {k: (self._sample(self.ranges[k]) if self.randomize else v) for k, v in NOMINAL.items()}
        self.p = p
        self.cells = self.cells0 + p["cell_err"] * self.rng.normal(0, CELL_SD, self.cells0.shape)
        self.cells[0] = 0.0
        self.pose = np.array([*self._random_point(), self.rng.uniform(-np.pi, np.pi)])
        self.goal = self._random_point()
        while np.hypot(*(self.goal - self.pose[:2])) < 0.10:
            self.goal = self._random_point()
        self.t = 0.0
        self.queue = [(-1.0, np.zeros(3))]      # (time it acts from, twist) of every command still relevant
        self.twist = np.zeros(3)                # lagged twist
        self.hist = [0] * self.n_hist
        self.dist = np.hypot(*(self.goal - self.pose[:2]))
        return self._obs(), {}

    def step(self, action):
        p = self.p
        tw = self.cells[int(action)] * [p["gv"], p["gv"], p["gw"]]
        if action != 0:
            tw = tw + [0, 0, p["wbias"]]
        # acts after the episode's dead time plus this command's jitter, never before the command sent before it
        t_act = self.t + max(0.0, p["delay"] + self.rng.uniform(-1, 1) * p["jitter"])
        self.queue.append((max(t_act, self.queue[-1][0]), tw))
        self.hist = self.hist[1:] + [int(action)]
        tick = p["tick"] if not self.randomize else self.rng.uniform(*self.ranges["tick"])
        noise = self.rng.normal(0, NOISE_SD * p["noise"])
        a = SUB / (p["tau"] + SUB)
        x, y, th = self.pose
        for _ in range(int(round(tick / SUB))):
            # the command in force: the last one whose dead time has passed
            while len(self.queue) > 1 and self.queue[1][0] <= self.t + 1e-9:
                self.queue.pop(0)
            self.twist += a * (self.queue[0][1] - self.twist)
            u1, u2, w = self.twist + (noise if np.any(self.queue[0][1]) else 0.0)  # no wobble when stopped
            c, s = np.cos(th), np.sin(th)
            x += (u1 * c - u2 * s) * SUB
            y += (u1 * s + u2 * c) * SUB
            th += w * SUB
            self.t += SUB
        self.pose = np.array([x, y, (th + np.pi) % (2 * np.pi) - np.pi])

        dist = np.hypot(*(self.goal - self.pose[:2]))
        reward = 100 * (self.dist - dist) - 0.01
        self.dist = dist
        out = not (ARENA[0, 0] <= x <= ARENA[0, 1] and ARENA[1, 0] <= y <= ARENA[1, 1])
        reached = dist < self.goal_tol
        if reached:
            reward += 10
        if out:
            reward -= 10
        truncated = self.t >= self.t_max and not (reached or out)
        return self._obs(), reward, reached or out, truncated, {"reached": reached, "out": out, "t": self.t}


class GreedyMap:
    """Baseline that uses the map directly: every tick, pick the cell whose nominal arc ends nearest the goal (and best facing it) after
    `horizon` seconds, from the pose predicted through the commands still in flight (the nominal delay)."""

    def __init__(self, horizon=0.6, delay=NOMINAL["delay"], tick=NOMINAL["tick"], k_face=0.03):
        self.LR, self.cells = load_cells()
        self.h, self.delay, self.tick, self.k_face = horizon, delay, tick, k_face   # k_face [m per rad]

    @staticmethod
    def _arc(pose, tw, T):
        x, y, th = pose[..., 0], pose[..., 1], pose[..., 2]
        u1, u2, w = tw[..., 0], tw[..., 1], tw[..., 2]
        small = np.abs(w) < 1e-6
        ws = np.where(small, 1.0, w)
        th1 = th + w * T
        # body twist (u1, u2) rotates with the heading: exact for constant twist
        dx = np.where(small, (u1 * np.cos(th) - u2 * np.sin(th)) * T,
                      (u1 * (np.sin(th1) - np.sin(th)) + u2 * (np.cos(th1) - np.cos(th))) / ws)
        dy = np.where(small, (u1 * np.sin(th) + u2 * np.cos(th)) * T,
                      (-u1 * (np.cos(th1) - np.cos(th)) + u2 * (np.sin(th1) - np.sin(th))) / ws)
        return np.stack([x + dx, y + dy, th1], -1)

    def __call__(self, env):
        # roll the pose forward through the delay with the commands already sent (oldest first)
        # the command sent k ticks ago acts on [-k*tick + delay, -(k-1)*tick + delay], clipped to [0, delay]
        pose = env.pose.copy()
        for k in range(len(env.hist), 0, -1):
            t0 = max(0.0, -k * self.tick + self.delay)
            t1 = min(self.delay, -(k - 1) * self.tick + self.delay)
            if t1 > t0:
                pose = self._arc(pose, self.cells[env.hist[-k]], t1 - t0)
        ends = self._arc(np.broadcast_to(pose, (len(self.cells), 3)), self.cells, self.h)
        d = np.hypot(ends[:, 0] - env.goal[0], ends[:, 1] - env.goal[1])
        # facing the goal counts too, and standing still is never chosen: with distance alone it stalled on every
        # goal behind the robot, where no short arc ends nearer than stopping (16 % of episodes)
        face = np.arctan2(env.goal[1] - ends[:, 1], env.goal[0] - ends[:, 0]) - ends[:, 2]
        d += self.k_face * np.abs((face + np.pi) % (2 * np.pi) - np.pi)
        d[0] += 1.0
        inside =((ends[:, 0] > ARENA[0, 0] + 0.02) & (ends[:, 0] < ARENA[0, 1] - 0.02)
                  & (ends[:, 1] > ARENA[1, 0] + 0.02) & (ends[:, 1] < ARENA[1, 1] - 0.02))
        d[~inside] += 1.0
        return int(np.argmin(d))
