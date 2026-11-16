"""Run a trained policy on the robot: overhead camera -> tag pose -> the env's own observation function -> PPO ->
one of the 81 grid cells -> "pwm L R ms" to the IR sender (W10nY, ir_remote_tx) -> the robot (udZn9, ir_drive_rx).
Camera, tracker and IR link are hw_fig8's (brainbot_ws, tools/hw_fig8).

    python rl_walk/deploy.py --mode sim --n-goals 5                 # the env's plant instead of camera and IR
    python rl_walk/deploy.py --mode dry --n-goals 5                 # live camera, policy runs, nothing is sent
    python rl_walk/deploy.py --mode run --n-goals 5 --port COM5     # the robot drives to goals (train.py's policy)
    python rl_walk/deploy.py --mode run --task track --shape circle --radius 0.15 --period 20
    python rl_walk/deploy.py --mode run --task track --shape figure8 --ax 0.25 --ay 0.15 --period 40 --loops 2

--task goal: --goals "0.3,0.3 0.6,0.4" or --n-goals N random points (training's margins, >= 10 cm from the robot).
--task track (train_track.py's policy): a circle or figure-8 around --center (default the arena's), entered at the
point nearest the robot, for --loops periods; the error is the distance to the reference point of the moment, counted
after the first 3 s. The run stops if the error passes 25 cm (the policy never trained beyond that).
Safety: stop when the tag is unseen for --unseen-stop s, abort after --abort-s; abort when the tag, the sighting
brought up to now, or the pose after the commands in flight leaves the arena the policy was trained in; stop after
--max-min minutes (full charge, first 10 min); Ctrl-C stops; every pwm line lasts --cmd-ms, so the robot stops by
itself within --cmd-ms + 0.5 s (ir_drive_rx's coast) if this program stalls.
IR load: a pwm line only on a change or after --cmd-ms / 2, as in calibration, where the 0.17 s delay was measured.
The policy sees the sighting rolled forward by its age with the commands sent (training saw the current pose).
"""
import argparse
import csv
import math
import os
import pathlib
import signal
import sys
import time

import numpy as np

from env import ARENA, MARGIN, NOMINAL, GoalEnv, GreedyMap, load_cells, make_obs
from track_env import EDGE, LOST, Circle, Figure8, make_track_obs, phase_near

HERE = pathlib.Path(__file__).resolve().parent


# ------------------------------------------------------------------------------------------------------------- rigs
class SimRig:
    """The env's plant (delay, lag, noise) in place of camera and IR; time is the env's clock."""

    def __init__(self, randomize, seed):
        self.env = GoalEnv(randomize=randomize, seed=seed)
        self.env.reset(seed=seed)

    def pose(self, tick):
        return self.env.t, tuple(self.env.pose), 0.0

    def send(self, a, L, R):
        self.env._advance(a)

    def stop(self):
        for _ in range(2):              # until the last drive has passed the dead time
            self.env._advance(0)

    def close(self):
        pass


class LiveRig:
    """hw_fig8's camera, tracker and IR link (brainbot_ws)."""

    def __init__(self, a, out, stamp):
        ws = pathlib.Path(a.ws).resolve()
        for p in ("src/graspion_tracking", "src/graspion_sim", "tools/hw_fig8"):
            sys.path.insert(0, str(ws / p))
        import json
        import hw_fig8 as hf
        from graspion_tracking import live_tracking as lt
        from graspion_tracking.arena import calibrate_arena
        from graspion_tracking.intrinsics import load_camera
        from graspion_tracking.markers import ArenaConfig, RobotTag, detector
        from graspion_tracking.tracker import F_JUMP, TagTracker
        self.F_JUMP = F_JUMP
        calib = pathlib.Path(a.calib) if a.calib else ws / "src/graspion_tracking/calibration/results/" \
            "calib_2026-11-12_interim_v4"
        cam = load_camera(calib / "intrinsics.json")
        arena = ArenaConfig.from_dict(json.loads((calib / "arena.json").read_text()))
        r = next(e for e in json.loads((calib / "robots.json").read_text())["robots"] if e["robot"] == a.robot)
        self.tag = RobotTag(r["robot"], int(r["tag_id"]), float(r["side"]), float(r["height"]),
                            tuple(r.get("offset", (0, 0))), float(r.get("yaw_offset", 0.0)))
        self.link = None
        self.rx_log = open(out / f"ir_{a.mode}_{stamp}.log", "w")
        if a.mode == "run":
            self.link = hf.IrLink(a.port, lambda ln: self.rx_log.write(f"{time.monotonic():.3f} {ln}\n"))
            self.link.send("s")
        self.src = lt.PylonSource(exposure_us=a.exposure_us, gain_db=a.gain_db, latest_only=True)
        geo = self.src.geometry
        win = cam.meta.get("window") or {"offset_x": 0, "offset_y": 0, "width": cam.size[0], "height": cam.size[1]}
        have = tuple(int(geo.get(k) or 0) for k in ("OffsetX", "OffsetY", "Width", "Height"))
        need = tuple(int(win[k]) for k in ("offset_x", "offset_y", "width", "height"))
        if have != need:
            self.close()
            raise SystemExit(f"camera window {have} differs from the calibration window {need}: set it first")
        det = detector()
        cal = None
        for _ in range(10):                               # the best of 10 frames (a hand in view spoils one)
            c = calibrate_arena(self.src.read().gray, cam, arena, det, gates=hf.LOOSE)
            if c.pose is not None and (cal is None or cal.pose is None or c.rms_px < cal.rms_px):
                cal = c
            cal = cal or c
        if cal.pose is None:
            self.close()
            raise SystemExit("no camera pose from the arena markers")
        print(f"camera pose: height {cal.height_m:.3f} m, tilt {cal.tilt_deg:.2f} deg, rms {cal.rms_px:.2f} px",
              flush=True)
        tdet = detector()                                 # hw_fig8's decoder settings for a blurred tag
        p = tdet.getDetectorParameters()
        p.perspectiveRemoveIgnoredMarginPerCell, p.errorCorrectionRate = 0.3, 1.0
        p.perspectiveRemovePixelPerCell, p.maxErroneousBitsInBorderRate = 8, 0.5
        tdet.setDetectorParameters(p)
        self.tracker = TagTracker(cam, arena, cal, [self.tag], det=tdet, monitor=False)
        # "unseen" counts from now, so a tag missing at the start waits --abort-s before the run gives up
        self.last, self.last_seen, self.last_ctrl = None, time.monotonic(), 0.0
        self.sent_a, self.sent_t = 0, -1e9

    def pose(self, tick):
        """Read frames until one control tick has passed; the newest sighting, and how old it is."""
        while True:
            f = self.src.read()
            now = time.monotonic()
            obs, _ = self.tracker.process(f.gray, f.frame_id, f.t_host)
            o = next((o for o in obs if o.tag_id == self.tag.tag_id and not (o.flags & self.F_JUMP)), None)
            if o is not None:
                self.last, self.last_seen = (o.x, o.y, o.yaw), now
            if self.link is not None:
                self.link.drain()
            if now - self.last_ctrl >= tick:
                self.last_ctrl = now
                return now, self.last, now - self.last_seen

    def send(self, a, L, R):
        """A line only when the command changes, or cmd_ms / 2 after the last one (as the calibration bridge did:
        the remote repeats every 100 ms by itself, and each extra line adds a ~70 ms frame on the 2400-baud link,
        which would change the delay measured in calibration)."""
        now = time.monotonic()
        if a == self.sent_a and (a == 0 or now - self.sent_t < self.cmd_ms / 2000):
            return
        if self.link is not None:
            self.link.send(f"pwm {128 + int(L)} {128 + int(R)} {self.cmd_ms}" if a != 0 else "s")
        self.sent_a, self.sent_t = a, now

    def stop(self):
        if self.link is not None:
            self.link.stop()
        self.sent_a, self.sent_t = 0, time.monotonic()

    def close(self):
        if self.link is not None:
            self.link.stop()
            self.link.close()
        if getattr(self, "src", None) is not None:
            self.src.close()
        self.rx_log.close()


# ------------------------------------------------------------------------------------------------------------- main
def roll(pose, t0, t1, sent, cells, delay):
    """Pose at t1 from the pose at t0 with the commands sent (t_send, action), each acting from t_send + delay
    until the next one does: brings a stale sighting up to now, or now to now + delay."""
    p = np.array(pose, float)
    for i, (ts, act) in enumerate(sent):
        a0 = max(t0, ts + delay)
        a1 = min(t1, sent[i + 1][0] + delay if i + 1 < len(sent) else t1)
        if a1 > a0:
            p = GreedyMap._arc(p, cells[act], a1 - a0)
    return tuple(p)


def inside(p):
    return ARENA[0, 0] <= p[0] <= ARENA[0, 1] and ARENA[1, 0] <= p[1] <= ARENA[1, 1]


def random_goal(rng, pose):
    while True:
        g = np.array([rng.uniform(ARENA[0, 0] + MARGIN, ARENA[0, 1] - MARGIN),
                      rng.uniform(ARENA[1, 0] + MARGIN, ARENA[1, 1] - MARGIN)])
        if pose is None or math.hypot(g[0] - pose[0], g[1] - pose[1]) >= 0.10:
            return g


class GoalTask:
    """Drive to one point."""

    def __init__(self, goal, tol, timeout):
        self.goal, self.tol, self.timeout = np.asarray(goal, float), tol, timeout
        self.label = f"goal ({goal[0]:.3f}, {goal[1]:.3f})"

    def start(self, now, pose):
        self.t0 = now

    def target(self, now):
        return self.goal

    def obs(self, pose, now, hist_LR):
        return make_obs(pose, self.goal, hist_LR)

    def outcome(self, dist, now):
        return "reached" if dist < self.tol else "timeout" if now - self.t0 > self.timeout else None


class TrackTask:
    """Follow a reference for `duration` seconds, entered at the point nearest the robot."""

    def __init__(self, ref, duration, label):
        self.ref, self.duration, self.label = ref, duration, label

    def start(self, now, pose):
        self.t0, self.phase = now, phase_near(self.ref, *pose)

    def t_ref(self, now):
        return self.phase + now - self.t0

    def target(self, now):
        return np.array([float(v) for v in self.ref.at(self.t_ref(now))[:2]])

    def obs(self, pose, now, hist_LR):
        return make_track_obs(pose, self.ref, self.t_ref(now), hist_LR)

    def outcome(self, dist, now):
        return "lost" if dist > LOST else "done" if now - self.t0 > self.duration else None


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mode", choices=("sim", "dry", "run"), default="sim")
    ap.add_argument("--task", choices=("goal", "track"), default="goal")
    ap.add_argument("--policy", help="default: runs/ppo_goal.zip, or runs/ppo_track.zip for --task track")
    ap.add_argument("--goals", help='"x,y x,y ..." in metres')
    ap.add_argument("--n-goals", type=int, default=5)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--goal-tol", type=float, default=0.03)
    ap.add_argument("--goal-timeout", type=float, default=30.0)
    ap.add_argument("--pause", type=float, default=1.5, help="stop between goals [s]")
    ap.add_argument("--shape", choices=("circle", "figure8"), default="circle")
    ap.add_argument("--radius", type=float, default=0.15, help="circle radius [m]")
    ap.add_argument("--ccw", action="store_true", help="circle: counter-clockwise (default clockwise)")
    ap.add_argument("--ax", type=float, default=0.25, help="figure-8 half width in x [m]")
    ap.add_argument("--ay", type=float, default=0.15, help="figure-8 half height in y [m]")
    ap.add_argument("--period", type=float, help="time per lap [s] (default: circle 20, figure-8 40)")
    ap.add_argument("--center", type=float, nargs=2, help="centre of the reference [m] (default: the arena's)")
    ap.add_argument("--loops", type=float, default=2.0)
    ap.add_argument("--tick", type=float, default=NOMINAL["tick"],
                    help="control period [s] (training: 0.12-0.16; --mode sim uses the env's own tick)")
    ap.add_argument("--cmd-ms", type=int, default=1000,
                    help="duration of each pwm line [ms], resent after half of it (the calibration bridge's 1000)")
    ap.add_argument("--delay", type=float, default=NOMINAL["delay"],
                    help="dead time used to bring stale sightings up to now and to look ahead for the geofence [s]")
    ap.add_argument("--unseen-stop", type=float, default=0.4)
    ap.add_argument("--abort-s", type=float, default=6.0)
    ap.add_argument("--max-min", type=float, default=10.0, help="stop the run after this many minutes")
    ap.add_argument("--randomize", action="store_true", help="--mode sim: a randomised plant")
    ap.add_argument("--ws", default=str(HERE.parents[1] / "brainbot_ws"), help="brainbot_ws checkout")
    ap.add_argument("--calib", help="camera calibration (default: the ws's calib_2026-11-12_interim_v4)")
    ap.add_argument("--robot", default="udZn9")
    ap.add_argument("--port", default="COM5", help="the IR sender's serial port")
    ap.add_argument("--exposure-us", type=float, default=18000)
    ap.add_argument("--gain-db", type=float, default=5.8)
    ap.add_argument("--out", default=str(HERE / "runs"))
    a = ap.parse_args()

    ref = None
    if a.task == "track":
        cx, cy = a.center or ARENA.mean(1)
        if a.shape == "circle":
            ref = Circle(cx, cy, a.radius, a.period or 20.0, 1 if a.ccw else -1)
            label = f"circle r {a.radius * 100:.0f} cm, {'counter-' if a.ccw else ''}clockwise, {ref.T:.0f} s per lap"
        else:
            ref = Figure8(cx, cy, a.ax, a.ay, a.period or 40.0)
            label = f"figure-8 {200 * a.ax:.0f} x {200 * a.ay:.0f} cm, {ref.T:.0f} s per lap"
        rx, ry, _, rv, _ = ref.at(np.linspace(0, ref.T, 721))
        label += f", {100 * rv.min():.1f}-{100 * rv.max():.1f} cm/s"
        if (rx.min() < ARENA[0, 0] + EDGE or rx.max() > ARENA[0, 1] - EDGE or ry.min() < ARENA[1, 0] + EDGE
                or ry.max() > ARENA[1, 1] - EDGE):
            raise SystemExit(f"the reference leaves the arena (it must stay {EDGE * 100:.0f} cm inside x "
                             f"{ARENA[0, 0]:.3f}..{ARENA[0, 1]:.3f}, y {ARENA[1, 0]:.3f}..{ARENA[1, 1]:.3f} m)")
        if rv.max() > 0.065:
            print(f"warning: the reference peaks at {100 * rv.max():.1f} cm/s; training went up to 6.2 and the "
                  f"robot's fastest measured cells do 7-8", flush=True)

    from stable_baselines3 import PPO
    policy = a.policy or str(HERE / "runs" / ("ppo_track.zip" if a.task == "track" else "ppo_goal.zip"))
    model = PPO.load(policy, device="cpu")
    LR, cells = load_cells()
    out = pathlib.Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    rig = SimRig(a.randomize, a.seed) if a.mode == "sim" else LiveRig(a, out, stamp)
    rig.cmd_ms = a.cmd_ms
    rng = np.random.default_rng(a.seed)
    if a.mode == "sim" and ref is not None:               # in sim, start 3 cm beside the reference, 0.3 rad off
        x0, y0, th0 = (float(v) for v in ref.at(0.0)[:3])
        rig.env.pose = np.array([x0 + 0.03, y0, th0 + 0.3])
    fixed = [np.array([float(v) for v in g.split(",")]) for g in a.goals.split()] if a.goals else None
    n_tasks = 1 if ref is not None else len(fixed) if fixed else a.n_goals

    stop_flag = {"stop": False}
    signal.signal(signal.SIGINT, lambda *_: stop_flag.__setitem__("stop", True))
    log_f = open(out / f"deploy_{a.task}_{a.mode}_{stamp}.csv", "w", newline="")
    log = csv.writer(log_f)
    log.writerow(["t", "task", "tx", "ty", "x", "y", "yaw", "pose_age", "dist", "action", "L", "R", "note"])
    print(f"{a.mode}: policy {policy}, tick {a.tick:.2f} s, arena x {ARENA[0, 0]:.3f}..{ARENA[0, 1]:.3f}"
          f" y {ARENA[1, 0]:.3f}..{ARENA[1, 1]:.3f} m", flush=True)

    t_run0, reason, results, errs = None, "all done", [], []
    try:
        now, pose, age = rig.pose(a.tick)
        for gi in range(n_tasks):
            if ref is not None:
                task = TrackTask(ref, a.loops * ref.T, label)
            else:
                task = GoalTask(fixed[gi] if fixed else random_goal(rng, pose), a.goal_tol, a.goal_timeout)
            hist = [0, 0]
            sent = [(-1e9, 0)]                            # (time sent, action), for roll()
            t_task, best, outcome = None, float("inf"), "aborted"
            print(f"{gi + 1}/{n_tasks}: {task.label}", flush=True)
            while not stop_flag["stop"]:
                now, pose, age = rig.pose(a.tick)
                t_run0 = now if t_run0 is None else t_run0
                if (now - t_run0) / 60 > a.max_min:
                    reason = f"{a.max_min:.0f} min run time reached (battery window)"
                    stop_flag["stop"] = True
                    break
                if pose is None or age > a.unseen_stop:
                    rig.stop()
                    hist = [0, 0]
                    sent.append((now, 0))
                    log.writerow([f"{now:.3f}", gi, *[""] * 10, f"tag unseen {age:.1f} s: stop"])
                    if age > a.abort_s:
                        reason = f"tag unseen for {a.abort_s:.0f} s"
                        stop_flag["stop"] = True
                        break
                    continue
                seen_pose = pose
                pose = roll(pose, now - age, now, sent, cells, a.delay)      # the sighting brought up to now
                ahead = roll(pose, now, now + a.delay, sent, cells, a.delay)  # where the commands in flight end
                x, y, yaw = pose
                if not (inside(seen_pose) and inside(pose) and inside(ahead)):
                    reason = (f"left the arena: seen ({seen_pose[0]:.3f}, {seen_pose[1]:.3f}), now ({x:.3f}, "
                              f"{y:.3f}), in {a.delay:.2f} s ({ahead[0]:.3f}, {ahead[1]:.3f})")
                    stop_flag["stop"] = True
                    break
                if t_task is None:
                    t_task = now
                    task.start(now, pose)
                tx, ty = task.target(now)
                dist = math.hypot(tx - x, ty - y)
                best = min(best, dist)
                if ref is not None and now - t_task > 3.0:
                    errs.append(dist)
                outcome = task.outcome(dist, now)
                if outcome is not None:
                    if outcome == "lost":
                        reason = f"the error passed {LOST * 100:.0f} cm"
                        stop_flag["stop"] = True
                    break
                outcome = "aborted"
                act = int(model.predict(task.obs(pose, now, LR[hist]), deterministic=True)[0])
                L, R = LR[act].astype(int)
                rig.send(act, L, R)
                hist = hist[1:] + [act]
                sent = sent[-8:] + [(now, act)]
                log.writerow([f"{now:.3f}", gi, f"{tx:.4f}", f"{ty:.4f}", f"{x:.5f}", f"{y:.5f}",
                              f"{yaw:.5f}", f"{age:.3f}", f"{dist:.4f}", act, L, R, ""])
            rig.stop()
            took = now - t_task if t_task is not None else float("nan")
            results.append((outcome, took, best))
            if ref is None:
                print(f"  {outcome} after {took:.1f} s, closest {best * 100:.1f} cm", flush=True)
            log.writerow([f"{now:.3f}", gi, *[""] * 10, f"{outcome} after {took:.1f} s"])
            if stop_flag["stop"]:
                break
            t_pause = time.monotonic()
            while a.mode != "sim" and time.monotonic() - t_pause < a.pause:
                now, pose, age = rig.pose(a.tick)
    finally:
        rig.close()
        log_f.close()
    if ref is not None:
        e = 100 * np.array(errs) if errs else np.array([np.nan])
        print(f"end: {reason}; {results[-1][0] if results else 'nothing run'} after "
              f"{results[-1][1] if results else 0:.0f} s; error median {np.median(e):.1f} cm, rms "
              f"{np.sqrt(np.mean(e ** 2)):.1f} cm, max {e.max():.1f} cm; log {log_f.name}", flush=True)
    else:
        ok = [r for r in results if r[0] == "reached"]
        print(f"end: {reason}; reached {len(ok)}/{len(results)}"
              + (f", median {np.median([r[1] for r in ok]):.1f} s" if ok else "") + f"; log {log_f.name}", flush=True)


if __name__ == "__main__":
    main()
