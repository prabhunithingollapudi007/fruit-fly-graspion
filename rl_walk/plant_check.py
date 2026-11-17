"""Replay the PWM commands logged in hw_fig8 runs open loop through grid map + delay + lag and compare with the
tracked pose at 1 s and 3 s (the sim-to-real gate of FEASIBILITY.md §5.4; multi-step prediction error as in
Seegmiller et al. 2013). Delay and lag come from step_fit.py, not from these runs (--fit shows why).

What it supports: on the one run that stays on the measured map (20261112_152628, 95 s), the plant predicts the
robot far better than standing still. One run, one day, one robot; the 11-11 runs drive off the map.

Plant: the twist (u1, u2, omega) of the tracked point is the grid map at the last PWM pair sent at least `delay`
before, passed through a first-order lag `tau`, scaled by gains (gv, gw). Battery is ideal (map at nominal).
python rl_walk/plant_check.py [--map data/grid_map_combined.json] [--fit] [RUN.csv ...]
"""
import argparse
import csv
import glob
import os
import sys

import numpy as np
from scipy.signal import lfilter

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "calibration", "pc"))
from graspion_calib.grid_map import GridMap  # noqa: E402

DT = 0.005  # integration step [s]


def load_run(path, gmap):
    rows = list(csv.DictReader(open(path)))
    f = lambda k: np.array([float(r[k]) if r[k] not in ("", "nan") else np.nan for r in rows])
    t, x, y, yaw = f("t"), f("x"), f("y"), f("yaw")
    seen = np.array([r["seen"] == "1" for r in rows])
    # drive = |PWM byte - 128| (reverse acts like forward on this robot, as in GridMap.pwm_to_twist)
    drv = np.array([[0 if r["en"] != "1" else abs(int(r[k]) - 128) for k in ("pwm_l", "pwm_r")] for r in rows])
    full = gmap.predict(drv[:, 0], drv[:, 1])
    full[drv.sum(1) == 0] = 0.0
    off = ((drv > 0) & (drv < 55)) | (drv > 120)
    return dict(name=os.path.basename(path), t=t, x=x, y=y, yaw=yaw, seen=seen, cmd=full,
                outside=float(off.any(1).mean()))


def twist_series(run, delay, tau, t0, t1):
    """Filtered twist on a DT grid over [t0, t1], starting from the steady state of the command in force at t0."""
    tg = np.arange(t0, t1 + 1e-9, DT)
    k = np.clip(np.searchsorted(run["t"], tg - delay, side="right") - 1, 0, len(run["t"]) - 1)
    raw = run["cmd"][k]
    raw[tg - delay < run["t"][0]] = 0.0
    if tau <= 0:
        return tg, raw
    a = DT / (tau + DT)
    out, _ = lfilter([a], [1, a - 1], raw, axis=0, zi=np.outer([1 - a], raw[0]))
    return tg, out


def integrate(pose, tw, gains):
    x, y, th0 = pose
    gv, gw = gains
    if len(tw) == 0:
        return x, y, th0
    th = th0 + np.concatenate(([0.0], np.cumsum(gw * tw[:-1, 2] * DT)))
    c, s = np.cos(th), np.sin(th)
    x += np.sum(gv * (tw[:, 0] * c - tw[:, 1] * s)) * DT
    y += np.sum(gv * (tw[:, 0] * s + tw[:, 1] * c)) * DT
    return x, y, th[-1] + gw * tw[-1, 2] * DT


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def full_filtered(run, delay, tau):
    """Filtered twist over the whole run (computed once, so the lag carries across intervals)."""
    return twist_series(run, delay, tau, run["t"][0], run["t"][-1] + 0.5)


def one_step(runs, delay, tau, gains=(1, 1)):
    """Residuals of the measured vs predicted pose change between consecutive sightings: body-frame dx, dy [m]
    per second and dyaw per second."""
    res = []
    for r in runs:
        tg, tw = full_filtered(r, delay, tau)
        t, s = r["t"], r["seen"]
        for i in range(len(t) - 1):
            if not (s[i] and s[i + 1]) or not 0.05 < t[i + 1] - t[i] < 0.4:
                continue
            a, b = np.searchsorted(tg, [t[i], t[i + 1]])
            p = integrate((r["x"][i], r["y"][i], r["yaw"][i]), tw[a:b], gains)
            dt = t[i + 1] - t[i]
            c, sn = np.cos(r["yaw"][i]), np.sin(r["yaw"][i])
            ex, ey = r["x"][i + 1] - p[0], r["y"][i + 1] - p[1]
            res.append([(c * ex + sn * ey) / dt, (-sn * ex + c * ey) / dt, wrap(r["yaw"][i + 1] - p[2]) / dt])
    return np.array(res)


def replay(runs, delay, tau, gains, horizon):
    """Open-loop replay from every sighting: position and heading error after `horizon` seconds."""
    pe, he = [], []
    for r in runs:
        tg, tw = full_filtered(r, delay, tau)
        t, s = r["t"], r["seen"]
        for i in range(len(t)):
            j = np.searchsorted(t, t[i] + horizon)
            if j >= len(t) or not (s[i] and s[j]) or abs(t[j] - t[i] - horizon) > 0.1:
                continue
            a, b = np.searchsorted(tg, [t[i], t[j]])
            p = integrate((r["x"][i], r["y"][i], r["yaw"][i]), tw[a:b], gains)
            pe.append(np.hypot(r["x"][j] - p[0], r["y"][j] - p[1]))
            he.append(abs(wrap(r["yaw"][j] - p[2])))
    return np.array(pe), np.array(he)


def noise_check(run, delay, tau, sig, horizon, tick=0.14, seed=0):
    """Same replay, but against the noiseless sim instead of the robot: the sim's own error when white noise
    (u1, u2, w sd = sig) is added to the twist, held for one control tick. Compare with replay()."""
    rng = np.random.default_rng(seed)
    tg, tw = full_filtered(run, delay, tau)
    t, s = run["t"], run["seen"]
    hold = int(round(tick / DT))
    pe, he = [], []
    for i in range(len(t)):
        j = np.searchsorted(t, t[i] + horizon)
        if j >= len(t) or not (s[i] and s[j]):
            continue
        a, b = np.searchsorted(tg, [t[i], t[j]])
        nz = np.repeat(rng.normal(0, sig, (-(-(b - a) // hold), 3)), hold, axis=0)[: b - a]
        p0 = integrate((0.0, 0.0, 0.0), tw[a:b], (1, 1))
        p1 = integrate((0.0, 0.0, 0.0), tw[a:b] + nz, (1, 1))
        pe.append(np.hypot(p1[0] - p0[0], p1[1] - p0[1]))
        he.append(abs(wrap(p1[2] - p0[2])))
    return np.array(pe), np.array(he)


def cost(res, sv=0.01, sw=0.3):
    return np.mean((res[:, 0] / sv) ** 2 + (res[:, 1] / sv) ** 2 + (res[:, 2] / sw) ** 2)


def fit(runs):
    best = None
    for delay in np.arange(0.0, 0.81, 0.05):
        for tau in (0.0, 0.05, 0.1, 0.2, 0.3, 0.5):
            c = cost(one_step(runs, delay, tau))
            if best is None or c < best[0]:
                best = (c, delay, tau)
    _, d0, t0 = best
    for delay in np.arange(max(0, d0 - 0.06), d0 + 0.061, 0.02):
        for tau in np.unique(np.clip(t0 + np.array([-0.06, -0.03, 0, 0.03, 0.06]), 0, None)):
            c = cost(one_step(runs, delay, tau))
            if c < best[0]:
                best = (c, delay, tau)
    _, delay, tau = best
    # gains by least squares on the one-step change: measured = g * predicted
    res1 = one_step(runs, delay, tau)
    pred0 = one_step(runs, delay, tau, gains=(0, 0))  # = measured (relative to holding still)
    meas = pred0
    pred = meas - res1
    gv = np.sum(meas[:, 0] * pred[:, 0]) / np.sum(pred[:, 0] ** 2)
    gw = np.sum(meas[:, 2] * pred[:, 2]) / np.sum(pred[:, 2] ** 2)
    return delay, tau, (gv, gw), best[0]


def report(label, runs, delay, tau, gains):
    r1 = one_step(runs, delay, tau, gains)
    line = f"{label:34s} one-step rms u1 {100*np.sqrt(np.mean(r1[:,0]**2)):.2f} cm/s, u2 " \
           f"{100*np.sqrt(np.mean(r1[:,1]**2)):.2f} cm/s, w {np.sqrt(np.mean(r1[:,2]**2)):.2f} rad/s"
    for h in (1.0, 3.0):
        pe, he = replay(runs, delay, tau, gains, h)
        line += f" | {h:.0f}s: pos med/p90 {100*np.median(pe):.1f}/{100*np.percentile(pe,90):.1f} cm," \
                f" yaw med {np.degrees(np.median(he)):.0f} deg"
    print(line)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="*")
    ap.add_argument("--map", default=os.path.join(HERE, "data", "grid_map_combined.json"))
    ap.add_argument("--delay", type=float, default=0.17, help="dead time [s] (step_fit.py: 0.16-0.18)")
    ap.add_argument("--tau", type=float, default=0.02, help="lag [s] (step_fit.py: <= 0.06)")
    ap.add_argument("--fit", action="store_true",
                    help="also fit delay, lag and gains from these closed-loop runs. Not reliable: where a run "
                         "leaves the measured map, the map's error is absorbed into the timing (pooled: 0.80 s, "
                         "the edge of the search grid)")
    a = ap.parse_args()
    paths = a.runs or sorted(glob.glob(os.path.join(HERE, "data", "run_*.csv")))
    gmap = GridMap.load(a.map)
    runs = [load_run(p, gmap) for p in paths]
    print(f"map {os.path.basename(a.map)}, {len(runs)} runs, delay {a.delay:.2f} s, tau {a.tau:.2f} s")
    print("'off map' = share of ticks with a drive in the dead-zone bridge (1-54) or above 120, where the map is an "
          "extrapolation; only runs that stay on the map test the plant")
    for r in runs:
        report(f"{r['name'][4:-4]} off map {100 * r['outside']:.0f} %", [r], a.delay, a.tau, (1, 1))
        report("  hold still", [r], a.delay, a.tau, (0, 0))
    sig = np.array([0.013, 0.007, 0.65])  # the env's per-tick noise: the one-step rms of run 20261112_152628
    for r in [r for r in runs if r["outside"] < 0.05]:
        print(f"noise check on {r['name']} (the run the noise level was taken from, so in-sample): noisy sim vs "
              f"noiseless sim")
        for scale in (0.5, 1.0):
            for h in (1.0, 3.0):
                pe, he = noise_check(r, a.delay, a.tau, scale * sig, h)
                print(f"  noise x{scale} {h:.0f}s: pos med/p90 {100*np.median(pe):.1f}/{100*np.percentile(pe,90):.1f} cm,"
                      f" yaw med {np.degrees(np.median(he)):.0f} deg")
    if a.fit:
        delay, tau, gains, _ = fit(runs)
        print(f"closed-loop fit (not reliable, see --help): delay {delay:.2f} s, tau {tau:.2f} s, gains v "
              f"{gains[0]:.2f} w {gains[1]:.2f}")
        for r in runs:
            d, t, g, _ = fit([r])
            print(f"  {r['name']}: own fit delay {d:.2f} tau {t:.2f} gains {g[0]:.2f}/{g[1]:.2f}")


if __name__ == "__main__":
    main()
