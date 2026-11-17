"""Correct the simulator's plant from robot logs (deploy.py's csv), the way Swift did for drone racing (Kaufmann et
al., Nature 2023): fly, fit what the simulator got wrong, add it to the simulator, fine-tune the policy there.

What is fitted, from the sightings between consecutive control ticks:
- the twist (u1, u2, omega) of every grid cell the run used, in place of the calibration map's value;
- a start-up twist per cell: what is added during the first START seconds after the command changed to that cell
  (negative forward values = the robot is slow to get going; this is what one-tick bursts depend on);
- the dead time (the value that leaves the smallest residual) and the noise per tick (what is left);
- for the cells the run did not use: the map's forward speed times the ratio found on the used cells (an
  extrapolation: a slow day slows every cell, not only the ones that were driven). On 2026-11-17 this was only
  half right: (80, 90) and (0, 80) ran 20-25 % slow, but (120, 100) and (90, 120), driven later that day, ran at
  the map's speed. Refit after every run so the cells a new policy uses get measured.
Each interval's measured twist is modelled as the time-weighted mix of the cells in force during it, so intervals
that straddle a command change are used too. Cells are pulled towards the map and start-up terms towards zero with
the weight of PRIOR intervals, so a cell seen for a moment barely moves.

python rl_walk/residual.py runs/deploy_track_run_*.csv --out runs/residual.json
then:  python rl_walk/train_track.py --name track_ft --init track --residual runs/residual.json --steps 300000
       python rl_walk/deploy.py --mode run --task track --policy runs/ppo_track_ft.zip ...

Checked in simulation only (2026-11-16), on simulated robots whose plant was hidden from the fit, from about 3 min
of the tracking policy's own logs: the fit recovered the dead time within 0.02 s, the noise level, the used cells'
speeds within 0.1-0.4 cm/s (1 cm/s where the robot was far too slow for the reference) and most of a slow start.
Fine-tuning then helped only where the hidden error broke the policy's habits: with slow starts the figure-8 error
went 1.4 -> 1.0 cm median and a counter-clockwise circle 6.3 -> 3.2 cm; with errors inside the training ranges, or
with a robot too slow for the reference, it changed nothing reliable. So read the fit first, fine-tune second.

Limits: only cells and changes the logged runs contain are corrected; a few minutes of one day on one robot; the
dead time found here is from closed-loop data and is only meaningful because the runs stay on the measured cells.
"""
import argparse
import csv
import json

import numpy as np

from env import NOISE_SD, NOMINAL, START, load_cells

PRIOR = 5.0   # weight of the map (cells) and of "no start-up effect", in intervals
STEP = 0.005  # sampling of the command timeline [s]


def load_log(path):
    """Sightings (time, pose) and the commands sent (time, action); a row without an action is a stop."""
    rows = list(csv.DictReader(open(path)))
    sight, cmd = [], []
    for r in rows:
        t = float(r["t"])
        if r["action"] == "":
            cmd.append((t, 0))
            continue
        cmd.append((t, int(r["action"])))
        if r.get("sx"):
            sight.append((t - float(r["pose_age"]), float(r["sx"]), float(r["sy"]), float(r["syaw"])))
        else:                                              # logs from before the sighting was written
            sight.append((t, float(r["x"]), float(r["y"]), float(r["yaw"])))
    sight = np.array(sorted(set(sight)))
    return sight, np.array([c[0] for c in cmd]), np.array([c[1] for c in cmd])


def intervals(sight):
    """Measured body twist between consecutive sightings 0.05-0.4 s apart: (t0, t1, twist)."""
    t, x, y, th = sight.T
    dt = np.diff(t)
    ok = (dt > 0.05) & (dt < 0.4)
    dth = (np.diff(th) + np.pi) % (2 * np.pi) - np.pi
    mid = th[:-1] + dth / 2
    dx, dy = np.diff(x), np.diff(y)
    tw = np.stack([(np.cos(mid) * dx + np.sin(mid) * dy) / dt, (-np.sin(mid) * dx + np.cos(mid) * dy) / dt,
                   dth / dt], 1)
    return t[:-1][ok], t[1:][ok], tw[ok]


def design(t0, t1, t_cmd, a_cmd, delay, n_cells):
    """Per interval: the share of its time each cell is in force (F), and the share it is in force and new (G)."""
    # the time each command's cell came in: the first command of its run of equal actions
    run_start = t_cmd.copy()
    for k in range(1, len(t_cmd)):
        if a_cmd[k] == a_cmd[k - 1]:
            run_start[k] = run_start[k - 1]
    F, G = np.zeros((len(t0), n_cells)), np.zeros((len(t0), n_cells))
    for i, (a, b) in enumerate(zip(t0, t1)):
        ts = np.arange(a + STEP / 2, b, STEP)
        k = np.searchsorted(t_cmd + delay, ts, side="right") - 1
        act = np.where(k >= 0, a_cmd[np.clip(k, 0, None)], 0)
        new = (k >= 0) & (ts - (run_start[np.clip(k, 0, None)] + delay) < START)
        np.add.at(F[i], act, 1.0 / len(ts))
        np.add.at(G[i], act[new], 1.0 / len(ts))
    return F, G


def fit(logs, delay, cells_map):
    """Ridge fit of cell and start-up twists at one dead time. Returns (cells, start, residual sd, counts, sse)."""
    n = len(cells_map)
    Fs, Gs, Ms = [], [], []
    for sight, t_cmd, a_cmd in logs:
        t0, t1, tw = intervals(sight)
        F, G = design(t0, t1, t_cmd, a_cmd, delay, n)
        Fs.append(F), Gs.append(G), Ms.append(tw)
    F, G, M = np.vstack(Fs), np.vstack(Gs), np.vstack(Ms)
    used = np.flatnonzero((F.sum(0) > 0) & (np.arange(n) != 0))        # the stop cell stays at zero
    A = np.hstack([F[:, used], G[:, used]])
    prior = np.vstack([cells_map[used], np.zeros((len(used), 3))])
    beta = np.linalg.solve(A.T @ A + PRIOR * np.eye(A.shape[1]), A.T @ M + PRIOR * prior)
    res = M - A @ beta
    cells = {int(c): beta[j].tolist() for j, c in enumerate(used)}
    start = {int(c): beta[len(used) + j].tolist() for j, c in enumerate(used)}
    counts = {int(c): [float(F[:, c].sum()), float(G[:, c].sum())] for c in used}
    moving = F[:, 0] < 0.5                                              # the noise level while driving
    return cells, start, res[moving].std(0), counts, float(np.sum((res / NOISE_SD) ** 2)), len(M)


def fit_logs(paths, delays=np.arange(0.05, 0.501, 0.01)):
    LR, cells_map = load_cells()
    logs = [load_log(p) for p in paths]
    sse = [fit(logs, d, cells_map)[4] for d in delays]
    delay = float(delays[int(np.argmin(sse))])
    cells, start, sd, counts, _, n = fit(logs, delay, cells_map)
    # the cells the runs did not use: the map's value times the forward-speed ratio of the cells they did use
    # (weighted by use), turn rate as mapped. Without this the unvisited cells keep the map's speed and look better
    # than the corrected ones for no reason, and a fine-tuned policy would move to them.
    w = np.array([counts[c][0] for c in cells])
    fit_u, map_u = np.array([cells[c][0] for c in cells]), np.array([cells_map[c][0] for c in cells])
    gain = float(np.sum(w * fit_u * map_u) / np.sum(w * map_u ** 2))
    other = {c: [gain * cells_map[c][0], gain * cells_map[c][1], float(cells_map[c][2])]
             for c in range(1, len(cells_map)) if c not in cells}
    return dict(delay=delay, start_window=START, noise_sd=sd.tolist(), forward_gain=gain, cells=cells, start=start,
                other_cells=other, counts=counts, n_intervals=n, logs=[str(p) for p in paths]), LR, cells_map


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("logs", nargs="+")
    ap.add_argument("--out", default="runs/residual.json")
    a = ap.parse_args()
    r, LR, cells_map = fit_logs(a.logs)
    json.dump(r, open(a.out, "w"), indent=1)
    sd = r["noise_sd"]
    print(f"{r['n_intervals']} intervals from {len(a.logs)} logs; dead time {r['delay']:.2f} s (model "
          f"{NOMINAL['delay']:.2f}); noise per tick {100 * sd[0]:.1f} cm/s, {100 * sd[1]:.1f} cm/s, {sd[2]:.2f} rad/s "
          f"(model {100 * NOISE_SD[0]:.1f}, {100 * NOISE_SD[1]:.1f}, {NOISE_SD[2]:.2f})")
    print("cell (L, R)   intervals  new   forward cm/s map -> fit   turn rad/s map -> fit   start-up: forward, turn")
    for c, tw in sorted(r["cells"].items(), key=lambda kv: -r["counts"][kv[0]][0]):
        m, s, (nf, ng) = cells_map[c], r["start"][c], r["counts"][c]
        print(f"({int(LR[c][0]):3d}, {int(LR[c][1]):3d})   {nf:7.1f} {ng:6.1f}      {100 * m[0]:5.1f} -> {100 * tw[0]:5.1f}"
              f"            {m[2]:+5.2f} -> {tw[2]:+5.2f}        {100 * s[0]:+5.1f} cm/s, {s[2]:+5.2f} rad/s")
    print(f"the other {len(r['other_cells'])} cells: forward speed x{r['forward_gain']:.2f} of the map (the used cells' "
          f"ratio), turn rate as mapped")
    print(f"wrote {a.out}")


if __name__ == "__main__":
    main()
