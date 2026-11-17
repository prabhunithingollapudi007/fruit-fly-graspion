"""Figures of the hardware runs of 2026-11-17 (udZn9), drawn from the logs in runs/ into figures/.

python rl_walk/make_figures.py

Error everywhere = distance from the tag to the reference point of the moment, after the first 3 s (the MPC logs:
reference time t_ref - delay). Colours follow the controller, the same in every figure: blue = RL as trained in
simulation, orange = the lab's MPC, aqua = RL fine-tuned on the day's fitted plant; the reference is grey.
Needs matplotlib.
"""
import csv
import glob
import json
import os
import time

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from env import ARENA
from track_env import Figure8, PathRef

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS, OUT = os.path.join(HERE, "runs"), os.path.join(HERE, "figures")
SURFACE, INK, INK2, MUTED, GRID, AXIS = "#fcfcfb", "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#c3c2b7"
RL, MPC, FT = "#2a78d6", "#eb6834", "#1baf7a"     # categorical slots 1-3 (validated all-pairs on this surface)
CX, CY = ARENA.mean(1)
plt.rcParams.update({"font.family": ["Segoe UI", "DejaVu Sans"], "font.size": 10, "text.color": INK,
                     "axes.edgecolor": AXIS, "axes.labelcolor": INK2, "xtick.color": MUTED, "ytick.color": MUTED,
                     "figure.facecolor": SURFACE, "axes.facecolor": SURFACE, "savefig.facecolor": SURFACE,
                     "axes.spines.top": False, "axes.spines.right": False, "axes.titlesize": 11,
                     "axes.titleweight": "regular", "axes.titlelocation": "left"})


def rl_run(name):
    ok = [r for r in csv.DictReader(open(os.path.join(RUNS, name))) if r["sx"]]
    t = np.array([float(r["t"]) for r in ok])
    return dict(t=t - t[0], p=np.array([[float(r["sx"]), float(r["sy"])] for r in ok]),
                ref=np.array([[float(r["tx"]), float(r["ty"])] for r in ok]))


def mpc_run(name, delay):
    rows = [r for r in csv.DictReader(open(os.path.join(RUNS, "hw_fig8", name))) if r["x_ref"] and r["seen"] == "1"]
    t = np.array([float(r["t"]) for r in rows])
    tr = np.array([float(r["t_ref"]) for r in rows])
    ref = np.column_stack(Figure8(CX, CY, 0.25, 0.15, 40.0).at(tr - delay)[:2])
    return dict(t=t - t[0], p=np.array([[float(r["x"]), float(r["y"])] for r in rows]), ref=ref)


def stats(run, t_max=80.0):
    e = 100 * np.hypot(*(run["p"] - run["ref"]).T)
    k = (run["t"] > 3) & (run["t"] <= t_max)
    run["err"] = e
    run["median"], run["rms"], run["max"] = np.median(e[k]), np.sqrt(np.mean(e[k] ** 2)), e[k].max()
    return run


def fig8_path(ax_m, ay_m, n=400):
    x, y = Figure8(CX, CY, ax_m, ay_m, 1.0).at(np.linspace(0, 1, n))[:2]
    return 100 * (x - CX), 100 * (y - CY)


def path_panel(ax, run, color, title, size=(0.25, 0.15), lim=(36, 24)):
    """One run from above: the reference in grey, the tag's path in the controller's colour, in cm from the centre."""
    rx, ry = fig8_path(*size)
    ax.plot(rx, ry, color=AXIS, lw=3.0, solid_capstyle="round", zorder=1)
    k = run["t"] > 3
    ax.plot(100 * (run["p"][k, 0] - CX), 100 * (run["p"][k, 1] - CY), color=color, lw=1.6, zorder=2)
    ax.set_aspect("equal")
    ax.set_xlim(-lim[0], lim[0])
    ax.set_ylim(-lim[1], lim[1])
    ax.set_xticks([])
    ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(False)
    ax.set_title(title, color=INK, pad=18)
    ax.text(0, 1.02, f"median {run['median']:.1f} cm  ·  max {run['max']:.1f} cm", transform=ax.transAxes,
            color=INK2, fontsize=9.5, va="bottom")


def scale_bar(ax, lim, length=10):
    x0, y0 = -lim[0] + 1, -lim[1] + 1.5
    ax.plot([x0, x0 + length], [y0, y0], color=INK2, lw=1.2)
    ax.text(x0 + length / 2, y0 + 1.0, f"{length} cm", color=INK2, fontsize=9, ha="center", va="bottom")


def save(fig, name):
    os.makedirs(OUT, exist_ok=True)
    fig.savefig(os.path.join(OUT, name), dpi=170, bbox_inches="tight", pad_inches=0.25)
    plt.close(fig)
    print("wrote", os.path.join("figures", name))


def main():
    rl = stats(rl_run("deploy_track_run_20261117_152104.csv"))
    mpc = stats(mpc_run("run_20261117_152351.csv", 0.45))
    ft = stats(rl_run("deploy_track_run_20261117_153111.csv"))
    three = ((rl, RL, "RL, as trained in simulation"), (mpc, MPC, "MPC, settings of 12 November"),
             (ft, FT, "RL, fine-tuned on the day's plant"))

    # 1. the benchmark figure-8 from above, one panel per controller
    fig, axes = plt.subplots(1, 3, figsize=(12.6, 3.6))
    for ax, (run, color, title) in zip(axes, three):
        path_panel(ax, run, color, title)
    scale_bar(axes[0], (36, 24))
    fig.suptitle("Figure-8 on the robot, 50 × 30 cm in 40 s, two laps: grey is the reference, colour is the robot",
                 x=0.125, y=1.08, ha="left", fontsize=12.5, color=INK)
    save(fig, "fig8_three_controllers.png")

    # 2. the same three runs: error against time
    fig, ax = plt.subplots(figsize=(9.6, 3.9))
    for run, color, label in three:
        k = run["t"] <= 80
        ax.plot(run["t"][k], run["err"][k], color=color, lw=1.8, label=label)
    # direct labels at the line ends, kept at least 0.9 cm apart (the lines end close together)
    ends = sorted(((run["err"][run["t"] <= 80][-1], color, label) for run, color, label in
                   ((mpc, MPC, "MPC"), (rl, RL, "RL as trained"), (ft, FT, "RL fine-tuned"))))
    y_last = -1.0
    for y_end, color, label in ends:
        y_lab = max(y_end, y_last + 0.9)
        ax.plot([80.3, 81.6], [y_end, y_lab], color=color, lw=1.0)
        ax.text(82, y_lab, label, color=INK, fontsize=9.5, va="center")
        y_last = y_lab
    ax.axvspan(0, 3, color=GRID, alpha=0.6, lw=0)
    ax.text(3.6, 11.6, "first 3 s: not scored", color=MUTED, fontsize=8.5, ha="left", va="top")
    ax.set_xlim(0, 94)
    ax.set_ylim(0, 12)
    ax.set_xticks(range(0, 81, 10))
    ax.set_xlabel("time into the run [s]")
    ax.set_ylabel("distance to the reference point [cm]")
    ax.grid(axis="y", color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    ax.legend(frameon=False, loc="lower left", ncol=3, bbox_to_anchor=(-0.01, 1.0), fontsize=9.5)
    ax.set_title("Tracking error over the two laps of the 50 × 30 cm figure-8", pad=34, fontsize=12.5)
    save(fig, "fig8_error_over_time.png")

    # 3. other figure-8 sizes with the fine-tuned policy unchanged
    sizes = (("50 × 30 cm in 40 s (the benchmark)", (0.25, 0.15), "deploy_track_run_20261117_153111.csv"),
             ("30 × 20 cm in 28 s", (0.15, 0.10), "deploy_track_run_20261117_153602.csv"),
             ("64 × 40 cm in 53 s, larger than any trained", (0.32, 0.20), "deploy_track_run_20261117_153717.csv"),
             ("24 × 16 cm in 22 s, tighter than any trained", (0.12, 0.08), "deploy_track_run_20261117_153827.csv"))
    fig, axes = plt.subplots(2, 2, figsize=(10.4, 7.2))
    for ax, (title, size, name) in zip(axes.ravel(), sizes):
        path_panel(ax, stats(rl_run(name), t_max=1e9), FT, title, size=size, lim=(36, 24))
    scale_bar(axes[0, 0], (36, 24))
    fig.suptitle("The fine-tuned policy on four figure-8 sizes, without retraining (same scale in every panel)",
                 x=0.125, y=0.98, ha="left", fontsize=12.5, color=INK)
    fig.subplots_adjust(hspace=0.32, wspace=0.08)
    save(fig, "fig8_sizes_no_retraining.png")

    # 4. every tracking run: median error (bar) and maximum (dot)
    rows = (("RL as trained  ·  circle r 15 cm", stats(rl_run("deploy_track_run_20261117_151904.csv"), 1e9), RL),
            ("RL as trained  ·  figure-8 50 × 30 cm", rl, RL),
            ("MPC, 12 Nov settings  ·  figure-8 50 × 30 cm", mpc, MPC),
            ("MPC, map at 4.14 V  ·  same, after a recharge", stats(mpc_run("run_20261117_160110.csv", 0.24)), MPC),
            ("RL fine-tuned  ·  figure-8 50 × 30 cm", ft, FT),
            ("RL fine-tuned  ·  same, after a recharge", stats(rl_run("deploy_track_run_20261117_155939.csv")), FT),
            ("RL fine-tuned  ·  figure-8 30 × 20 cm", stats(rl_run("deploy_track_run_20261117_153602.csv"), 1e9), FT),
            ("RL fine-tuned  ·  figure-8 64 × 40 cm", stats(rl_run("deploy_track_run_20261117_153717.csv"), 1e9), FT),
            ("RL fine-tuned  ·  figure-8 24 × 16 cm", stats(rl_run("deploy_track_run_20261117_153827.csv"), 1e9), FT))
    fig, ax = plt.subplots(figsize=(10.4, 4.9))
    y = np.arange(len(rows))[::-1]
    for yi, (label, run, color) in zip(y, rows):
        ax.barh(yi, run["median"], height=0.56, color=color, zorder=2)
        ax.plot([run["max"]], [yi], "o", ms=7, color=color, mec=SURFACE, mew=1.5, zorder=3)
        ax.plot([run["median"], run["max"]], [yi, yi], color=color, lw=1.0, alpha=0.45, zorder=1)
        ax.text(run["max"] + 0.25, yi, f"{run['median']:.1f}  /  {run['max']:.1f} cm", color=INK2, fontsize=9, va="center")
    ax.set_yticks(y)
    ax.set_yticklabels([r[0] for r in rows], color=INK)
    ax.tick_params(axis="y", length=0)
    ax.set_xlim(0, 13.6)
    ax.set_xlabel("distance to the reference point [cm]")
    ax.grid(axis="x", color=GRID, lw=0.8)
    ax.set_axisbelow(True)
    ax.spines["left"].set_visible(False)
    handles = [plt.Rectangle((0, 0), 1, 1, color=c) for c in (RL, MPC, FT)]
    ax.legend(handles, ["RL as trained in simulation", "MPC", "RL fine-tuned on the day's plant"], frameon=False,
              loc="lower right", fontsize=9.5)
    ax.set_title("Every tracking run of 17 November: bar = median error, dot = maximum", pad=12, fontsize=12.5)
    save(fig, "tracking_runs_summary.png")

    # 5. goal reaching: the five goals of the first run, from above
    rows = [r for r in csv.DictReader(open(os.path.join(RUNS, "deploy_goal_run_20261117_151541.csv"))) if r["sx"]]
    fig, ax = plt.subplots(figsize=(8.4, 6.2))
    (x0, x1), (y0, y1) = 100 * ARENA
    ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ec=AXIS, lw=1.0))
    ax.text(x0 + 1, y1 - 1, "arena the policy trained in", color=MUTED, fontsize=9, va="top")
    every = 100 * np.array([[float(r["sx"]), float(r["sy"])] for r in rows])
    ax.text(every[0, 0], every[0, 1] + 1.6, "start", color=INK2, fontsize=9.5, ha="center", va="bottom")
    for g in sorted({int(r["task"]) for r in rows}):
        seg = [r for r in rows if int(r["task"]) == g]
        p = 100 * np.array([[float(r["sx"]), float(r["sy"])] for r in seg])
        gx, gy = 100 * float(seg[0]["tx"]), 100 * float(seg[0]["ty"])
        ax.plot(p[:, 0], p[:, 1], color=RL, lw=2.0, zorder=2)
        ax.plot(*p[0], "o", ms=5, color=RL, mec=SURFACE, mew=1.2, zorder=3)
        ax.add_patch(plt.Circle((gx, gy), 3.0, fill=False, ec=INK2, lw=1.2, zorder=3))
        took = float(seg[-1]["t"]) - float(seg[0]["t"])
        # the label goes on the side of the ring that is farthest from every driven path
        sides = {"left": (-4.2, 0, "right", "center", (-10, 0)), "right": (4.2, 0, "left", "center", (10, 0)),
                 "below": (0, -4.4, "center", "top", (0, -6)), "above": (0, 4.4, "center", "bottom", (0, 6))}
        clear = {k: np.min(np.hypot(every[:, 0] - gx - v[4][0], every[:, 1] - gy - v[4][1])) for k, v in sides.items()}
        dx, dy, ha, va, _ = sides[max(clear, key=clear.get)]
        ax.text(gx + dx, gy + dy, f"goal {g + 1}: {took:.1f} s", color=INK, fontsize=9.5, ha=ha, va=va)
    ax.set_aspect("equal")
    ax.set_xlim(x0 - 3, x1 + 3)
    ax.set_ylim(y0 - 3, y1 + 3)
    ax.set_xlabel("x [cm]")
    ax.set_ylabel("y [cm]")
    ax.set_title("Goal reaching on the robot: five goals in a row, each ring is the 3 cm target", pad=12, fontsize=12.5)
    save(fig, "goal_reaching.png")

    # 6. paths drawn in the path editor (path_ui.py) and driven on the robot
    drawn = drawn_runs()
    if drawn:
        fig, axes = plt.subplots(1, len(drawn), figsize=(5.9 * len(drawn), 4.7), squeeze=False)
        (x0, x1), (y0, y1) = 100 * ARENA
        for ax, (spec, log) in zip(axes[0], drawn):
            q = json.load(open(spec))
            ref = PathRef(q["points"], speed=q.get("speed", 0.045), closed=q.get("closed", False), w_max=q.get("w_max", 1.0))
            rows = [r for r in csv.DictReader(open(log)) if r["sx"]]
            ap = 100 * np.array([[float(r["sx"]), float(r["sy"])] for r in rows if r["task"] == "0"])
            tr = [r for r in rows if r["task"] == "1"]
            p = 100 * np.array([[float(r["sx"]), float(r["sy"])] for r in tr])
            t = np.array([float(r["t"]) for r in tr])
            e = 100 * np.hypot(*(p / 100 - np.array([[float(r["tx"]), float(r["ty"])] for r in tr])).T)
            k = t - t[0] > 3
            ax.add_patch(plt.Rectangle((x0, y0), x1 - x0, y1 - y0, fill=False, ec=AXIS, lw=1.0))
            ax.plot(100 * ref.xy[:, 0], 100 * ref.xy[:, 1], color=AXIS, lw=3.2, solid_capstyle="round", zorder=1)
            if len(ap):
                ax.plot(ap[:, 0], ap[:, 1], color=FT, lw=1.0, alpha=0.55, zorder=2)
            ax.plot(p[:, 0], p[:, 1], color=FT, lw=1.7, zorder=3)
            pts = 100 * np.asarray(q["points"])
            ax.plot(pts[:, 0], pts[:, 1], "o", ms=4, mfc=SURFACE, mec=INK2, mew=1.0, zorder=4)
            ax.text(pts[0, 0], pts[0, 1] - 2.2, "start", color=INK2, fontsize=9, ha="center", va="top")
            ax.set_aspect("equal")
            ax.set_xlim(x0 - 2, x1 + 2)
            ax.set_ylim(y0 - 2, y1 + 2)
            ax.set_xticks([])
            ax.set_yticks([])
            for s in ax.spines.values():
                s.set_visible(False)
            ax.set_title(f"{len(pts)} waypoints, {100 * ref.length:.0f} cm in {ref.T:.0f} s", color=INK, pad=18)
            ax.text(0, 1.02, f"median {np.median(e[k]):.1f} cm  ·  max {e[k].max():.1f} cm", transform=ax.transAxes,
                    color=INK2, fontsize=9.5, va="bottom")
        fig.suptitle("Paths drawn in the editor and driven on the robot: grey is the drawn path with its waypoints,\n"
                     "aqua is the robot (the thin line is its approach to the start), the box is the arena",
                     x=0.125, y=1.1, ha="left", fontsize=12.5, color=INK)
        save(fig, "drawn_paths.png")


def drawn_runs():
    """(path json, deploy log) of every run started from the path editor: the log that began within a minute after
    the path was saved."""
    stamp = lambda name, a, b: time.mktime(time.strptime(os.path.basename(name)[a:b], "%Y%m%d_%H%M%S"))  # noqa: E731
    logs = sorted(glob.glob(os.path.join(RUNS, "deploy_track_run_*.csv")))
    out = []
    for spec in sorted(glob.glob(os.path.join(RUNS, "paths", "path_*.json"))):
        t0 = stamp(spec, 5, -5)
        log = next((g for g in logs if 0 <= stamp(g, 17, -4) - t0 < 60), None)
        if log:
            out.append((spec, log))
    return out


if __name__ == "__main__":
    main()
