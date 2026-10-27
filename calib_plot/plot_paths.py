"""Plot what a calibration run looks like: one grid session and one PRBS session, as a single HTML page.

    python calib_plot/plot_paths.py --grid sessions/bot90_grid_a --prbs sessions/bot90_steps --out paths.html

Panels: the whole grid session in the arena (drive segments vs rest and re-centring), every drive segment rotated
into its start pose, the (L, R) commands over the first 80 s, and one PRBS segment's commands and path. Reads only
tracker.csv, segments.csv and session.json from each session folder (calibration/README.md, output files).

example_sim.html was made from simulated sessions (no hardware):

    python -m graspion_calib schedule grid --blocks 1 --seed 7 --out grid1.json
    python -m graspion_calib schedule prbs --seed 7 --out prbs.json
    python -m graspion_calib run --sim --bot 90 --seed 1 --schedule grid1.json --out sim_grid
    python -m graspion_calib run --sim --bot 90 --seed 1 --schedule prbs.json --out sim_prbs
    python calib_plot/plot_paths.py --grid sim_grid --prbs sim_prbs --out calib_plot/example_sim.html
"""
import argparse
import csv
import json
import math
import os


def mm(v):
    return round(v * 1000)


def load(sess):
    rows = csv.DictReader(open(os.path.join(sess, "tracker.csv")))
    tr = [(float(r["t"]), float(r["x"]), float(r["y"]), float(r["theta"])) for r in rows if r["x"] not in ("", "nan")]
    segs = [r for r in csv.DictReader(open(os.path.join(sess, "segments.csv"))) if r["status"] == "ok"]
    info = json.load(open(os.path.join(sess, "session.json")))
    return tr, segs, info


def describe(info):
    meta = info.get("schedule_meta") or {}
    src = "simulated" if info.get("simulated") else "recorded"
    return f"{meta.get('kind', '?')} schedule, seed {meta.get('seed', '?')}, bot {info.get('bot_id', '?')}, {src}"


def grid_data(sess):
    tr, segs, info = load(sess)
    win = [(float(s["t_seg_ack"]), float(s["t_end_ack"]), i) for i, s in enumerate(segs)]
    # The whole session at ~3 Hz, split into runs of one drive segment (tag = index) or rest/re-centring (-1).
    runs, k, last = [], 0, -1e9
    for t, x, y, _ in tr:
        while k < len(win) and win[k][1] < t:
            k += 1
        tag = win[k][2] if k < len(win) and win[k][0] <= t <= win[k][1] else -1
        if runs and runs[-1][0] != tag:
            runs.append([tag, runs[-1][1][-2:]])
        elif t - last < 0.35:
            continue
        if not runs:
            runs.append([tag, []])
        runs[-1][1] += [mm(x), mm(y)]
        last = t
    # Each drive segment in its start frame: start at the origin, heading +y.
    fan = []
    for s in segs:
        pts = [p for p in tr if float(s["t_seg_ack"]) <= p[0] <= float(s["t_end_ack"])]
        if len(pts) < 3:
            continue
        _, x0, y0, th0 = pts[0]
        c, sn = math.cos(math.pi / 2 - th0), math.sin(math.pi / 2 - th0)
        rel = []
        for p in pts[::6] + [pts[-1]]:
            rel += [mm((p[1] - x0) * c - (p[2] - y0) * sn), mm((p[1] - x0) * sn + (p[2] - y0) * c)]
        fan.append([int(s["L"]), int(s["R"]), rel])
    a = info["arena"]
    return {
        "path": runs, "fan": fan,
        "segs": [[int(s["L"]), int(s["R"]), round(float(s["t_seg_ack"]), 2), round(float(s["t_end_ack"]), 2)] for s in segs],
        "arena": [mm(a["xmin"]), mm(a["ymin"]), mm(a["xmax"]), mm(a["ymax"]), mm(a.get("margin") or 0)],
    }, info, (tr[-1][0] - tr[0][0]) / 60 if tr else 0


def prbs_data(sess, seg):
    tr, segs, info = load(sess)
    s = segs[seg]
    t0, t1 = float(s["t_seg_ack"]), float(s["t_end_ack"])
    p = []
    for q in [q for q in tr if t0 <= q[0] <= t1][::3]:
        p += [mm(q[1]), mm(q[2])]
    return {"levels": [[round(a, 2), l, r] for a, l, r in json.loads(s["levels"])], "dur": float(s["duration_s"]), "p": p}, info


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--grid", required=True, help="grid session folder")
    ap.add_argument("--prbs", required=True, help="PRBS session folder")
    ap.add_argument("--prbs-seg", type=int, default=1, help="which ok PRBS segment to show (default 1)")
    ap.add_argument("--out", required=True)
    a = ap.parse_args()
    d, ginfo, minutes = grid_data(a.grid)
    d["prbs"], pinfo = prbs_data(a.prbs, a.prbs_seg)
    sub = (f"Grid session: {len(d['segs'])} segments in {minutes:.0f} min ({describe(ginfo)}). "
           f"PRBS session: {describe(pinfo)}.")
    if ginfo.get("simulated") or pinfo.get("simulated"):
        sub += " Simulated data shows the shape of the protocol, not how a real bot moves."
    html = open(os.path.join(os.path.dirname(os.path.abspath(__file__)), "template.html"), encoding="utf-8").read()
    html = html.replace("__SUB__", sub).replace("__DATA__", json.dumps(d, separators=(",", ":")))
    with open(a.out, "w", encoding="utf-8") as f:
        f.write(html)
    print("wrote", a.out)


if __name__ == "__main__":
    main()
