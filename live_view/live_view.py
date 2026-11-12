"""Live view of a calibration run: for each segment, the track the map expects next to the track the camera sees.

    python live_view/live_view.py calibration/pc/sessions/<session>  [--map analysis/<run>/forward_map.json]

Start it in a second terminal next to `python -m graspion_calib run ...`; it follows the session folder as the run
writes it (tracker.csv, frames.csv, segments.csv) and needs neither the camera nor the blaster. Before a segment
(the operator pause) it draws where the next (L, R) should take the robot from where it stands now, so the robot
can be placed with room for it; during and after the segment, the expected track from the segment's start pose
and the measured one. The expected track integrates the body twist of --map (a forward_map.json from `analyze`);
without one it uses PRIOR, a rough guess from the teammate's closed-loop trial (dead zone 55, about 6.6 cm/s at
full drive, veering right), not a calibration. q or Esc closes the window.
"""
import argparse
import csv
import json
import math
import os
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "calibration", "pc"))
from graspion_calib.forward_map import PowerLawMap  # noqa: E402
from graspion_calib.grid_map import load_map  # noqa: E402
from graspion_calib.geometry import BotGeometry  # noqa: E402

A, B, C = 4.6e-4, 0.012, 0.0031          # u1 per unit drive past the dead zone; turn per motor; right veer
PRIOR = PowerLawMap(p0=(55.0, 55.0), gamma=(1.0, 1.0),
                    coef=np.array([[0.0, A, A], [0.0, 0.0, 0.0], [0.0, -B - C, B - C]]), features="lin")
PX_PER_M = 1000
COL = {"bg": (32, 32, 32), "grid": (55, 55, 55), "box": (150, 150, 150), "fence": (60, 60, 200),   # BGR
       "ref": (230, 230, 230), "exp": (40, 150, 255), "act": (80, 220, 80), "old": (90, 120, 90),
       "old_exp": (50, 100, 150), "bot": (255, 220, 120), "text": (235, 235, 235)}


class Tail:
    """New complete rows of a CSV that another process is still appending to."""

    def __init__(self, path):
        self.path, self.f, self.cols, self.buf = path, None, None, ""

    def rows(self):
        if self.f is None:
            if not os.path.exists(self.path):
                return []
            self.f = open(self.path, newline="")
        self.buf += self.f.read()
        *lines, self.buf = self.buf.split("\n")
        out = []
        for vals in csv.reader(ln.rstrip("\r") for ln in lines):   # segments.csv quotes JSON with commas
            if self.cols is None:
                self.cols = vals
            elif len(vals) == len(self.cols):
                out.append(dict(zip(self.cols, vals)))
        return out


def num(v):
    try:
        return float(v)
    except ValueError:
        return float("nan")


def predict_path(m, L, R, pose, duration, dt=0.02):
    """Integrate the map's body twist (u1, u2, omega) for `duration` from pose (x, y, theta)."""
    u1, u2, w = m.predict(L, R)[0]
    x, y, th = pose
    pts = [(x, y)]
    for _ in range(int(duration / dt)):
        x += (u1 * math.cos(th) - u2 * math.sin(th)) * dt
        y += (u1 * math.sin(th) + u2 * math.cos(th)) * dt
        th += w * dt
        pts.append((x, y))
    return np.array(pts), (u1, w)


def measured_twist(t, x, y, th, skip=0.5):
    """Mean forward speed and turn rate over a segment's samples after its first `skip` seconds."""
    k = t >= t[0] + skip
    if k.sum() < 5:
        return None
    t, x, y, th = t[k], x[k], y[k], np.unwrap(th[k])
    w = np.polyfit(t - t[0], th, 1)[0]
    dx, dy, mid = np.diff(x), np.diff(y), 0.5 * (th[1:] + th[:-1])
    u1 = np.sum(dx * np.cos(mid) + dy * np.sin(mid)) / (t[-1] - t[0])
    return u1, w


class View:
    def __init__(self, meta, refs, arena):
        pts = [(v[0], v[1]) for v in refs.values()] or [(arena["xmin"], arena["ymin"]), (arena["xmax"], arena["ymax"])]
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        self.x0, self.y1 = min(xs) - 0.06, max(ys) + 0.06
        self.w = int((max(xs) - min(xs) + 0.12) * PX_PER_M)
        self.h = int((max(ys) - min(ys) + 0.12) * PX_PER_M) + 90
        self.meta, self.refs, self.arena = meta, refs, arena

    def px(self, p):
        p = np.atleast_2d(p)
        return np.c_[(p[:, 0] - self.x0) * PX_PER_M, 90 + (self.y1 - p[:, 1]) * PX_PER_M].astype(np.int32)

    def base(self):
        img = np.full((self.h, self.w, 3), COL["bg"], np.uint8)
        for gx in np.arange(math.floor(self.x0 * 10) / 10, self.x0 + self.w / PX_PER_M, 0.1):
            cv2.line(img, tuple(self.px((gx, self.y1))[0]), tuple(self.px((gx, self.y1 - self.h / PX_PER_M))[0]), COL["grid"], 1)
        for gy in np.arange(math.floor((self.y1 - self.h / PX_PER_M) * 10) / 10, self.y1, 0.1):
            cv2.line(img, tuple(self.px((self.x0, gy))[0]), tuple(self.px((self.x0 + self.w / PX_PER_M, gy))[0]), COL["grid"], 1)
        a, m = self.arena, self.arena.get("margin", 0.0)
        cv2.rectangle(img, tuple(self.px((a["xmin"], a["ymax"]))[0]), tuple(self.px((a["xmax"], a["ymin"]))[0]), COL["box"], 1)
        cv2.rectangle(img, tuple(self.px((a["xmin"] + m, a["ymax"] - m))[0]),
                      tuple(self.px((a["xmax"] - m, a["ymin"] + m))[0]), COL["fence"], 1)
        for i, (rx, ry) in self.refs.items():
            c = self.px((rx, ry))[0]
            s = int(0.02 * PX_PER_M)
            cv2.rectangle(img, (c[0] - s, c[1] - s), (c[0] + s, c[1] + s), COL["ref"], 1)
            cv2.putText(img, str(i), (c[0] - 5, c[1] + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, COL["ref"], 1)
        return img

    def path(self, img, pts, col, end_dot=False, thick=2):
        if len(pts) < 2:
            return
        p = self.px(pts)
        cv2.polylines(img, [p], False, col, thick, cv2.LINE_AA)
        if end_dot:                                               # where the expected track ends
            cv2.circle(img, tuple(p[-1]), 5, col, -1, cv2.LINE_AA)

    def bot(self, img, pose, col, filled=False):
        x, y, th = pose
        c = self.px((x, y))[0]
        ax = (int(0.03 * PX_PER_M), int(0.015 * PX_PER_M))       # the 60 x 30 mm body
        cv2.ellipse(img, tuple(c), ax, -math.degrees(th), 0, 360, col, -1 if filled else 2, cv2.LINE_AA)
        tip = self.px((x + 0.045 * math.cos(th), y + 0.045 * math.sin(th)))[0]
        cv2.arrowedLine(img, tuple(c), tuple(tip), col, 2, cv2.LINE_AA, tipLength=0.35)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session")
    ap.add_argument("--map", help="forward_map.json from analyze or grid_map.json from gridmap (default: the rough PRIOR)")
    ap.add_argument("--keep", type=int, default=6, help="finished segments drawn in grey")
    ap.add_argument("--once", metavar="PNG", help="draw the session as it is now into this image and exit")
    a = ap.parse_args()
    m = load_map(a.map) if a.map else PRIOR
    d = a.session
    while not (os.path.exists(os.path.join(d, "session.json")) and os.path.exists(os.path.join(d, "schedule.json"))):
        print(f"waiting for {d} ...")
        time.sleep(1.0)
    meta = json.load(open(os.path.join(d, "session.json")))
    sched = json.load(open(os.path.join(d, "schedule.json")))["segments"]
    rig = meta.get("rig", {})
    refs = {int(k): v for k, v in rig.get("reference_markers", {}).items()}
    g = meta["geometry"]
    geom = BotGeometry(int(g["marker_id"]), tuple(g["marker_offset"]), float(g["marker_yaw"]))
    view = View(meta, refs, meta["arena"])
    trk, frm, sgs = (Tail(os.path.join(d, f)) for f in ("tracker.csv", "frames.csv", "segments.csv"))
    T, X, Y, TH = [], [], [], []
    seg_start, seg_end, seg_lr, status = {}, {}, {}, {}
    cur, purpose = -1, ""
    auto = bool(meta.get("recentre", {}).get("enabled"))         # run --auto: the robot re-centres itself
    win = f"live view: {os.path.basename(os.path.normpath(d))}"
    if not a.once:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    while True:
        for r in trk.rows():
            if int(num(r["id"])) == geom.marker_id:
                bx, by, bth = geom.body_from_marker(num(r["x"]), num(r["y"]), num(r["theta"]))
                T.append(num(r["t"])); X.append(float(bx)); Y.append(float(by)); TH.append(float(bth))
        for r in frm.rows():
            s = int(num(r["seg"]))
            purpose = r["purpose"]
            if s >= 0:
                cur = s
            if r["op"] == "SEG":
                seg_start[s], seg_lr[s] = num(r["t_ack"]), (int(num(r["L"])), int(num(r["R"])))
                seg_end.pop(s, None)
            elif r["op"] == "STOP" and s in seg_start and s not in seg_end and num(r["t_ack"]) > seg_start[s]:
                seg_end[s] = num(r["t_ack"])
        for r in sgs.rows():
            status[int(num(r["seg"]))] = r["status"]
        t, x, y, th = map(np.asarray, (T, X, Y, TH))
        img = view.base()

        def track(s):
            k = (t >= seg_start[s]) & (t <= seg_end.get(s, np.inf))
            return np.c_[x[k], y[k]], (t[k], x[k], y[k], th[k])

        def start_pose(s):
            k = np.searchsorted(t, seg_start[s]) - 1
            return (x[k], y[k], th[k]) if k >= 0 else None

        for s in sorted(seg_end)[-a.keep:]:                       # finished segments, faded
            if s == cur:
                continue
            p0 = start_pose(s)
            if p0 is not None:
                view.path(img, predict_path(m, *seg_lr[s], p0, sched[s]["duration_s"])[0], COL["old_exp"], True, 1)
            view.path(img, track(s)[0], COL["old"], False, 1)
        lines = []
        if 0 <= cur < len(sched):
            spec = sched[cur]
            L, R = spec["levels"][0][1], spec["levels"][0][2]
            if cur in seg_start and len(t):                       # running or just finished
                p0 = start_pose(cur)
                if p0 is not None:
                    exp, (eu, ew) = predict_path(m, L, R, p0, spec["duration_s"])
                    view.path(img, exp, COL["exp"], True)
                    view.bot(img, p0, COL["exp"])
                pts, samples = track(cur)
                view.path(img, pts, COL["act"])
                done = cur in seg_end
                el = (seg_end[cur] if done else t[-1]) - seg_start[cur]
                mt = measured_twist(*samples) if len(samples[0]) else None
                lines.append(f"segment {cur}/{len(sched) - 1}  (L, R) = ({L}, {R})  "
                             + (f"done: {status.get(cur, '...')}" if done else f"running {el:.1f}/{spec['duration_s']:g} s"))
                if p0 is not None:
                    lines.append(f"expected u1 {100 * eu:5.1f} cm/s  w {ew:+.2f} rad/s     measured "
                                 + (f"u1 {100 * mt[0]:5.1f} cm/s  w {mt[1]:+.2f} rad/s" if mt else "(first 0.5 s skipped)"))
            elif len(t):                                          # the operator pause: the plan from here
                exp, (eu, ew) = predict_path(m, L, R, (x[-1], y[-1], th[-1]), spec["duration_s"])
                view.path(img, exp, COL["exp"], True)
                ok = all(view.arena["xmin"] + view.arena["margin"] <= px <= view.arena["xmax"] - view.arena["margin"] and
                         view.arena["ymin"] + view.arena["margin"] <= py <= view.arena["ymax"] - view.arena["margin"]
                         for px, py in exp)
                lines.append(f"next: segment {cur}/{len(sched) - 1}  (L, R) = ({L}, {R}), {spec['duration_s']:g} s  "
                             f"expected u1 {100 * eu:.1f} cm/s, w {ew:+.2f} rad/s")
                if purpose.startswith("recentre") and purpose != "recentre_stop":
                    lines.append(f"re-centring the robot over the camera ({purpose[9:]}) before segment {cur}")
                elif auto:
                    lines.append("re-centred: the segment starts after a short rest")
                else:
                    lines.append("expected track stays inside the fence: Enter in the run window" if ok else
                                 "expected track leaves the fence: move the robot first")
        if len(t):
            view.bot(img, (x[-1], y[-1], th[-1]), COL["bot"])
        else:
            lines.append("no robot pose yet (tag 10 not seen)")
        n_ok = sum(1 for v in status.values() if v == "ok")
        lines.append(f"{len(status)} segments logged, {n_ok} ok   "
                     f"map: {'--map ' + os.path.basename(a.map) if a.map else 'PRIOR (rough guess)'}   "
                     "orange = expected (dot: end), green = measured")
        for k, ln in enumerate(lines[:3]):
            cv2.putText(img, ln, (10, 24 + 26 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.6, COL["text"], 1, cv2.LINE_AA)
        if a.once:
            cv2.imwrite(a.once, img)
            return
        cv2.imshow(win, img)
        key = cv2.waitKey(100) & 0xFF
        if key in (ord("q"), 27) or cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
            break
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
