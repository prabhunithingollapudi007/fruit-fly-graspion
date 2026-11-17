"""Loop delay and motor lag from the calibration segments: every segment is a step from rest to (L, R) and back to
stop, tracked at ~49 fps, with the SEG request and the STOP ack on the same PC clock as the tracker.

Start: twist(t) = (u1, u2, w) * k(t), k = 0 before delay, then 1 - exp(-(t - delay) / tau)       (t from the SEG request)
Stop:  twist(t) = (u1, u2, w) * k(t), k = 1 before delay, then exp(-(t - delay) / tau)           (t from the STOP ack)
delay and tau are shared per transition kind; (u1, u2, w) is free per segment, so the map's own error stays out.

Limits: only rest -> drive and drive -> stop are measured, not a change from one drive to another. The delay is to
the frame's arrival at the PC under the calibration's IR load (a line on a change or every 0.5 s). Delay and tau
trade off where the cost is flat; the "flat" range printed is where the cost is within 2 % of its minimum, a
description of that trade-off and not a confidence interval (the residuals are cumulative and correlated).
python rl_walk/step_fit.py SESSION_DIR [SESSION_DIR ...]
"""
import csv
import sys

import numpy as np

SP, SH = 0.002, 0.03  # weights: position [m], heading [rad]
WINDOWS = {"start": (-0.3, 2.0), "stop": (-1.0, 0.7)}  # seconds around the transition


def load(session, kind):
    tr = np.genfromtxt(f"{session}/tracker.csv", delimiter=",", names=True)
    segs = [r for r in csv.DictReader(open(f"{session}/segments.csv"))
            if r["status"] == "ok" and r["kind"] == "drive" and float(r["moved_m"]) > 0.02]
    a, b = WINDOWS[kind]
    out = []
    for s in segs:
        t0 = float(s["t_seg_req"] if kind == "start" else s["t_end_ack"])
        m = (tr["t"] >= t0 + a) & (tr["t"] <= t0 + b)
        t, x, y, th = tr["t"][m] - t0, tr["x"][m], tr["y"][m], np.unwrap(tr["theta"][m])
        if len(t) < 0.6 * 49 * (b - a):
            continue
        ref = t < a + 0.1 if kind == "start" else np.arange(len(t)) < 1
        x0, y0, h0 = x[ref].mean(), y[ref].mean(), th[ref].mean()
        c, sn = np.cos(h0), np.sin(h0)
        bx, by = c * (x - x0) + sn * (y - y0), -sn * (x - x0) + c * (y - y0)
        out.append(dict(t=t, bx=bx, by=by, dh=th - h0, kind=kind))
    return out


def shape(t, delay, tau, kind):
    tt = np.clip(t - delay, 0, None)
    e = np.exp(-tt / tau) if tau > 0 else (tt == 0).astype(float)
    k = 1 - e if kind == "start" else np.where(t < delay, 1.0, e)
    dt = np.diff(t, prepend=t[0])
    return k, np.cumsum(k * dt)


def seg_cost(sg, delay, tau):
    t = sg["t"]
    k, K = shape(t, delay, tau, sg["kind"])
    one, zero = np.ones_like(t), np.zeros_like(t)
    (W, h0), *_ = np.linalg.lstsq(np.column_stack([K, one]), sg["dh"], rcond=None)
    h = W * K + h0
    dt = np.diff(t, prepend=t[0])
    # the reference pose is one noisy sample (stop) or a short mean (start): fit a constant offset as well
    A = np.column_stack([np.cumsum(k * np.cos(h) * dt), np.cumsum(-k * np.sin(h) * dt), one, zero])
    B = np.column_stack([np.cumsum(k * np.sin(h) * dt), np.cumsum(k * np.cos(h) * dt), zero, one])
    M = np.vstack([A, B])
    rhs = np.concatenate([sg["bx"], sg["by"]])
    u, *_ = np.linalg.lstsq(M, rhs, rcond=None)
    return np.sum(((M @ u - rhs) / SP) ** 2) + np.sum(((h - sg["dh"]) / SH) ** 2)


DELAYS = np.arange(0.0, 0.61, 0.02)
TAUS = np.array([0.0, 0.02, 0.04, 0.06, 0.08, 0.1, 0.15, 0.2, 0.3, 0.4, 0.6])


def grid(segs):
    C = np.array([[sum(seg_cost(s, d, tau) for s in segs) for tau in TAUS] for d in DELAYS])
    i, j = np.unravel_index(np.argmin(C), C.shape)
    return C, DELAYS[i], TAUS[j]


def summary(label, segs):
    C, d, tau = grid(segs)
    ok = C / C.min() <= 1.02
    per = [DELAYS[np.argmin([seg_cost(s, dd, tau) for dd in DELAYS])] for s in segs]
    q = np.percentile(per, [10, 50, 90])
    print(f"  {label:10s} n={len(segs):3d}: delay {d:.2f} s, tau {tau:.2f} s | flat (cost within 2 %): delay "
          f"{DELAYS[ok.any(1)].min():.2f}-{DELAYS[ok.any(1)].max():.2f}, tau {TAUS[ok.any(0)].min():.2f}-"
          f"{TAUS[ok.any(0)].max():.2f} | per-segment delay p10/50/90 {q[0]:.2f}/{q[1]:.2f}/{q[2]:.2f}")


def main():
    for kind in ("start", "stop"):
        print(kind)
        pooled = []
        for sess in sys.argv[1:]:
            segs = load(sess, kind)
            pooled += segs
            summary(sess.replace("\\", "/").rstrip("/").split("/")[-1], segs)
        if len(sys.argv) > 2:
            summary("pooled", pooled)


if __name__ == "__main__":
    main()
