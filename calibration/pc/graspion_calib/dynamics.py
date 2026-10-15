"""First-order lag + dead time of the twist response, from step and PRBS segments (FEASIBILITY.md 5.2 step 5).

Model per output y in {u1, omega}::

    y_ss(t) = g_seg * F(level(t - d))          F = fitted forward map, g_seg = per-segment gain (absorbs map error)
    tau * dy/dt = y_ss - y                     y = 0 before the segment starts

Times are on the camera clock: a command takes effect at t_ack + cam_offset, where cam_offset is the median
delay from the blaster's acknowledgement to the sync-flash onset in the video (IR decode + camera latency).
So d is the *mechanical* dead time after the bot decodes a frame; the full loop delay adds the bridge/air time
and the camera latency (see report.py). The measured velocity is a Savitzky-Golay derivative, so the model
prediction is passed through the same smoothing filter before comparison.
"""
from __future__ import annotations

import numpy as np
from scipy.optimize import minimize
from scipy.signal import savgol_filter

from .twist import resample_uniform, uniform_chunks


def step_response(ts, t_steps, dy_steps, d, tau):
    """Sum of first-order step responses: y(t) = sum_j dy_j (1 - exp(-(t - t_j - d)/tau)) for t > t_j + d."""
    s = ts[:, None] - (np.asarray(t_steps)[None, :] + d)
    resp = np.where(s > 0, 1.0 - np.exp(-np.clip(s, 0, None) / max(tau, 1e-4)), 0.0)
    return resp @ np.asarray(dy_steps, float)


class DynSegment:
    """One segment prepared for the lag fit: uniform velocity runs plus its commanded level changes."""

    def __init__(self, t, x, y, th, levels, fwd_map, window_s=0.15, polyorder=2):
        # levels: list of (t_effect, L, R) on the camera clock; must include the final stop (0, 0)
        self.levels = sorted(levels)
        self.runs = []
        t = np.asarray(t, float)
        th = np.unwrap(np.asarray(th, float))
        dt, chunks = uniform_chunks(t)
        self.win = None
        for idx in chunks:
            tu, xu, yu, hu = resample_uniform(t[idx], x[idx], y[idx], th[idx], dt=dt)
            win = max(polyorder + 2, int(round(window_s / dt)) | 1)
            if len(tu) < win + 2:
                continue
            vx = savgol_filter(xu, win, polyorder, deriv=1, delta=dt)
            vy = savgol_filter(yu, win, polyorder, deriv=1, delta=dt)
            w = savgol_filter(hu, win, polyorder, deriv=1, delta=dt)
            hs = savgol_filter(hu, win, polyorder)
            u1 = np.cos(hs) * vx + np.sin(hs) * vy
            self.runs.append((tu, {"u1": u1, "omega": w}))
            self.win = win
        self.polyorder = polyorder
        Ls = np.array([lv[1] for lv in self.levels], float)
        Rs = np.array([lv[2] for lv in self.levels], float)
        ss = fwd_map.predict(Ls, Rs)
        self.t_steps = np.array([lv[0] for lv in self.levels], float)
        self.ss = {"u1": ss[:, 0], "omega": ss[:, 2]}

    def predict(self, out, d, tau):
        """Model velocity passed through the measurement's own filter: Savitzky-Golay derivative of its integral
        (exact for omega, and for u1 while the heading turns little within one filter window)."""
        dy = np.diff(np.r_[0.0, self.ss[out]])
        preds = []
        for tu, _ in self.runs:
            p = step_response(tu, self.t_steps, dy, d, tau)
            if len(p) > self.win:
                dt = tu[1] - tu[0]
                integ = np.r_[0.0, np.cumsum(0.5 * (p[1:] + p[:-1]) * dt)]
                p = savgol_filter(integ, self.win, self.polyorder, deriv=1, delta=dt)
            preds.append(p)
        return preds

    def sse(self, out, d, tau, fit_gain=True):
        preds = self.predict(out, d, tau)
        m = np.concatenate([r[1][out] for r in self.runs]) if self.runs else np.array([])
        p = np.concatenate(preds) if preds else np.array([])
        if len(m) == 0:
            return 0.0, 0, 1.0
        g = float(p @ m / (p @ p)) if (fit_gain and p @ p > 1e-12) else 1.0
        r = m - g * p
        return float(r @ r), len(m), g


def fit_lag(segments, outputs=("u1", "omega"), d_range=(0.0, 0.4), tau_range=(0.01, 0.6), fit_gain=True):
    """Pooled fit of (dead time d, lag tau) per output, and jointly. segments: list of DynSegment."""
    results = {}
    d_grid = np.arange(d_range[0], d_range[1] + 1e-9, 0.02)
    tau_grid = np.geomspace(tau_range[0], tau_range[1], 24)

    def total(out_list, z):
        d, tau = z
        if not (d_range[0] <= d <= d_range[1] and tau_range[0] <= tau <= tau_range[1]):
            return 1e30
        s = 0.0
        for o in out_list:
            norm = np.mean([np.var(np.concatenate([r[1][o] for r in sg.runs])) for sg in segments if sg.runs]) + 1e-12
            s += sum(sg.sse(o, d, tau, fit_gain)[0] for sg in segments) / norm
        return s

    for name, outs in [(o, (o,)) for o in outputs] + [("joint", tuple(outputs))]:
        coarse = min(((total(outs, (d, tau)), d, tau) for d in d_grid for tau in tau_grid))
        res = minimize(lambda z: total(outs, z), np.array(coarse[1:]), method="Nelder-Mead",
                       options={"xatol": 1e-4, "fatol": 1e-10, "maxiter": 400})
        d, tau = (float(v) for v in res.x)
        rmse, gains, n_tot = {}, {}, 0
        for o in outs:                                  # per output: u1 in m/s, omega in rad/s
            rows = [sg.sse(o, d, tau, fit_gain) for sg in segments]
            n = sum(r[1] for r in rows)
            rmse[o] = float(np.sqrt(sum(r[0] for r in rows) / max(n, 1)))
            gains[o] = [r[2] for r in rows]
            n_tot += n
        results[name] = {"dead_time": d, "tau": tau, "rmse": rmse, "n": int(n_tot), "gains": gains}
    return results
