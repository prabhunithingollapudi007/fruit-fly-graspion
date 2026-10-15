"""First-order lag + dead time of the twist response, from step and PRBS segments (FEASIBILITY.md 5.2 step 5).

Model per output y in {u1, omega}::

    y_ss(t) = g_seg * F(level(t - d))          F = fitted forward map, g_seg = per-segment gain (absorbs map error)
    tau * dy/dt = y_ss - y                     y = 0 before the segment starts

Times are on the camera clock: a command takes effect at t_ack + cam_offset, where cam_offset is the median
delay from the blaster's acknowledgement to the sync-flash onset in the video (IR decode + camera latency).
So d is the *mechanical* dead time after the bot decodes a frame; the full loop delay adds the bridge/air time
and the camera latency (see report.py). The measured velocity is a Savitzky-Golay derivative, so the model
prediction is passed through the same smoothing filter before comparison.

A level change or STOP that the bot missed (IR loss) takes effect one blaster period later, at the next frame
that repeats it. Fitted at the first frame, such a change reads as extra lag: on the simulated rig (20 seeds,
truth 60 ms) the joint tau grew from 58 +- 2 ms at 0 % frame loss to 71 +- 12 ms at 10 %. So fit_lag re-times
each change to the repeat that fits best (DynSegment.reassign) and refits: 59 +- 4 ms at 10 %.
"""
from __future__ import annotations

import numpy as np
from scipy.optimize import minimize
from scipy.signal import savgol_coeffs, savgol_filter

from .twist import clean_heading, resample_uniform, uniform_chunks


def step_response(ts, t_steps, dy_steps, d, tau):
    """Sum of first-order step responses: y(t) = sum_j dy_j (1 - exp(-(t - t_j - d)/tau)) for t > t_j + d."""
    s = ts[:, None] - (np.asarray(t_steps)[None, :] + d)
    resp = np.where(s > 0, 1.0 - np.exp(-np.clip(s, 0, None) / max(tau, 1e-4)), 0.0)
    return resp @ np.asarray(dy_steps, float)


class DynSegment:
    """One segment prepared for the lag fit: uniform velocity runs plus its commanded level changes."""

    def __init__(self, t, x, y, th, levels, fwd_map, window_s=0.15, polyorder=2, v_bat=None):
        # levels: list of (t_effect, L, R) or (t_effect, L, R, later) on the camera clock, including the final stop
        # (0, 0); later = effect times of the frames repeating that command (extract._segment_levels)
        # v_bat: battery voltage [V] at which a 'bat' map is evaluated (default: the map's v_bat_median / v_ref);
        # the per-segment gain absorbs the rest of the battery effect
        self.levels = sorted(levels)
        self.runs = []
        t, th, x, y = clean_heading(t, th, x, y)          # a pi flip would otherwise be a 2 pi / dt omega spike
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
        if self.win is not None:
            # savgol_filter(mode='interp') as precomputed pieces: interior convolution coefficients and the two
            # edge-fit blocks (identical output; savgol_filter would redo the edge polynomial fits per call)
            E = savgol_filter(np.eye(self.win), self.win, polyorder, deriv=1, delta=dt, axis=0)
            half = self.win // 2
            self._sg = (savgol_coeffs(self.win, polyorder, deriv=1, delta=dt), E[:half], E[self.win - half:])
        Ls = np.array([lv[1] for lv in self.levels], float)
        Rs = np.array([lv[2] for lv in self.levels], float)
        Vs = None
        if getattr(fwd_map, "features", None) == "bat":
            if v_bat is None:
                v_bat = fwd_map.meta.get("v_bat_median", fwd_map.v_ref)
            Vs = np.full(len(Ls), float(v_bat))
        ss = fwd_map.predict(Ls, Rs, Vs)
        self.t_steps = np.array([lv[0] for lv in self.levels], float)    # effect times in use (see reassign)
        self.t_first = self.t_steps.copy()
        self.t_later = [tuple(lv[3]) if len(lv) > 3 else () for lv in self.levels]
        self.ss = {"u1": ss[:, 0], "omega": ss[:, 2]}

    def reassign(self, d, tau, weights, fit_gain=True, min_gain=10.0):
        """Re-time the changes after the SEG for lost frames: a change the bot missed takes effect at a later frame
        that repeats it. A level whose frames were all lost (a short PRBS dwell) never does, i.e. it ends at the
        next change; a STOP whose repeats were all lost leaves the bot driving until the next command, after the
        window. For each change, in time order, take the candidate (first frame, a repeat, never) that
        fits best at (d, tau), weights: {output: 1 / residual variance}, but leave the first frame only if that
        lowers the weighted SSE by more than min_gain per filter window (the residuals are correlated over about
        one window). Without the margin, changes that hardly move u1, e.g. (90, 90) -> (90, 0), were moved
        without any frame loss: gains of up to 4.3 per window on the simulated rig, against mostly 18 to 600 for
        the changes moved at 10 % frame loss. Returns the number of times changed."""
        n = 0
        if not self.runs:
            return n
        for j in range(1, len(self.t_steps)):
            cands = [self.t_first[j], *self.t_later[j], self.t_steps[j + 1] if j + 1 < len(self.t_steps) else np.inf]
            old, costs = self.t_steps[j], []
            for c in cands:
                self.t_steps[j] = c
                costs.append(sum(w * self.sse(o, d, tau, fit_gain)[0] for o, w in weights.items()))
            k = int(np.argmin(costs))
            self.t_steps[j] = cands[k] if costs[0] - costs[k] > min_gain * self.win else cands[0]
            n += int(self.t_steps[j] != old)
        return n

    def predict(self, out, d, tau):
        """Model velocity passed through the measurement's own filter: Savitzky-Golay derivative of its integral
        (exact for omega, and for u1 while the heading turns little within one filter window)."""
        dy = np.diff(np.r_[0.0, self.ss[out]])
        preds = []
        for tu, _ in self.runs:
            p = step_response(tu, self.t_steps, dy, d, tau)
            dt = tu[1] - tu[0]
            integ = np.r_[0.0, np.cumsum(0.5 * (p[1:] + p[:-1]) * dt)]
            coef, e_lo, e_hi = self._sg
            p = np.convolve(integ, coef, mode="same")
            p[:len(e_lo)] = e_lo @ integ[:self.win]
            p[len(p) - len(e_hi):] = e_hi @ integ[len(integ) - self.win:]
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


def fit_lag(segments, outputs=("u1", "omega"), d_range=(0.0, 0.4), tau_range=(0.01, 0.6), fit_gain=True,
            max_reassign=3):
    """Pooled fit of (dead time d, lag tau) per output, and jointly. segments: list of DynSegment.

    The joint fit is a profile likelihood with its own unknown noise variance per output: it minimises
    sum_o n_o / 2 * log(SSE_o), which weights each output by its residual variance. Normalising by the signal
    variance instead let omega dominate, although its Savitzky-Golay derivative carries the heading diffusion
    and is far noisier than u1: on the simulated rig the joint (d, tau) then scattered more across seeds than
    the u1 fit alone. For a single output this is the plain least-squares fit.

    Lost frames: after the joint fit, every change is re-timed to its best candidate (DynSegment.reassign) and the
    joint fit is repeated, up to max_reassign times or until no time changes. The per-output fits use the final
    times, which stay on the segments (t_steps). 'late_changes' counts the changes that took effect after their
    first frame (max_reassign=0: none re-timed)."""
    results = {}
    d_grid = np.arange(d_range[0], d_range[1] + 1e-9, 0.02)
    tau_grid = np.geomspace(tau_range[0], tau_range[1], 24)

    def sse_n(o, d, tau):
        rows = [sg.sse(o, d, tau, fit_gain) for sg in segments]
        return sum(r[0] for r in rows), sum(r[1] for r in rows)

    def total(out_list, z):
        d, tau = z
        if not (d_range[0] <= d <= d_range[1] and tau_range[0] <= tau <= tau_range[1]):
            return 1e30
        s = 0.0
        for o in out_list:
            sse, n = sse_n(o, d, tau)
            if n:
                s += 0.5 * n * np.log(max(sse, 1e-300))
        return s

    def fit(outs):
        coarse = min(((total(outs, (d, tau)), d, tau) for d in d_grid for tau in tau_grid))
        res = minimize(lambda z: total(outs, z), np.array(coarse[1:]), method="Nelder-Mead",
                       options={"xatol": 1e-4, "fatol": 1e-10, "maxiter": 400})
        return tuple(float(v) for v in res.x)

    joint = fit(tuple(outputs))
    for _ in range(max_reassign):
        w = {}
        for o in outputs:
            sse, n = sse_n(o, *joint)
            if n and sse > 0:
                w[o] = n / sse
        if not w or not sum(sg.reassign(*joint, w, fit_gain) for sg in segments):
            break
        joint = fit(tuple(outputs))
    n_late = int(sum(np.sum(sg.t_steps != sg.t_first) for sg in segments))

    for name, outs in [(o, (o,)) for o in outputs] + [("joint", tuple(outputs))]:
        d, tau = joint if name == "joint" else fit(outs)
        rmse, gains, n_tot = {}, {}, 0
        for o in outs:                                  # per output: u1 in m/s, omega in rad/s
            rows = [sg.sse(o, d, tau, fit_gain) for sg in segments]
            n = sum(r[1] for r in rows)
            rmse[o] = float(np.sqrt(sum(r[0] for r in rows) / max(n, 1)))
            gains[o] = [r[2] for r in rows]
            n_tot += n
        results[name] = {"dead_time": d, "tau": tau, "rmse": rmse, "n": int(n_tot), "gains": gains,
                         "late_changes": n_late}
    return results
