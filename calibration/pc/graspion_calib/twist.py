"""Per-segment body twist (u1, u2, omega) and ICR from a tracked path; body-frame velocity series.

Constant-twist model for one segment (after the start transient)::

    theta(t) = theta0 + omega t
    P(t)     = P0 + int_0^t Rot(theta(s)) [u1, u2] ds

omega comes from a line fit to the unwrapped heading; (P0, u1, u2) from one linear least-squares problem that
uses the *measured* heading inside the integral, so heading diffusion during the segment does not bias u1/u2.
The problem stays well posed for straight motion (omega -> 0) and for pure pivots (u -> 0).
"""
from __future__ import annotations

import numpy as np
from scipy.signal import savgol_filter

from .geometry import icr_body


def _cumtrapz0(y, t):
    return np.r_[0.0, np.cumsum(0.5 * (y[1:] + y[:-1]) * np.diff(t))]


def heading_spikes(th, k=2, thresh=0.5):
    """Samples whose heading differs from the circular mean of their +-k neighbours by more than thresh rad.

    Needed before unwrapping: a single pi flip is ambiguous to any increment-based unwrap and would shift the
    rest of the path by 2 pi."""
    th = np.asarray(th, float)
    n = len(th)
    c, s = np.cos(th), np.sin(th)
    bad = np.zeros(n, bool)
    for i in range(n):
        lo, hi = max(0, i - k), min(n, i + k + 1)
        cs, ss = c[lo:hi].sum() - c[i], s[lo:hi].sum() - s[i]
        if hi - lo > 1:
            bad[i] = abs(np.angle(np.exp(1j * (th[i] - np.arctan2(ss, cs))))) > thresh
    return bad


def fit_twist(t, x, y, th, reject_k=5.0):
    """Fit a constant body twist. Returns a dict (u1, u2, omega, se_*, icr_x/y, pos_rms, th_rms, n, residuals).

    t in s, x/y in m, th in rad (any wrapping). Heading spikes are dropped before unwrapping, then one pass of
    outlier rejection (residual > reject_k * robust scale). Standard errors assume white residuals; tracked
    residuals are autocorrelated, so treat them as lower bounds.
    """
    spikes = heading_spikes(th)
    t = np.asarray(t, float)[~spikes]
    x, y = np.asarray(x, float)[~spikes], np.asarray(y, float)[~spikes]
    th = np.unwrap(np.asarray(th, float)[~spikes])
    keep = np.ones(len(t), bool)
    for it in range(2):
        tt, xx, yy, hh = t[keep] - t[keep][0], x[keep], y[keep], th[keep]
        n = len(tt)
        if n < 6:
            raise ValueError("too few samples for a twist fit")
        A_th = np.c_[np.ones(n), tt]
        (th0, omega), *_ = np.linalg.lstsq(A_th, hh, rcond=None)
        Ic, Is = _cumtrapz0(np.cos(hh), tt), _cumtrapz0(np.sin(hh), tt)
        A = np.zeros((2 * n, 4))
        A[:n, 0] = 1.0
        A[n:, 1] = 1.0
        A[:n, 2], A[:n, 3] = Ic, -Is          # x: cos(th) u1 - sin(th) u2
        A[n:, 2], A[n:, 3] = Is, Ic           # y: sin(th) u1 + cos(th) u2
        b = np.r_[xx, yy]
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
        res = b - A @ sol
        th_res = hh - (th0 + omega * tt)
        if it == 1:
            break
        rp = np.hypot(res[:n], res[n:])                       # Rayleigh-like: median ~ 1.18 sigma
        mad_h = 1.4826 * np.median(np.abs(th_res - np.median(th_res)))
        bad = (rp > reject_k * np.median(rp) + 1e-4) | (np.abs(th_res) > reject_k * mad_h + 0.02)
        if not bad.any():
            break
        keep[np.flatnonzero(keep)[bad]] = False
    dof = max(1, 2 * n - 4)
    s2 = float(res @ res) / dof
    try:
        cov = s2 * np.linalg.inv(A.T @ A)
        se_u1, se_u2 = float(np.sqrt(cov[2, 2])), float(np.sqrt(cov[3, 3]))
    except np.linalg.LinAlgError:
        se_u1 = se_u2 = float("nan")
    s2h = float(th_res @ th_res) / max(1, n - 2)
    se_w = float(np.sqrt(s2h / max(1e-12, np.sum((tt - tt.mean()) ** 2))))
    u1, u2 = float(sol[2]), float(sol[3])
    icx, icy = icr_body(u1, u2, float(omega))
    return {"u1": u1, "u2": u2, "omega": float(omega), "se_u1": se_u1, "se_u2": se_u2, "se_omega": se_w,
            "icr_x": icx, "icr_y": icy, "x0": float(sol[0]), "y0": float(sol[1]), "th0": float(th0),
            "pos_rms": float(np.sqrt(np.mean(res ** 2))), "th_rms": float(np.sqrt(np.mean(th_res ** 2))),
            "n": int(n), "n_rejected": int((~keep).sum() + spikes.sum()), "duration": float(tt[-1]),
            "t": t[keep], "th_resid": th_res}


def uniform_chunks(t, max_gap_factor=1.6, min_len=8):
    """Split sample indices into runs without dropped frames. Returns (dt, [index arrays])."""
    t = np.asarray(t, float)
    if len(t) < 2:
        return np.nan, []
    d = np.diff(t)
    dt = float(np.median(d))
    breaks = np.flatnonzero(d > max_gap_factor * dt) + 1
    return dt, [c for c in np.split(np.arange(len(t)), breaks) if len(c) >= min_len]


def resample_uniform(t, *ys, dt=None):
    """Linear resampling of one gap-free run onto a uniform grid."""
    t = np.asarray(t, float)
    if dt is None:
        dt = float(np.median(np.diff(t)))
    tu = np.arange(t[0], t[-1] + 0.5 * dt, dt)
    tu = tu[tu <= t[-1]]
    return (tu,) + tuple(np.interp(tu, t, np.asarray(y, float)) for y in ys)


def body_velocity(t, x, y, th, window_s=0.15, polyorder=2):
    """Body-frame velocity (u1, u2, omega) series by Savitzky-Golay differentiation on gap-free runs.

    Returns (t_uniform, u1, u2, omega, window_samples); NaN-free, concatenated over runs. Positions are
    differentiated in the world frame and rotated with the smoothed heading.
    """
    out = [[], [], [], []]
    t = np.asarray(t, float)
    th = np.unwrap(np.asarray(th, float))
    dt, runs = uniform_chunks(t)
    win = 0
    for idx in runs:
        tu, xu, yu, hu = resample_uniform(t[idx], x[idx], y[idx], th[idx], dt=dt)
        win = max(polyorder + 2, int(round(window_s / dt)) | 1)
        if len(tu) < win + 2:
            continue
        vx = savgol_filter(xu, win, polyorder, deriv=1, delta=dt)
        vy = savgol_filter(yu, win, polyorder, deriv=1, delta=dt)
        w = savgol_filter(hu, win, polyorder, deriv=1, delta=dt)
        hs = savgol_filter(hu, win, polyorder)
        c, s = np.cos(hs), np.sin(hs)
        out[0].append(tu); out[1].append(c * vx + s * vy); out[2].append(-s * vx + c * vy); out[3].append(w)
    if not out[0]:
        return (np.array([]),) * 4 + (win,)
    return tuple(np.concatenate(o) for o in out) + (win,)
