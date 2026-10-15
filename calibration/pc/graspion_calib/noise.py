"""Heading diffusion, measurement noise, speed jitter and lateral slip.

Heading diffusion uses second differences of the heading, which cancel any constant turn rate exactly (no bias
from estimating omega inside short segments, and segments with different omega can be pooled)::

    S(tau) = E[(th(t + 2 tau) - 2 th(t + tau) + th(t))^2] = 4 V(tau) - V(2 tau) + 6 sigma_m^2

with V(tau) the variance of a heading increment. Brownian heading (white omega noise): V = 2 D_r tau, so
S = 4 D_r tau + 6 sigma_m^2. OU-correlated omega (variance s2, correlation time tc):
V(tau) = 2 s2 tc^2 (tau/tc - 1 + exp(-tau/tc)), long-time D_r = s2 tc.
"""
from __future__ import annotations

import numpy as np
from scipy.optimize import least_squares

from .twist import body_velocity, resample_uniform, uniform_chunks


def second_difference_msd(runs, max_lag_s=1.0, min_count=20):
    """Pool S(tau) over runs. runs: iterable of (t, th) arrays (each may contain gaps).

    Returns (tau [s], S [rad^2], counts)."""
    sums, counts, dt_ref = {}, {}, None
    for t, th in runs:
        t = np.asarray(t, float)
        th = np.unwrap(np.asarray(th, float))
        dt, chunks = uniform_chunks(t)
        if not chunks:
            continue
        dt_ref = dt if dt_ref is None else dt_ref
        for idx in chunks:
            tu, hu = resample_uniform(t[idx], th[idx], dt=dt_ref)
            kmax = min(int(max_lag_s / dt_ref), (len(tu) - 1) // 2)
            for k in range(1, kmax + 1):
                d2 = hu[2 * k:] - 2 * hu[k:-k] + hu[:-2 * k]
                sums[k] = sums.get(k, 0.0) + float(np.sum(d2 ** 2))
                counts[k] = counts.get(k, 0) + len(d2)
    ks = sorted(k for k in sums if counts[k] >= min_count)
    if not ks:
        return np.array([]), np.array([]), np.array([])
    return (np.array(ks) * dt_ref, np.array([sums[k] / counts[k] for k in ks]), np.array([counts[k] for k in ks]))


def fit_brownian(tau, S, counts=None):
    """S = 4 D tau + 6 sigma^2. Returns D_r [rad^2/s], sigma_meas [rad].

    Weights sqrt(count) / S: the sampling SD of each S(tau) estimate grows roughly in proportion to S."""
    w = (np.ones_like(tau) if counts is None else np.sqrt(counts)) / np.maximum(S, 1e-15)
    A = np.c_[4 * tau, 6 * np.ones_like(tau)] * w[:, None]
    (D, s2), *_ = np.linalg.lstsq(A, S * w, rcond=None)
    return {"D_r": float(D), "sigma_meas": float(np.sqrt(max(s2, 0.0)))}


def _V_ou(tau, s2, tc):
    x = tau / tc
    return 2 * s2 * tc ** 2 * (x - 1 + np.exp(-x))


def fit_ou(tau, S, counts=None):
    """OU-correlated turn-rate noise. Returns s2, tc, D_r = s2 tc, sigma_meas, and the cost ratio vs Brownian."""
    w = (np.ones_like(tau) if counts is None else np.sqrt(counts)) / np.maximum(S, 1e-15)
    br = fit_brownian(tau, S, counts)

    def res(z):
        s2, tc, sm2 = np.exp(np.clip(z, -30, 10))
        return (4 * _V_ou(tau, s2, tc) - _V_ou(2 * tau, s2, tc) + 6 * sm2 - S) * w

    tc0 = max(tau[0], 0.05)
    z0 = np.log([max(br["D_r"], 1e-6) / tc0, tc0, max(br["sigma_meas"] ** 2, 1e-8)])
    r = least_squares(res, z0, method="lm" if len(tau) > 3 else "trf")
    s2, tc, sm2 = np.exp(np.clip(r.x, -30, 10))
    rb = (4 * br["D_r"] * tau + 6 * br["sigma_meas"] ** 2 - S) * w
    return {"s2": float(s2), "tc": float(tc), "D_r": float(s2 * tc), "sigma_meas": float(np.sqrt(sm2)),
            "cost_ou": float(np.sum(r.fun ** 2)), "cost_brownian": float(np.sum(rb ** 2))}


def heading_diffusion(runs, max_lag_s=1.0, ou=False):
    tau, S, n = second_difference_msd(runs, max_lag_s=max_lag_s)
    if len(tau) < 3:
        return {"D_r": float("nan"), "sigma_meas": float("nan"), "n_lags": int(len(tau))}
    out = fit_brownian(tau, S, n)
    out.update({"n_lags": int(len(tau)), "tau": tau.tolist(), "S": S.tolist()})
    if ou and len(tau) >= 5:
        out["ou"] = fit_ou(tau, S, n)
    return out


def static_noise(t, x, y, th):
    """Measurement noise of a stationary marker (60 s static run): SD of x, y, theta and of frame-to-frame steps."""
    th = np.unwrap(np.asarray(th, float))
    x, y = np.asarray(x, float), np.asarray(y, float)
    return {"sd_x": float(np.std(x)), "sd_y": float(np.std(y)), "sd_th": float(np.std(th)),
            "sd_dx_step": float(np.std(np.diff(x)) / np.sqrt(2)), "sd_dy_step": float(np.std(np.diff(y)) / np.sqrt(2)),
            "sd_dth_step": float(np.std(np.diff(th)) / np.sqrt(2)), "n": int(len(x)),
            "drift_x": float(x[-1] - x[0]), "drift_y": float(y[-1] - y[0])}


def velocity_jitter(t, x, y, th, u1, u2, omega, window_s=0.2):
    """Fluctuation of the body-frame velocity around the segment's fitted twist (speed jitter, lateral slip)."""
    tv, a, b, w, _ = body_velocity(t, x, y, th, window_s=window_s)
    if len(tv) < 5:
        return {"sd_u1": float("nan"), "sd_u2": float("nan"), "sd_omega": float("nan")}
    return {"sd_u1": float(np.std(a - u1)), "sd_u2": float(np.std(b - u2)), "sd_omega": float(np.std(w - omega)),
            "window_s": window_s}
