"""Simulated robot population for the calibration paper's figures 2-3, before any hardware data exists.

    python calibration_paper/popsim.py loro      [--robots 8 --spread 1 --reps 3 --out loro.json]
    python calibration_paper/popsim.py variance  [--robots 8 --days 2]
    python calibration_paper/popsim.py power     [--sims 5]
    python calibration_paper/popsim.py selftest

What it does (PLAN.md section 3, phases B-C):
  population  robots drawn around sim.default_truth(): per-robot dead zones p0, exponents gamma, motor gains, CW
              bias; per-day gain and dead-zone drift; per-segment twist noise. Every number is synthetic: the
              spreads are guesses to be replaced by the measured robot / day / segment components.
  prior       a population prior over theta = (p0_L, p0_R, gamma_L, gamma_R, coef rows) from N fully calibrated
              robots: sample mean and covariance, shrunk towards its diagonal, plus a floor that shrinks as 1/sqrt(N)
              (N = 1 is the "one reference robot, inflated variance" fallback). Blocks: (p0, gamma) and each twist
              output's coefficient row; cross-block covariance is dropped (not estimable from a few robots).
  few-shot    MAP fit of a new robot from k segments: Nelder-Mead over (p0, gamma) with the coefficient rows solved
              in closed form (Gaussian-prior least squares), as forward_map.fit_power_law does without the prior.
  baselines   'scratch' (forward_map.fit_power_law on the k segments alone), 'ref' (prior mean used as is),
              'ref_gain' (prior mean plus a per-output affine correction: master/slave standardisation).
  designs     'random' segments from the robot's pool, or 'dopt': greedy Bayesian D-optimal cells on the prior
              (maximise log det of prior precision + sum J^T R^-1 J, J the twist Jacobian at the prior mean).
  loro        leave-one-robot-out: for every held-out robot, every prior size N, design and k, the RMSE of the
              predicted twist against the robot's noise-free truth over the whole grid (figure 3). On real data
              the same functions take measured pools and score against the robot's unused measured segments.
  variance    nested robot / day / segment variance components per twist output, method of moments on a balanced
              grid (figure 2), checked against the noise-free truth in the simulation.
  power       how well those components and the figure-3 curves are recovered for n_robots x spread, over
              repeated simulated populations.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import minimize

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "calibration", "pc"))
from graspion_calib import forward_map as fm  # noqa: E402
from graspion_calib.schedules import GRID  # noqa: E402
from graspion_calib.sim import default_truth  # noqa: E402

SCALE = np.asarray(fm.DEFAULT_SCALE, float)
NOISE = (0.003, 0.002, 0.05)          # per-segment twist noise SD (m/s, m/s, rad/s): a guess, not a measurement
NL_FLOOR = np.array([5.0, 5.0, 0.1, 0.1])   # prior SD floor on (p0_L, p0_R, gamma_L, gamma_R)
METHODS = ("hier", "ref_gain", "ref", "scratch")


# ---------------------------------------------------------------- parameter vector <-> map

def theta_of(m: fm.PowerLawMap):
    return np.r_[np.asarray(m.p0, float), np.asarray(m.gamma, float), np.asarray(m.coef, float).ravel()]


def map_of(theta, features):
    k = fm.FEATURES[features]
    theta = np.asarray(theta, float)
    return fm.PowerLawMap(p0=(float(theta[0]), float(theta[1])), gamma=(float(theta[2]), float(theta[3])),
                          coef=theta[4:].reshape(3, k).copy(), features=features)


def grid_cells(levels=GRID):
    return [(float(a), float(b)) for a in levels for b in levels]


# ---------------------------------------------------------------- synthetic population

@dataclass
class Spread:
    """Hyper-SDs of the population (synthetic). scaled(s) multiplies every robot and day spread by s.

    At s = 1 the robot SD on the grid is about 0.5 cm/s in u1 and 0.25 rad/s in omega (5-10 % of the envelope),
    the day SD a quarter of that; s = 0.3 and 3 bracket it. Replace with the measured components."""
    p0: float = 2.0            # drive units
    gamma: float = 0.03
    gain: float = 0.05         # lognormal SD on the motor-gain columns (f_L, f_R[, d])
    w_bias: float = 0.05       # rad/s, omega intercept (CW bias at L = R)
    u_bias: float = 0.001      # m/s, u1 / u2 intercepts
    day_gain: float = 0.02     # lognormal SD per day on each output row
    day_p0: float = 1.0        # drive units per day
    noise: tuple = NOISE

    def scaled(self, s):
        return Spread(self.p0 * s, self.gamma * s, self.gain * s, self.w_bias * s, self.u_bias * s,
                      self.day_gain * s, self.day_p0 * s, self.noise)


def population_mean(features="lin", d_cor=0.015):
    m = default_truth()
    if features == "cor":
        m = fm.PowerLawMap(p0=m.p0, gamma=m.gamma, coef=np.c_[m.coef, [d_cor, 0.0, 0.0]], features="cor")
    elif features != "lin":
        raise ValueError("population_mean supports 'lin' and 'cor'")
    return m


def draw_population(n_robots, n_days=1, spread=1.0, seed=0, features="lin", mean=None):
    """[{'id', 'days': [PowerLawMap per day]}]: per-robot draws around `mean`, per-day drift on top."""
    sp = Spread().scaled(spread)
    mean = population_mean(features) if mean is None else mean
    rng = np.random.default_rng(seed)
    robots = []
    for r in range(n_robots):
        p0 = np.clip(np.asarray(mean.p0) + rng.normal(0, sp.p0, 2), 20, 100)
        gamma = np.clip(np.asarray(mean.gamma) + rng.normal(0, sp.gamma, 2), 0.3, 2.0)
        coef = np.asarray(mean.coef, float).copy()
        coef[:, 1:3] *= np.exp(rng.normal(0, sp.gain, (3, 2)))
        if features == "cor":
            coef[0, 3] *= np.exp(rng.normal(0, sp.gain))
        coef[2, 0] += rng.normal(0, sp.w_bias)
        coef[0, 0] += rng.normal(0, sp.u_bias)
        coef[1, 0] += rng.normal(0, sp.u_bias)
        days = []
        for _ in range(n_days):
            c = coef.copy()
            c[:, 1:3] *= np.exp(rng.normal(0, sp.day_gain, (3, 1)))
            pd = np.clip(p0 + rng.normal(0, sp.day_p0, 2), 20, 100)
            days.append(fm.PowerLawMap(p0=tuple(pd), gamma=tuple(gamma), coef=c, features=features))
        robots.append({"id": r, "days": days})
    return robots


def segments(day_map, blocks=5, noise=NOISE, seed=0, cells=None):
    """A full-grid pool: every cell `blocks` times, twist = truth + noise. Returns (L, R, Y)."""
    cells = grid_cells() if cells is None else cells
    rng = np.random.default_rng(seed)
    LR = np.array(cells * blocks)
    L, R = LR[:, 0], LR[:, 1]
    Y = day_map.predict(L, R) + rng.normal(0, np.asarray(noise, float), (len(L), 3))
    return L, R, Y


def truth_on_grid(day_map, cells=None):
    cells = grid_cells() if cells is None else cells
    LR = np.array(cells)
    return LR[:, 0], LR[:, 1], day_map.predict(LR[:, 0], LR[:, 1])


# ---------------------------------------------------------------- population prior

@dataclass
class Prior:
    features: str
    mu: np.ndarray
    cov_nl: np.ndarray               # 4 x 4
    cov_rows: list                   # 3 blocks, k x k
    noise: np.ndarray                # per-output segment SD
    n_sources: int = 0
    meta: dict = field(default_factory=dict)

    @property
    def k(self):
        return fm.FEATURES[self.features]

    def mean_map(self):
        return map_of(self.mu, self.features)

    def row_prior(self, o):
        """(mu_o, precision_o) for output row o; the 'cor' rows u2 and omega use only their first 3 entries."""
        k = self.k
        mu = self.mu[4:].reshape(3, k)[o]
        cov = self.cov_rows[o]
        if self.features == "cor" and o > 0:
            mu, cov = mu[:3], cov[:3, :3]
        return mu, np.linalg.inv(cov)

    def full_precision(self):
        n = 4 + 3 * self.k
        P = np.zeros((n, n))
        P[:4, :4] = np.linalg.inv(self.cov_nl)
        for o in range(3):
            s = slice(4 + o * self.k, 4 + (o + 1) * self.k)
            P[s, s] = np.linalg.inv(self.cov_rows[o])
        return P

    @classmethod
    def from_maps(cls, maps, shrink=0.5, rel=0.2, noise=None):
        """Population prior from fully fitted maps. N = 1: diagonal floor only; N >= 2: shrunk sample covariance
        plus the floor divided by sqrt(N)."""
        maps = list(maps)
        features = maps[0].features
        k = fm.FEATURES[features]
        T = np.stack([theta_of(m) for m in maps])
        n = len(maps)
        mu = T.mean(axis=0)
        rows = mu[4:].reshape(3, k)
        floor = np.r_[NL_FLOOR, np.concatenate([rel * np.abs(r) + 0.1 * np.max(np.abs(r)) + 1e-9 for r in rows])]
        floor = floor / np.sqrt(n)
        if n == 1:
            S = np.zeros((len(mu), len(mu)))
        else:
            S = np.cov(T, rowvar=False, ddof=1)
            S = (1 - shrink) * S + shrink * np.diag(np.diag(S))
        cov = S + np.diag(floor ** 2)
        cov_nl = cov[:4, :4]
        cov_rows = [cov[4 + o * k: 4 + (o + 1) * k, 4 + o * k: 4 + (o + 1) * k] for o in range(3)]
        if noise is None:
            rm = [m.meta["rmse"] for m in maps if "rmse" in m.meta]
            noise = np.mean(rm, axis=0) if rm else np.asarray(NOISE, float)
        return cls(features, mu, cov_nl, cov_rows, np.asarray(noise, float), n,
                   meta={"shrink": shrink, "rel": rel, "sd": np.sqrt(np.diag(cov)).tolist()})


# ---------------------------------------------------------------- fits

def _rows_map(m, L, R, Y, prior, rowp):
    """Coefficient rows given (p0, gamma): Gaussian-prior least squares per output; 'cor' in two stages."""
    X, active = m.design(L, R)
    coef = np.zeros((3, fm.FEATURES[prior.features]))      # 'cor': 4 columns, the design has 3
    covs = [None] * 3
    Xa, Ya = X[active], Y[active]

    def solve(A, y, o):
        mu_o, P_o = rowp[o]
        s2 = prior.noise[o] ** 2
        H = A.T @ A / s2 + P_o
        c = np.linalg.solve(H, A.T @ y / s2 + P_o @ mu_o)
        return c, np.linalg.inv(H)

    if prior.features == "cor":
        for o in (1, 2):
            coef[o, :3], covs[o] = solve(Xa, Ya[:, o], o)
        Xu = np.c_[Xa, np.abs(Xa @ coef[2, :3])]
        coef[0], covs[0] = solve(Xu, Ya[:, 0], 0)
    else:
        for o in range(3):
            coef[o], covs[o] = solve(Xa, Ya[:, o], o)
    return coef, covs


def fit_map(L, R, Y, prior: Prior, maxiter=600):
    """MAP fit of a new robot from its segments under the population prior (k = 0 returns the prior mean)."""
    L, R, Y = np.asarray(L, float), np.asarray(R, float), np.asarray(Y, float)
    if len(L) == 0:
        m = prior.mean_map()
        m.meta = {"n": 0, "post_cov_rows": list(prior.cov_rows)}
        return m
    rowp = [prior.row_prior(o) for o in range(3)]
    Pn = np.linalg.inv(prior.cov_nl)
    mu_nl = prior.mu[:4]
    lo, hi = np.array([1.0, 1.0, 0.2, 0.2]), np.array([110.0, 110.0, 3.0, 3.0])

    def build(z):
        z = np.clip(z, lo, hi)
        return fm.PowerLawMap(p0=(z[0], z[1]), gamma=(z[2], z[3]), features=prior.features)

    def prior_pen(z, coef):
        dz = z - mu_nl
        pen = dz @ Pn @ dz
        for o in range(3):
            mu_o, P_o = rowp[o]
            dc = coef[o, :len(mu_o)] - mu_o
            pen += dc @ P_o @ dc
        return pen

    def obj(z):
        m = build(z)
        m.coef, _ = _rows_map(m, L, R, Y, prior, rowp)
        r = (Y - m.predict(L, R)) / prior.noise
        bound = float(np.sum(np.maximum(0, lo - z) ** 2 + np.maximum(0, z - hi) ** 2)) * 1e6
        return float(np.sum(r ** 2)) + prior_pen(np.clip(z, lo, hi), m.coef) + bound

    res = minimize(obj, mu_nl, method="Nelder-Mead",
                   options={"xatol": 1e-3, "fatol": 1e-9, "maxiter": maxiter, "maxfev": 2 * maxiter})
    m = build(res.x)
    m.coef, covs = _rows_map(m, L, R, Y, prior, rowp)
    m.meta = {"n": int(len(L)), "post_cov_rows": covs, "objective": float(res.fun)}
    return m


def predictive_sd(m: fm.PowerLawMap, L, R):
    """SD of the predicted mean twist per cell from the posterior of the coefficient rows (fit_map output)."""
    covs = m.meta.get("post_cov_rows")
    X, active = m.design(np.atleast_1d(L), np.atleast_1d(R))
    sd = np.zeros((len(X), 3))
    for o in range(3):
        cov = covs[o]
        Xo = X
        if m.features == "cor":
            Xo = np.c_[X, np.abs(X @ m.coef[2, :3])] if o == 0 else X[:, :3]
            cov = cov if o == 0 else cov[:3, :3]
        sd[:, o] = np.sqrt(np.maximum(np.einsum("ij,jk,ik->i", Xo, cov, Xo), 0))
    sd[~active] = 0.0
    return sd


class AffineOnRef:
    """'ref_gain': Y = a * ref(L, R) + b per output, the master/slave standardisation baseline."""

    def __init__(self, ref: fm.PowerLawMap, a=None, b=None):
        self.ref, self.a, self.b = ref, np.ones(3) if a is None else a, np.zeros(3) if b is None else b
        self.features = ref.features

    def predict(self, L, R):
        return self.ref.predict(L, R) * self.a + self.b

    @classmethod
    def fit(cls, L, R, Y, ref):
        a, b = np.ones(3), np.zeros(3)
        if len(L) >= 2:
            Yr = ref.predict(L, R)
            for o in range(3):
                x = Yr[:, o]
                if np.ptp(x) > 1e-9:
                    c, *_ = np.linalg.lstsq(np.c_[x, np.ones_like(x)], Y[:, o], rcond=None)
                    a[o], b[o] = c
                else:
                    b[o] = float(np.mean(Y[:, o] - x))
        elif len(L) == 1:
            b = (Y - ref.predict(L, R))[0]
        return cls(ref, a, b)


def fit_scratch(L, R, Y, features="lin", quick=False):
    """forward_map.fit_power_law on the segments alone; None when there is too little to fit."""
    active = (np.abs(L) > 0) | (np.abs(R) > 0)
    if len(L) < 8 or len(set(zip(np.abs(L[active]), np.abs(R[active])))) < 5:
        return None
    starts = [(a, a, g, g) for a in (30.0, 50.0) for g in (0.6, 1.2)] if quick else None
    return fm.fit_power_law(L, R, Y, features=features, starts=starts)


def score(model, L, R, Y_true):
    """RMSE per output and the scale-normalised score of forward_map.grouped_cv."""
    rmse = np.sqrt(np.mean((model.predict(L, R) - Y_true) ** 2, axis=0))
    return rmse, float(np.sqrt(np.mean((rmse / SCALE) ** 2)))


# ---------------------------------------------------------------- designs

def _pool_by_cell(L, R):
    by = {}
    for i, (a, b) in enumerate(zip(np.abs(L), np.abs(R))):
        by.setdefault((float(a), float(b)), []).append(i)
    return by


def pick_random(L, R, k, rng):
    return np.sort(rng.choice(len(L), size=min(k, len(L)), replace=False))


def twist_jacobian(prior: Prior, cells):
    """(n_cells, 3, n_theta) Jacobian of the twist with respect to theta at the prior mean (central differences)."""
    th = prior.mu
    LR = np.array(cells)
    h = np.r_[0.5, 0.5, 0.01, 0.01, 1e-3 * np.maximum(np.abs(th[4:]), 1e-3)]
    J = np.zeros((len(cells), 3, len(th)))
    for j in range(len(th)):
        tp, tm = th.copy(), th.copy()
        tp[j] += h[j]
        tm[j] -= h[j]
        J[:, :, j] = (map_of(tp, prior.features).predict(LR[:, 0], LR[:, 1])
                      - map_of(tm, prior.features).predict(LR[:, 0], LR[:, 1])) / (2 * h[j])
    return J


def pick_dopt(L, R, k, prior: Prior, rng):
    """Greedy Bayesian D-optimal choice of k segments from the pool (cells may repeat while the pool has them)."""
    by = _pool_by_cell(L, R)
    cells = list(by)
    for c in cells:
        rng.shuffle(by[c])
    J = twist_jacobian(prior, cells)
    Rinv = np.diag(1.0 / prior.noise ** 2)
    info = np.einsum("coi,op,cpj->cij", J, Rinv, J)      # per-cell J^T R^-1 J
    Lam = prior.full_precision()
    chosen = []
    for _ in range(min(k, len(L))):
        best, best_v = None, -np.inf
        for ci, c in enumerate(cells):
            if not by[c]:
                continue
            v = np.linalg.slogdet(Lam + info[ci])[1]
            if v > best_v:
                best, best_v = ci, v
        Lam = Lam + info[best]
        chosen.append(by[cells[best]].pop())
    return np.sort(np.array(chosen, int))


# ---------------------------------------------------------------- leave-one-robot-out (figure 3)

def full_fit(L, R, Y, features="lin"):
    return fm.fit_power_law(L, R, Y, features=features)


def loro(pools, evals, features="lin", prior_sizes=(1, 2, 4, 8), ks=(0, 1, 2, 4, 8, 16, 32, 64),
         designs=("random", "dopt"), reps=3, seed=0, quick=False, full_maps=None, log=None):
    """pools: {robot: (L, R, Y)} measured segments; evals: {robot: (L, R, Y)} what to score against (the
    noise-free truth on the grid in simulation; on real data the robot's segments outside the pool).

    Returns {"scores": {method: {N: {design: {k: [score per robot x rep]}}}}, "coverage": {...}, "full": {...}}.
    'scratch' and 'full' carry N = 0 and design '-' / the design, as they use no prior."""
    ids = sorted(pools)
    full_maps = full_maps or {}
    t0 = time.time()
    for r in ids:
        if r not in full_maps:
            full_maps[r] = full_fit(*pools[r], features=features)
    res, cov = {}, {}
    full = {}

    def put(d, method, N, design, k, v):
        d.setdefault(method, {}).setdefault(int(N), {}).setdefault(design, {}).setdefault(int(k), []).append(v)

    for r in ids:
        full[r] = score(full_maps[r], *evals[r])[1]
        others = [s for s in ids if s != r]
        Le, Re, Ye = evals[r]
        L, R, Y = pools[r]
        for rep in range(reps):
            rng = np.random.default_rng(seed + 1000 * rep + r)
            for design in designs:
                scratch_done = set()
                for N in [n for n in prior_sizes if n <= len(others)]:
                    src = rng.choice(others, size=N, replace=False)
                    prior = Prior.from_maps([full_maps[s] for s in src])
                    ref = prior.mean_map()
                    for k in ks:
                        idx = pick_random(L, R, k, rng) if design == "random" else pick_dopt(L, R, k, prior, rng)
                        Lk, Rk, Yk = L[idx], R[idx], Y[idx]
                        m = fit_map(Lk, Rk, Yk, prior)
                        put(res, "hier", N, design, k, score(m, Le, Re, Ye)[1])
                        sd = predictive_sd(m, Le, Re)
                        ok = np.abs(m.predict(Le, Re) - Ye) <= 1.96 * sd + 1e-12
                        put(cov, "hier", N, design, k, float(np.mean(ok[np.any(sd > 0, axis=1)])))
                        put(res, "ref", N, design, k, score(ref, Le, Re, Ye)[1])
                        put(res, "ref_gain", N, design, k, score(AffineOnRef.fit(Lk, Rk, Yk, ref), Le, Re, Ye)[1])
                        if k not in scratch_done:
                            s = fit_scratch(Lk, Rk, Yk, features, quick=quick)
                            put(res, "scratch", 0, design, k, np.nan if s is None else score(s, Le, Re, Ye)[1])
                            scratch_done.add(k)
        if log:
            log(f"  robot {r}: full-fit score {full[r]:.3f}  ({time.time() - t0:.0f} s)")
    return {"scores": res, "coverage": cov, "full": {int(r): v for r, v in full.items()},
            "ks": list(ks), "prior_sizes": list(prior_sizes), "designs": list(designs)}


def summarize_loro(out):
    """{method: {N: {design: {k: (median, q25, q75)}}}} over robots x reps, NaN-aware."""
    s = {}
    for method, d1 in out["scores"].items():
        for N, d2 in d1.items():
            for design, d3 in d2.items():
                for k, v in d3.items():
                    a = np.asarray(v, float)
                    q = np.nanpercentile(a, [50, 25, 75]) if np.any(np.isfinite(a)) else [np.nan] * 3
                    s.setdefault(method, {}).setdefault(N, {}).setdefault(design, {})[k] = [float(x) for x in q]
    return s


def print_loro(out):
    s = summarize_loro(out)
    full = np.asarray(list(out["full"].values()))
    print(f"full-grid fit (floor): median score {np.median(full):.3f}  [{np.percentile(full, 25):.3f}, "
          f"{np.percentile(full, 75):.3f}]   (score 1 = one noise scale: {tuple(SCALE.tolist())})")
    for design in out["designs"]:
        cols = [("scratch", 0)] + [(m, N) for N in out["prior_sizes"] for m in ("ref", "ref_gain", "hier")
                                   if N in s.get(m, {})]
        print(f"\ndesign {design}: median score vs k (columns method/N)")
        print("   k  " + "".join(f"{m}/{N}".rjust(12) for m, N in cols))
        for k in out["ks"]:
            row = []
            for m, N in cols:
                v = s.get(m, {}).get(N, {}).get(design, {}).get(k)
                row.append("         nan" if v is None or not np.isfinite(v[0]) else f"{v[0]:12.3f}")
            print(f"{k:4d}  " + "".join(row))
    c = summarize_loro({"scores": out["coverage"]})
    print("\n95 % interval coverage of the true mean twist, 'hier', random design: "
          + ", ".join(f"N{N} k{k} {v[0]:.2f}" for N in sorted(c["hier"]) for k, v in sorted(
              c["hier"][N].get("random", {}).items()) if k in (0, 4, 16, 64)))


# ---------------------------------------------------------------- variance components (figure 2)

def variance_components(sessions):
    """sessions: {robot: [(L, R, Y) per day]}, every (robot, day) a full grid with the same replicate count.

    Nested random effects per cell and output (robot > day within robot > segment), method of moments on the
    balanced layout, variance components averaged over cells with any motion. Returns per output the SDs
    (robot, day, segment) and their fractions of the total; day is None with a single day."""
    ids = sorted(sessions)
    n_days = min(len(sessions[r]) for r in ids)
    cells = {}
    for r in ids:
        for d in range(n_days):
            L, R, Y = sessions[r][d]
            by = _pool_by_cell(L, R)
            for c, idx in by.items():
                cells.setdefault(c, {}).setdefault(r, {})[d] = Y[idx]
    a, b = len(ids), n_days
    comps = []
    for c, per_r in cells.items():
        if c == (0.0, 0.0) or len(per_r) < a:
            continue
        n = min(len(per_r[r][d]) for r in ids for d in range(b))
        y = np.stack([[per_r[r][d][:n] for d in range(b)] for r in ids])   # a x b x n x 3
        if np.max(np.abs(y)) < 1e-9:
            continue
        gm = y.mean(axis=(0, 1, 2))
        rm = y.mean(axis=(1, 2))
        dm = y.mean(axis=2)
        ms_err = np.sum((y - dm[:, :, None]) ** 2, axis=(0, 1, 2)) / max(a * b * (n - 1), 1)
        if b > 1:
            ms_day = np.sum(n * (dm - rm[:, None]) ** 2, axis=(0, 1)) / (a * (b - 1))
            ms_rob = np.sum(n * b * (rm - gm) ** 2, axis=0) / (a - 1)
            v_day = np.maximum((ms_day - ms_err) / n, 0)
            v_rob = np.maximum((ms_rob - ms_day) / (n * b), 0)
        else:
            ms_rob = np.sum(n * (rm - gm) ** 2, axis=0) / (a - 1)
            v_day = np.full(3, np.nan)
            v_rob = np.maximum((ms_rob - ms_err) / n, 0)
        comps.append(np.stack([v_rob, v_day, ms_err]))
    v = np.mean(np.stack(comps), axis=0)          # 3 components x 3 outputs
    total = np.nansum(v, axis=0)
    return {"sd": {"robot": np.sqrt(v[0]).tolist(), "day": None if b == 1 else np.sqrt(v[1]).tolist(),
                   "segment": np.sqrt(v[2]).tolist()},
            "fraction": {"robot": (v[0] / total).tolist(), "day": None if b == 1 else (v[1] / total).tolist(),
                         "segment": (v[2] / total).tolist()},
            "n_robots": a, "n_days": b, "n_cells": len(comps)}


def true_components(robots):
    """Noise-free robot and day SDs per output on the grid, from the population's day maps."""
    L, R, _ = truth_on_grid(robots[0]["days"][0])
    keep = ~((L == 0) & (R == 0))
    T = np.stack([[m.predict(L, R)[keep] for m in r["days"]] for r in robots])   # a x b x cells x 3
    rm = T.mean(axis=1)
    v_rob = np.mean(np.var(rm, axis=0, ddof=1), axis=0)
    v_day = np.mean(np.var(T, axis=1, ddof=1), axis=(0, 1)) if T.shape[1] > 1 else np.full(3, np.nan)
    return {"robot": np.sqrt(v_rob).tolist(), "day": None if T.shape[1] == 1 else np.sqrt(v_day).tolist()}


# ---------------------------------------------------------------- simulation drivers

def simulate(n_robots, n_days=1, spread=1.0, blocks=5, seed=0, features="lin", noise=NOISE):
    robots = draw_population(n_robots, n_days, spread, seed, features)
    sessions = {r["id"]: [segments(m, blocks, noise, seed=seed * 7919 + 101 * r["id"] + d)
                          for d, m in enumerate(r["days"])] for r in robots}
    pools = {r: s[0] for r, s in sessions.items()}
    evals = {r["id"]: truth_on_grid(r["days"][0]) for r in robots}
    return robots, sessions, pools, evals


def run_loro(a):
    robots, sessions, pools, evals = simulate(a.robots, 1, a.spread, a.blocks, a.seed, a.features)
    ks = (0, 2, 8, 32) if a.quick else (0, 1, 2, 4, 8, 16, 32, 64)
    sizes = tuple(n for n in (1, 2, 4, 8) if n <= a.robots - 1)
    out = loro(pools, evals, a.features, sizes, ks, ("random", "dopt"), a.reps, a.seed, a.quick, log=print)
    out["config"] = vars(a)
    print_loro(out)
    return out


def run_variance(a):
    robots, sessions, _, _ = simulate(a.robots, a.days, a.spread, a.blocks, a.seed, a.features)
    est = variance_components(sessions)
    tru = true_components(robots)
    out = {"estimate": est, "truth": tru, "noise": list(NOISE), "config": vars(a)}
    names = ("u1 [m/s]", "u2 [m/s]", "omega [rad/s]")
    print(f"variance components, {a.robots} robots x {a.days} days x {a.blocks} replicates (spread {a.spread})")
    print("output          robot SD (true)      day SD (true)      segment SD (true)   fractions r/d/s")
    for o in range(3):
        d_est = "-" if est["sd"]["day"] is None else f"{est['sd']['day'][o]:.4f}"
        d_tru = "-" if tru["day"] is None else f"{tru['day'][o]:.4f}"
        fr = est["fraction"]
        f_day = "-" if fr["day"] is None else f"{fr['day'][o]:.2f}"
        print(f"{names[o]:14s}  {est['sd']['robot'][o]:.4f} ({tru['robot'][o]:.4f})    {d_est} ({d_tru})      "
              f"{est['sd']['segment'][o]:.4f} ({NOISE[o]:.4f})     {fr['robot'][o]:.2f}/{f_day}/{fr['segment'][o]:.2f}")
    return out


def run_power(a):
    spreads = (0.3, 1.0, 3.0)
    sizes = (4, 6, 8) if not a.quick else (4,)
    ks = (0, 4, 16)
    out = {"spreads": spreads, "n_robots": sizes, "ks": ks, "runs": [], "config": vars(a)}
    for sp in spreads:
        # the population-level robot SD (what n robots are supposed to estimate), from a large draw
        pop_sd = true_components(draw_population(2000, 1, sp, a.seed + 7, a.features))["robot"]
        for n in sizes:
            rows = []
            for s in range(a.sims):
                seed = a.seed + 100 * s
                robots, sessions, pools, evals = simulate(n, a.days, sp, a.blocks, seed, a.features)
                est = variance_components(sessions)
                tru = true_components(robots)
                lo = loro(pools, evals, a.features, (1, n - 1), ks, ("random",), 1, seed, True)
                sm = summarize_loro(lo)
                rows.append({"seed": seed, "robot_sd_est": est["sd"]["robot"], "robot_sd_true": tru["robot"],
                             "robot_sd_pop": pop_sd,
                             "hier1": {k: sm["hier"][1]["random"][k][0] for k in ks},
                             "hierN": {k: sm["hier"][n - 1]["random"][k][0] for k in ks},
                             "scratch": {k: sm["scratch"][0]["random"][k][0] for k in ks},
                             "full": float(np.median(list(lo["full"].values())))})
                print(f"spread {sp} n {n} sim {s}: omega robot SD est {est['sd']['robot'][2]:.3f} "
                      f"true {tru['robot'][2]:.3f}; hier(N={n - 1}) k4 {rows[-1]['hierN'][4]:.2f} "
                      f"k16 {rows[-1]['hierN'][16]:.2f}; scratch k16 {rows[-1]['scratch'][16]:.2f}; "
                      f"full {rows[-1]['full']:.2f}")
            out["runs"].append({"spread": sp, "n_robots": n, "rows": rows})
    print("\nsummary (median over simulations): estimated robot SD of omega over the population's (n robots stand "
          "in for the population), and figure-3 scores")
    print("spread  n   SDratio[q10,q90]   hier/N=1 k0  k4  k16 | hier/N=n-1 k0  k4  k16 | scratch k16 | full")
    for run in out["runs"]:
        rr = run["rows"]
        ratio = np.array([r["robot_sd_est"][2] / max(r["robot_sd_pop"][2], 1e-9) for r in rr])
        med = lambda key, k: np.nanmedian([r[key][k] for r in rr])
        print(f"{run['spread']:5.1f}  {run['n_robots']}   {np.median(ratio):.2f} [{np.percentile(ratio, 10):.2f},"
              f"{np.percentile(ratio, 90):.2f}]     {med('hier1', 0):5.2f} {med('hier1', 4):5.2f} {med('hier1', 16):5.2f}"
              f" |  {med('hierN', 0):5.2f} {med('hierN', 4):5.2f} {med('hierN', 16):5.2f} |  {med('scratch', 16):5.2f}"
              f"  | {np.median([r['full'] for r in rr]):.2f}")
    return out


def _json_ready(o):
    if isinstance(o, dict):
        return {str(k): _json_ready(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_json_ready(v) for v in o]
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (np.floating, np.integer)):
        return o.item()
    return o


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("command", choices=("loro", "variance", "power", "selftest"))
    ap.add_argument("--robots", type=int, default=8)
    ap.add_argument("--days", type=int, default=2)
    ap.add_argument("--spread", type=float, default=1.0, help="multiplier on every population spread")
    ap.add_argument("--blocks", type=int, default=5, help="replicates per cell in a full grid")
    ap.add_argument("--reps", type=int, default=3, help="random subsets per held-out robot")
    ap.add_argument("--sims", type=int, default=5, help="simulated populations per power cell")
    ap.add_argument("--features", default="lin", choices=("lin", "cor"))
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--quick", action="store_true")
    ap.add_argument("--out")
    a = ap.parse_args(argv)
    if a.command == "selftest":
        import unittest
        suite = unittest.defaultTestLoader.discover(os.path.dirname(os.path.abspath(__file__)), "test_popsim.py")
        ok = unittest.TextTestRunner(verbosity=1).run(suite).wasSuccessful()
        sys.exit(0 if ok else 1)
    out = {"loro": run_loro, "variance": run_variance, "power": run_power}[a.command](a)
    if a.out:
        with open(a.out, "w") as f:
            json.dump(_json_ready(out), f, indent=1)
        print(f"wrote {a.out}")


if __name__ == "__main__":
    main()
