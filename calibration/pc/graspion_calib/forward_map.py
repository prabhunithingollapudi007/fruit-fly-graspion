"""Forward map (L, R[, V_bat]) -> body twist (u1, u2, omega): parametric dead-zone power law (+ optional GP).

Model (FEASIBILITY.md 5.3)::

    f_k   = max(0, |p_k| - p0_k) ** gamma_k        k in {L, R}; |p| because only |drive| matters (paper)
    twist = C @ phi(f_L, f_R[, dV])                if f_L > 0 or f_R > 0, else 0 (both motors in the dead zone)

with feature sets
    'lin' : phi = [1, f_L, f_R]
    'int' : phi = [1, f_L, f_R, f_L f_R]
    'bat' : phi = [1, f_L, f_R, f_L dV, f_R dV]     dV = V_bat - v_ref (volts), needs per-segment battery data
    'cor' : omega, u2 on [1, f_L, f_R];  u1 = v(f) + d |w_hat|,  v(f) on [1, f_L, f_R], w_hat = model omega.
            EllipticalBot2D / digital-twin kinematics: rotation about a CoR offset d to the left (CCW) or
            right (CW) of the reference point, plus translation v along the heading. Only the mean offset is
            identifiable from steady-state twists: d_ccw w+ + d_cw w- = d|w| + (d_ccw - d_cw)/2 w, and the last
            term is linear in f, so it is absorbed by v(f). Use dist_rotPoint_to_CoM = [d, d] with this v(f) in
            EllipticalBot2D (exactly equivalent at steady state). (v, omega) is linear in f: closed-form inverse.

(p0, gamma) are shared by the three outputs (they describe the motors); C is solved by linear least squares
inside a Nelder-Mead search over (p0_L, p0_R, gamma_L, gamma_R) (variable projection). Variants are compared
by leave-cell-out cross-validation: every replicate of a drive cell (|L|, |R|) is held out together, so the
score measures interpolation to unseen drive levels rather than replicate noise.
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import minimize

OUTPUTS = ("u1", "u2", "omega")
FEATURES = {"lin": 3, "int": 4, "bat": 5, "cor": 4}
DEFAULT_SCALE = (0.005, 0.005, 0.1)   # noise scales (m/s, m/s, rad/s) that make the three outputs comparable


@dataclass
class PowerLawMap:
    p0: tuple = (55.0, 55.0)
    gamma: tuple = (1.0, 1.0)
    coef: np.ndarray = field(default_factory=lambda: np.zeros((3, 3)))   # rows u1, u2, omega
    features: str = "lin"
    v_ref: float = 3.8
    meta: dict = field(default_factory=dict)

    def activations(self, L, R):
        L, R = np.abs(np.asarray(L, float)), np.abs(np.asarray(R, float))
        fL = np.maximum(0.0, L - self.p0[0]) ** self.gamma[0]
        fR = np.maximum(0.0, R - self.p0[1]) ** self.gamma[1]
        return fL, fR

    def design(self, L, R, V=None):
        fL, fR = self.activations(L, R)
        cols = [np.ones_like(fL), fL, fR]
        if self.features == "int":
            cols.append(fL * fR)
        elif self.features == "bat":
            if V is None:
                raise ValueError("the 'bat' model needs battery voltages")
            dV = np.asarray(V, float) - self.v_ref
            cols += [fL * dV, fR * dV]
        return np.stack(cols, axis=-1), (fL > 0) | (fR > 0)

    def predict(self, L, R, V=None):
        X, active = self.design(np.atleast_1d(L), np.atleast_1d(R), None if V is None else np.atleast_1d(V))
        C = np.asarray(self.coef)
        if self.features == "cor":
            w = X @ C[2, :3]
            u1 = X @ C[0, :3] + C[0, 3] * np.abs(w)
            Y = np.stack([u1, X @ C[1, :3], w], axis=-1)
        else:
            Y = X @ C.T
        Y[~active] = 0.0
        return Y

    def cor_offset(self):
        """Mean CoR offset d [m] of the 'cor' model (0 for the other models)."""
        return float(np.asarray(self.coef)[0, 3]) if self.features == "cor" else 0.0

    def to_dict(self):
        return {"model": "power_law", "features": self.features, "p0": list(map(float, self.p0)),
                "gamma": list(map(float, self.gamma)), "coef": np.asarray(self.coef).tolist(),
                "outputs": list(OUTPUTS), "v_ref": self.v_ref, "meta": self.meta}

    @classmethod
    def from_dict(cls, d):
        return cls(p0=tuple(d["p0"]), gamma=tuple(d["gamma"]), coef=np.array(d["coef"], float),
                   features=d.get("features", "lin"), v_ref=d.get("v_ref", 3.8), meta=d.get("meta", {}))

    def save(self, path):
        with open(path, "w") as f:
            json.dump(self.to_dict(), f, indent=2)

    @classmethod
    def load(cls, path):
        with open(path) as f:
            return cls.from_dict(json.load(f))


def _solve_coef(m: PowerLawMap, L, R, V, Y, w, scale):
    """Weighted LS for C given (p0, gamma). Returns (coef, weighted SSE over all rows, scaled).

    Rows the model puts in the dead zone predict zero twist and enter the SSE as such."""
    X, active = m.design(L, R, V)
    k = FEATURES[m.features]
    coef = np.zeros((3, k))
    if active.sum() < k + 1:
        return coef, 1e12
    Xa, Ya = X[active], Y[active]
    sw = np.sqrt(w[active])[:, None]
    if m.features == "cor":                      # omega and u2 first, then u1 on [v features, |w_hat|]
        c12, *_ = np.linalg.lstsq(Xa * sw, Ya[:, 1:] * sw, rcond=None)
        coef[1:, :3] = c12.T
        Xu = np.c_[Xa, np.abs(Xa @ coef[2, :3])]
        c0, *_ = np.linalg.lstsq(Xu * sw, Ya[:, 0] * sw[:, 0], rcond=None)
        coef[0] = c0
    else:
        ct, *_ = np.linalg.lstsq(Xa * sw, Ya * sw, rcond=None)
        coef = ct.T
    m.coef = coef
    r = (Y - m.predict(L, R, V)) / scale
    return coef, float(np.sum(w[:, None] * r ** 2))


def fit_power_law(L, R, Y, V=None, weights=None, features="lin", scale=DEFAULT_SCALE, v_ref=3.8,
                  p0_bounds=(0.0, 100.0), gamma_bounds=(0.2, 2.5), starts=None):
    """Fit the dead-zone power-law model. L, R: signed drive per segment; Y: (n, 3) twists (u1, u2, omega)."""
    L, R, Y = np.asarray(L, float), np.asarray(R, float), np.asarray(Y, float)
    w = np.ones(len(L)) if weights is None else np.asarray(weights, float)
    scale = np.asarray(scale, float)
    if features == "bat" and V is None:
        raise ValueError("features='bat' needs V")
    lo = np.array([p0_bounds[0], p0_bounds[0], gamma_bounds[0], gamma_bounds[0]])
    hi = np.array([p0_bounds[1], p0_bounds[1], gamma_bounds[1], gamma_bounds[1]])
    pmin = max(1.0, min(np.min(np.abs(L[L != 0])) if np.any(L != 0) else 60.0,
                        np.min(np.abs(R[R != 0])) if np.any(R != 0) else 60.0))

    def unpack(z):
        z = np.clip(z, lo, hi)
        return PowerLawMap(p0=(z[0], z[1]), gamma=(z[2], z[3]), features=features, v_ref=v_ref)

    def obj(z):
        pen = float(np.sum(np.maximum(0, lo - z) ** 2 + np.maximum(0, z - hi) ** 2)) * 1e6
        return _solve_coef(unpack(z), L, R, V, Y, w, scale)[1] + pen

    if starts is None:
        starts = [(a, a, g, g) for a in (0.3 * pmin, 0.6 * pmin, 0.9 * pmin) for g in (0.5, 1.0, 1.5)]
    best = None
    for z0 in starts:
        res = minimize(obj, np.array(z0, float), method="Nelder-Mead",
                       options={"xatol": 1e-3, "fatol": 1e-9, "maxiter": 4000, "maxfev": 8000})
        if best is None or res.fun < best.fun:
            best = res
    m = unpack(best.x)
    m.coef, sse = _solve_coef(m, L, R, V, Y, w, scale)
    resid = Y - m.predict(L, R, V)
    m.meta = {"n": int(len(L)), "sse_scaled": sse, "rmse": np.sqrt(np.mean(resid ** 2, axis=0)).tolist(),
              "scale": scale.tolist()}
    return m


def cell_groups(L, R):
    """Group id per segment: the drive cell (|L|, |R|) (reverse-polarity checks share their cell)."""
    keys = [(int(round(abs(a))), int(round(abs(b)))) for a, b in zip(L, R)]
    uniq = {k: i for i, k in enumerate(sorted(set(keys)))}
    return np.array([uniq[k] for k in keys])


def _start_of(m: PowerLawMap):
    return [(m.p0[0], m.p0[1], m.gamma[0], m.gamma[1])]


def grouped_cv(L, R, Y, V=None, variants=("lin", "int", "cor"), groups=None, scale=DEFAULT_SCALE, k_folds=10,
               seed=0, gp=False):
    """Grouped CV RMSE per output for each variant. Returns {variant: {"rmse": [3], "score": float}}.

    Groups default to drive cells; k_folds=None means leave-one-group-out. Each fold is warm-started from the
    full-data fit (one Nelder-Mead run instead of the multi-start search).
    """
    L, R, Y = np.asarray(L, float), np.asarray(R, float), np.asarray(Y, float)
    g = cell_groups(L, R) if groups is None else np.asarray(groups)
    ug = np.unique(g)
    if k_folds is not None and k_folds < len(ug):
        rng = np.random.default_rng(seed)
        fold_of = {u: i % k_folds for i, u in enumerate(rng.permutation(ug))}
        folds = [np.array([fold_of[x] == f for x in g]) for f in range(k_folds)]
    else:
        folds = [g == u for u in ug]
    scale = np.asarray(scale, float)
    out = {}
    names = list(variants) + (["gp"] if gp else [])
    for name in names:
        pred = np.full_like(Y, np.nan)
        full = None if name == "gp" else fit_power_law(L, R, Y, V=V if name == "bat" else None, features=name,
                                                       scale=scale)
        for test in folds:
            train = ~test
            Vtr = None if V is None else np.asarray(V)[train]
            Vte = None if V is None else np.asarray(V)[test]
            if name == "gp":
                from .gp import GPMap
                model = GPMap.fit(L[train], R[train], Y[train], V=Vtr)
                pred[test] = model.predict(L[test], R[test], V=Vte)
            else:
                model = fit_power_law(L[train], R[train], Y[train], V=Vtr if name == "bat" else None,
                                      features=name, scale=scale, starts=_start_of(full))
                pred[test] = model.predict(L[test], R[test], Vte if name == "bat" else None)
        rmse = np.sqrt(np.nanmean((Y - pred) ** 2, axis=0))
        out[name] = {"rmse": rmse.tolist(), "score": float(np.sqrt(np.mean((rmse / scale) ** 2)))}
    return out


def select_and_fit(L, R, Y, V=None, variants=None, scale=DEFAULT_SCALE, gp=False, k_folds=10):
    """Grouped CV over the parametric variants, then refit the winner on all data."""
    if variants is None:
        variants = ("lin", "int", "cor", "bat") if V is not None else ("lin", "int", "cor")
    cv = grouped_cv(L, R, Y, V=V, variants=variants, scale=scale, gp=gp, k_folds=k_folds)
    param = {k: v for k, v in cv.items() if k != "gp"}
    best = min(param, key=lambda k: param[k]["score"])
    m = fit_power_law(L, R, Y, V=V if best == "bat" else None, features=best, scale=scale)
    m.meta["cv"] = cv
    m.meta["selected"] = best
    if best == "cor":
        m.meta["cor_offset_m"] = m.cor_offset()
    return m


def bootstrap(L, R, Y, V=None, features="lin", n=100, seed=0, scale=DEFAULT_SCALE):
    """Cell-level bootstrap of the parameters, for posterior-predictive draws in domain randomisation."""
    L, R, Y = np.asarray(L, float), np.asarray(R, float), np.asarray(Y, float)
    g = cell_groups(L, R)
    ug = np.unique(g)
    rng = np.random.default_rng(seed)
    full = fit_power_law(L, R, Y, V=V, features=features, scale=scale)
    draws = []
    for _ in range(n):
        pick = rng.choice(ug, size=len(ug), replace=True)
        idx = np.concatenate([np.flatnonzero(g == u) for u in pick])
        m = fit_power_law(L[idx], R[idx], Y[idx], V=None if V is None else np.asarray(V)[idx], features=features,
                          scale=scale, starts=_start_of(full))
        draws.append(m.to_dict())
    return draws
