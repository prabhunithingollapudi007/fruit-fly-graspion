"""Forward-map models with one interface, for any robot described by a designs.Spec:

    m = make("gp", spec).fit(X, Y)      X (n, d) commands in spec units, Y (n, n_out) measured outputs
    m.predict(X) -> (n, n_out);  m.predict_std(X) -> (mean, sd) where the model has an uncertainty

Classical
    deadzone      per input f = max(0, |x - off| - p0) ** gamma; outputs linear in [1, f]; zero when every input is
                  in its dead zone (graspion_calib.forward_map 'lin', any number of inputs); deadzone_int adds f_i f_j
    poly1..3      polynomial response surface (ridge); poly1 with a strong ridge is the affine "standardisation" map
    lut           piecewise-linear interpolation of the per-point means (what a firmware lookup table does)
    tps           thin-plate spline, smoothing chosen by grouped 5-fold CV
    gp            one GP per output, RBF-ARD + noise (graspion_calib.gp)
Neural
    mlp           ensemble of 5 tanh MLPs (32, 32), weight decay, full-batch L-BFGS; SD = ensemble spread
Grey box
    dz+gp, dz+mlp the dead-zone model plus a GP / small MLP on its residuals
Transfer to a new unit: Residual(spec, fleet_model, inner, frozen=True) keeps a model fitted on other robots and fits
only `inner` (poly1, gp, ...) to the few points measured on the new one.

Inputs are scaled to [-1, 1] by the spec's design box, outputs to noise units by spec.scale.
"""
from __future__ import annotations

import itertools
import os
import pickle
import sys

import numpy as np
from scipy.interpolate import LinearNDInterpolator, NearestNDInterpolator, RBFInterpolator
from scipy.optimize import minimize

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "calibration", "pc"))
from graspion_calib.gp import _GP1  # noqa: E402


class Model:
    def __init__(self, spec):
        self.spec = spec

    def _u(self, X):
        s = self.spec
        return 2 * (np.asarray(X, float) - s.lo) / (s.hi - s.lo) - 1

    def save(self, path):
        with open(path, "wb") as f:
            pickle.dump(self, f)


def load(path):
    with open(path, "rb") as f:
        return pickle.load(f)


def unique_means(X, Y):
    """Distinct rows of X, the mean of Y over each, and the group index of every row."""
    U, g = np.unique(np.asarray(X, float), axis=0, return_inverse=True)
    g = g.ravel()
    M = np.stack([np.asarray(Y)[g == i].mean(0) for i in range(len(U))])
    return U, M, g


# ---------------------------------------------------------------- classical

class DeadZone(Model):
    def __init__(self, spec, features="lin"):
        super().__init__(spec)
        self.features = features
        d = len(spec.lo)
        self.p0, self.gamma = np.zeros(d), np.ones(d)
        self.coef = None

    def act(self, X, p0=None, gamma=None):
        p0 = self.p0 if p0 is None else p0
        gamma = self.gamma if gamma is None else gamma
        return np.maximum(0.0, np.abs(np.asarray(X, float) - self.spec.off) - p0) ** gamma

    def _design(self, X, p0, gamma):
        F = self.act(X, p0, gamma)
        cols = [np.ones((len(F), 1)), F]
        if self.features == "int":
            cols += [F[:, [i]] * F[:, [j]] for i, j in itertools.combinations(range(F.shape[1]), 2)]
        return np.hstack(cols), np.any(F > 0, axis=1)

    def _solve(self, p0, gamma, X, Z):
        A, on = self._design(X, p0, gamma)
        coef = np.zeros((A.shape[1], Z.shape[1]))
        if on.sum() > A.shape[1]:
            coef = np.linalg.lstsq(A[on], Z[on], rcond=None)[0]
        P = A @ coef
        P[~on] = 0.0
        return coef, float(np.sum((Z - P) ** 2))

    def fit(self, X, Y, starts=None):
        X, Z = np.asarray(X, float), np.asarray(Y, float) / self.spec.scale
        d = X.shape[1]
        span = float(np.max(np.abs(X - self.spec.off)))
        lo = np.r_[np.zeros(d), np.full(d, 0.2)]
        hi = np.r_[np.full(d, 0.95 * span), np.full(d, 2.5)]

        def obj(z):
            zc = np.clip(z, lo, hi)
            return self._solve(zc[:d], zc[d:], X, Z)[1] + 1e6 * float(np.sum((z - zc) ** 2))

        if starts is None:                                    # the spec's prior, then fractions of the range
            starts = [np.r_[np.minimum(self.spec.p0, 0.9 * span), self.spec.gamma]] + [
                np.r_[np.full(d, f * span), np.full(d, g)] for f in (0.15, 0.35) for g in (0.5, 1.0, 1.5)]
        best = min((minimize(obj, s, method="Nelder-Mead",
                             options={"xatol": 1e-3, "fatol": 1e-9, "maxfev": 1500 * d})
                    for s in starts), key=lambda r: r.fun)
        z = np.clip(best.x, lo, hi)
        self.p0, self.gamma = z[:d], z[d:]
        self.coef = self._solve(self.p0, self.gamma, X, Z)[0]
        return self

    def predict(self, X):
        A, on = self._design(np.atleast_2d(X), self.p0, self.gamma)
        P = A @ self.coef
        P[~on] = 0.0
        return P * self.spec.scale

    def theta(self):
        return np.r_[self.p0, self.gamma, self.coef.ravel()]

    def set_theta(self, th):
        d = len(self.p0)
        self.p0, self.gamma = np.asarray(th[:d], float), np.asarray(th[d:2 * d], float)
        self.coef = np.asarray(th[2 * d:], float).reshape(-1, len(self.spec.outputs))
        return self

    def jacobian(self, X):
        """(n, n_out, n_par) derivative of the prediction in noise units w.r.t. theta (central differences)."""
        th = self.theta()
        d = len(self.p0)
        h = np.r_[np.full(d, 0.005 * float(np.max(self.spec.hi - self.spec.lo))), np.full(d, 0.01),
                  1e-3 * np.maximum(np.abs(th[2 * d:]), 1e-3)]
        J = np.zeros((len(X), len(self.spec.outputs), len(th)))
        for j in range(len(th)):
            tp, tm = th.copy(), th.copy()
            tp[j] += h[j]
            tm[j] -= h[j]
            J[:, :, j] = (self.set_theta(tp).predict(X) - self.set_theta(tm).predict(X)) / self.spec.scale / (2 * h[j])
        self.set_theta(th)
        return J


class Poly(Model):
    def __init__(self, spec, deg=2, ridge=1e-6):
        super().__init__(spec)
        self.deg, self.ridge = deg, ridge

    def _A(self, X):
        U = self._u(np.atleast_2d(X))
        cols = [np.ones(len(U))]
        for k in range(1, self.deg + 1):
            cols += [np.prod(U[:, list(c)], axis=1) for c in itertools.combinations_with_replacement(range(U.shape[1]), k)]
        return np.stack(cols, axis=1)

    def fit(self, X, Y):
        A, Z = self._A(X), np.asarray(Y, float) / self.spec.scale
        P = self.ridge * np.eye(A.shape[1])
        P[0, 0] = 0.0                                         # intercept not shrunk
        self.coef = np.linalg.solve(A.T @ A + P, A.T @ Z)
        return self

    def predict(self, X):
        return self._A(X) @ self.coef * self.spec.scale


class LUT(Model):
    def fit(self, X, Y):
        U, M, _ = unique_means(self._u(X), np.asarray(Y, float))
        self.U, self.M = U, M
        if U.shape[1] > 1 and len(U) > U.shape[1]:
            self.lin = LinearNDInterpolator(U, M)
        self.near = NearestNDInterpolator(U, M) if U.shape[1] > 1 else None
        return self

    def predict(self, X):
        U = self._u(np.atleast_2d(X))
        if U.shape[1] == 1:
            o = np.argsort(self.U[:, 0])
            return np.stack([np.interp(U[:, 0], self.U[o, 0], self.M[o, k]) for k in range(self.M.shape[1])], 1)
        P = self.lin(U) if hasattr(self, "lin") else np.full((len(U), self.M.shape[1]), np.nan)
        bad = np.any(np.isnan(P), axis=1)
        if bad.any():
            P[bad] = self.near(U[bad])
        return P


class TPS(Model):
    def __init__(self, spec, smoothings=(0.01, 0.1, 1.0, 10.0, 100.0)):
        super().__init__(spec)
        self.smoothings = smoothings

    def _make(self, U, Z, s):
        return RBFInterpolator(U, Z, kernel="thin_plate_spline", smoothing=s, degree=1)

    def fit(self, X, Y):
        U, Z = self._u(X), np.asarray(Y, float) / self.spec.scale
        _, _, g = unique_means(U, Z)
        ug = np.unique(g)
        best, best_e = self.smoothings[-1], np.inf
        if len(ug) >= 10:
            fold_of = dict(zip(np.random.default_rng(0).permutation(ug), np.arange(len(ug)) % 5))
            f = np.array([fold_of[x] for x in g])
            for s in self.smoothings:
                e = 0.0
                for k in range(5):
                    tr = f != k
                    try:
                        e += float(np.sum((self._make(U[tr], Z[tr], s)(U[~tr]) - Z[~tr]) ** 2))
                    except (np.linalg.LinAlgError, ValueError):
                        e = np.inf
                        break
                if e < best_e:
                    best, best_e = s, e
        self.s = best
        self.f = self._make(U, Z, best)
        return self

    def predict(self, X):
        return self.f(self._u(np.atleast_2d(X))) * self.spec.scale


class GP(Model):
    def fit(self, X, Y):
        U, Z = self._u(X), np.asarray(Y, float) / self.spec.scale
        self.gps = [_GP1().fit(U, Z[:, k]) for k in range(Z.shape[1])]
        return self

    def predict(self, X):
        U = self._u(np.atleast_2d(X))
        return np.stack([g.predict(U) for g in self.gps], 1) * self.spec.scale

    def predict_std(self, X):
        U = self._u(np.atleast_2d(X))
        ms = [g.predict(U, True) for g in self.gps]
        return (np.stack([m for m, _ in ms], 1) * self.spec.scale, np.stack([s for _, s in ms], 1) * self.spec.scale)


# ---------------------------------------------------------------- neural

class MLP(Model):
    def __init__(self, spec, hidden=(32, 32), members=5, wd=1e-3, maxiter=2000):
        super().__init__(spec)
        self.hidden, self.members, self.wd, self.maxiter = hidden, members, wd, maxiter

    def _shapes(self, d, o):
        sizes = [d, *self.hidden, o]
        return [(sizes[i], sizes[i + 1]) for i in range(len(sizes) - 1)]

    def _unpack(self, w, shapes):
        out, i = [], 0
        for a, b in shapes:
            out.append((w[i:i + a * b].reshape(a, b), w[i + a * b:i + a * b + b]))
            i += a * b + b
        return out

    def _forward(self, params, U):
        hs = [U]
        for W, b in params[:-1]:
            hs.append(np.tanh(hs[-1] @ W + b))
        W, b = params[-1]
        return hs, hs[-1] @ W + b

    def _loss(self, w, shapes, U, T):
        params = self._unpack(w, shapes)
        hs, out = self._forward(params, U)
        r = out - T
        n = len(U)
        loss = 0.5 * np.sum(r ** 2) / n + 0.5 * self.wd * sum(np.sum(W ** 2) for W, _ in params)
        grads, delta = [], r / n
        for i in range(len(params) - 1, -1, -1):
            W, _ = params[i]
            grads.append((hs[i].T @ delta + self.wd * W, delta.sum(0)))
            if i:
                delta = (delta @ W.T) * (1 - hs[i] ** 2)
        g = np.concatenate([np.r_[gW.ravel(), gb] for gW, gb in reversed(grads)])
        return loss, g

    def fit(self, X, Y, seed=0):
        U, Z = self._u(X), np.asarray(Y, float) / self.spec.scale
        self.mu, self.sd = Z.mean(0), Z.std(0) + 1e-9
        T = (Z - self.mu) / self.sd
        shapes = self._shapes(U.shape[1], T.shape[1])
        rng = np.random.default_rng(seed)
        self.nets = []
        for _ in range(self.members):
            w0 = np.concatenate([np.r_[rng.normal(0, np.sqrt(2 / (a + b)), a * b), np.zeros(b)] for a, b in shapes])
            res = minimize(self._loss, w0, args=(shapes, U, T), jac=True, method="L-BFGS-B",
                           options={"maxiter": self.maxiter})
            self.nets.append(self._unpack(res.x, shapes))
        return self

    def _all(self, X):
        U = self._u(np.atleast_2d(X))
        return np.stack([self._forward(p, U)[1] for p in self.nets]) * self.sd + self.mu

    def predict(self, X):
        return self._all(X).mean(0) * self.spec.scale

    def predict_std(self, X):
        P = self._all(X)
        return P.mean(0) * self.spec.scale, P.std(0) * self.spec.scale


# ---------------------------------------------------------------- grey box and transfer

class Residual(Model):
    """base + inner, inner fitted on what base leaves. frozen=True keeps an already fitted base (transfer)."""

    def __init__(self, spec, base, inner, frozen=False):
        super().__init__(spec)
        self.base, self.inner, self.frozen = base, inner, frozen

    def fit(self, X, Y):
        if not self.frozen:
            self.base.fit(X, Y)
        self.inner.fit(X, np.asarray(Y, float) - self.base.predict(X))
        return self

    def predict(self, X):
        return self.base.predict(X) + self.inner.predict(X)

    def predict_std(self, X):
        m, s = self.inner.predict_std(X)
        return self.base.predict(X) + m, s


class Zero(Model):
    """Predicts 0: as the inner model of a frozen Residual it uses the fleet model as is."""
    def fit(self, X, Y):
        self.n_out = np.asarray(Y).shape[1]
        return self

    def predict(self, X):
        return np.zeros((len(np.atleast_2d(X)), self.n_out))


ZOO = {
    "zero": Zero,
    "affine": lambda s: Poly(s, 1, ridge=1.0),
    "deadzone": lambda s: DeadZone(s, "lin"),
    "deadzone_int": lambda s: DeadZone(s, "int"),
    "poly2": lambda s: Poly(s, 2),
    "poly3": lambda s: Poly(s, 3),
    "lut": LUT,
    "tps": TPS,
    "gp": GP,
    "mlp": MLP,
    "dz+gp": lambda s: Residual(s, DeadZone(s), GP(s)),
    "dz+mlp": lambda s: Residual(s, DeadZone(s), MLP(s, hidden=(16, 16), wd=1e-2)),
}


def make(name, spec):
    return ZOO[name](spec)


def grouped_cv(spec, X, Y, names, k_folds=10, seed=0):
    """Grouped CV (a group = every replicate of one command point). {name: rmse per output}."""
    X, Y = np.asarray(X, float), np.asarray(Y, float)
    _, _, g = unique_means(X, Y)
    ug = np.unique(g)
    k = min(k_folds, len(ug))
    fold_of = dict(zip(np.random.default_rng(seed).permutation(ug), np.arange(len(ug)) % k))
    f = np.array([fold_of[x] for x in g])
    out = {}
    for name in names:
        P = np.zeros_like(Y)
        for i in range(k):
            te = f == i
            P[te] = make(name, spec).fit(X[~te], Y[~te]).predict(X[te])
        out[name] = np.sqrt(np.mean((P - Y) ** 2, axis=0))
    return out
