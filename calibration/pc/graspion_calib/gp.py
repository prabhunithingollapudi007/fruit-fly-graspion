"""Optional nonparametric comparator: one GP per twist output on (|L|, |R|[, V]), RBF-ARD + noise, numpy only.

Used in grouped CV next to the parametric map (forward_map.grouped_cv(..., gp=True)). It has no notion of the
dead zone, so expect it to do worse near p0 and better where the power law is too rigid.
"""
from __future__ import annotations

import numpy as np
from scipy.linalg import cho_factor, cho_solve
from scipy.optimize import minimize


def _rbf(A, B, ls, sf2):
    d = (A[:, None, :] - B[None, :, :]) / ls
    return sf2 * np.exp(-0.5 * np.sum(d ** 2, axis=-1))


class _GP1:
    def fit(self, X, y, restarts=3, seed=0):
        self.X, self.mu, self.sd = X, y.mean(), y.std() + 1e-12
        z = (y - self.mu) / self.sd
        D = X.shape[1]

        def nll(p):
            ls, sf2, sn2 = np.exp(p[:D]), np.exp(p[D]), np.exp(p[D + 1]) + 1e-8
            K = _rbf(X, X, ls, sf2) + sn2 * np.eye(len(X))
            try:
                c = cho_factor(K, lower=True)
            except np.linalg.LinAlgError:
                return 1e10
            a = cho_solve(c, z)
            return 0.5 * z @ a + np.sum(np.log(np.diag(c[0]))) + 0.5 * len(X) * np.log(2 * np.pi)

        rng = np.random.default_rng(seed)
        best = None
        for r in range(restarts):
            p0 = np.r_[np.log(np.full(D, 0.7) * np.exp(rng.normal(0, 0.3, D))), 0.0, np.log(0.05)]
            res = minimize(nll, p0, method="L-BFGS-B", bounds=[(-4, 3)] * D + [(-4, 3), (-12, 1)])
            if best is None or res.fun < best.fun:
                best = res
        p = best.x
        self.ls, self.sf2, self.sn2 = np.exp(p[:D]), np.exp(p[D]), np.exp(p[D + 1]) + 1e-8
        K = _rbf(X, X, self.ls, self.sf2) + self.sn2 * np.eye(len(X))
        self.c = cho_factor(K, lower=True)
        self.alpha = cho_solve(self.c, z)
        return self

    def predict(self, Xs, return_std=False):
        Ks = _rbf(Xs, self.X, self.ls, self.sf2)
        m = Ks @ self.alpha * self.sd + self.mu
        if not return_std:
            return m
        v = cho_solve(self.c, Ks.T)
        var = self.sf2 - np.sum(Ks * v.T, axis=1)
        return m, np.sqrt(np.maximum(var, 0)) * self.sd


class GPMap:
    def __init__(self, gps, x_mu, x_sd, use_v):
        self.gps, self.x_mu, self.x_sd, self.use_v = gps, x_mu, x_sd, use_v

    @staticmethod
    def _X(L, R, V):
        cols = [np.abs(np.asarray(L, float)), np.abs(np.asarray(R, float))]
        if V is not None:
            cols.append(np.asarray(V, float))
        return np.stack(cols, axis=-1)

    @classmethod
    def fit(cls, L, R, Y, V=None):
        X = cls._X(L, R, V)
        mu, sd = X.mean(0), X.std(0) + 1e-9
        Xn = (X - mu) / sd
        gps = [_GP1().fit(Xn, np.asarray(Y)[:, k]) for k in range(3)]
        return cls(gps, mu, sd, V is not None)

    def predict(self, L, R, V=None, return_std=False):
        X = (self._X(L, R, V if self.use_v else None) - self.x_mu) / self.x_sd
        if not return_std:
            return np.stack([g.predict(X) for g in self.gps], axis=-1)
        ms = [g.predict(X, True) for g in self.gps]
        return np.stack([m for m, _ in ms], -1), np.stack([s for _, s in ms], -1)
