"""Command designs: which drive values to calibrate at, for a robot with d inputs described by a Spec.

Product grids (one level set per input, every combination):
    uniform   equal steps over [lo, hi] (the usual "0..120 in steps of 10")
    cheb      Chebyshev-Lobatto nodes: denser at both ends of the range
    warped    the off level, then equal steps in the expected response f = (x - p0)^gamma of the spec's prior
              dead-zone model, i.e. denser just above the threshold where the response is steepest (gamma < 1)
Scattered designs (n distinct points, no replicates):
    halton    scrambled Halton low-discrepancy sequence
    lhs       Latin hypercube, centred-discrepancy optimised
    dopt      greedy Bayesian D-optimal points for the dead-zone model at the spec's prior, from an integer lattice
    active    sequential: half halton, then batches at the largest GP predictive SD (needs the robot: an oracle)
Every design is snapped to the input resolution (spec.step, 1 = integer drive values).
"""
from __future__ import annotations

import json
from dataclasses import dataclass, field

import numpy as np
from scipy.stats import qmc

PRODUCT = ("uniform", "cheb", "warped")
SCATTERED = ("halton", "lhs", "dopt", "active")


@dataclass
class Spec:
    """What a robot takes and returns. lo / hi bound the design; operate is the box the map is used (and scored)
    in; off is each input's motor-off value; prior is a rough dead-zone guess {p0, gamma} per input, measured from
    off, used by 'warped' and 'dopt'; scale is the output noise scale that makes outputs comparable."""
    inputs: list
    outputs: list
    lo: list
    hi: list
    scale: list
    off: list = None
    operate: list = None            # [[lo...], [hi...]]
    step: float = 1.0
    prior: dict = field(default_factory=dict)
    name: str = "robot"

    def __post_init__(self):
        d = len(self.inputs)
        self.lo, self.hi = np.asarray(self.lo, float), np.asarray(self.hi, float)
        self.off = np.asarray(self.lo if self.off is None else self.off, float)
        self.scale = np.asarray(self.scale, float)
        op = self.operate or [self.lo, self.hi]
        self.operate = (np.asarray(op[0], float), np.asarray(op[1], float))
        self.p0 = np.asarray(self.prior.get("p0", [0.0] * d), float)
        self.gamma = np.asarray(self.prior.get("gamma", [1.0] * d), float)

    @classmethod
    def load(cls, path):
        with open(path) as f:
            return cls(**json.load(f))

    def snap(self, X):
        X = np.round(np.asarray(X, float) / self.step) * self.step
        return np.clip(X, self.lo, self.hi)


def levels(kind, k, lo, hi, off=None, p0=0.0, gamma=1.0):
    """k levels on [lo, hi] for one input."""
    if kind == "uniform":
        return np.linspace(lo, hi, k)
    if kind == "cheb":
        return lo + (hi - lo) * (1 - np.cos(np.pi * np.arange(k) / (k - 1))) / 2
    if kind == "warped":
        off = lo if off is None else off
        below = lo < off + p0                                    # the box reaches into the dead zone: keep lo
        start = max(lo, off + p0 + 0.05 * (hi - off - p0))      # first level just above the threshold
        f0, f1 = (start - off - p0) ** gamma, (hi - off - p0) ** gamma
        above = off + p0 + np.linspace(f0, f1, k - 1 if below else k) ** (1 / gamma)
        return np.r_[lo, above] if below else above
    raise ValueError(kind)


def product(spec: Spec, kind, k):
    lv = [np.unique(np.round(levels(kind, k, spec.lo[j], spec.hi[j], spec.off[j], spec.p0[j], spec.gamma[j])
                             / spec.step) * spec.step) for j in range(len(spec.lo))]
    return np.array(np.meshgrid(*lv, indexing="ij")).reshape(len(lv), -1).T


def _unit_to_box(U, spec):
    return spec.snap(spec.lo + U * (spec.hi - spec.lo))


def halton(spec: Spec, n, seed=0):
    return _unit_to_box(qmc.Halton(len(spec.lo), scramble=True, seed=seed).random(n), spec)


def lhs(spec: Spec, n, seed=0):
    return _unit_to_box(qmc.LatinHypercube(len(spec.lo), optimization="random-cd", seed=seed).random(n), spec)


def lattice(spec: Spec, step=None):
    """Every snapped point at `step` spacing (candidate set for dopt)."""
    step = step or max(spec.step, float(np.min(spec.hi - spec.lo)) / 40)
    axes = [np.arange(spec.lo[j], spec.hi[j] + 1e-9, step) for j in range(len(spec.lo))]
    return spec.snap(np.array(np.meshgrid(*axes, indexing="ij")).reshape(len(axes), -1).T)


def dopt(spec: Spec, n, seed=0, theta=None, cand=None, repeats=True):
    """Greedy Bayesian D-optimal design for the dead-zone model (models.DeadZone) at its prior.

    theta = the model's (p0, gamma, head) at which to linearise; default: spec.prior p0 / gamma with a unit head
    per output on every activation (shape only, so the design does not depend on the unknown gains).
    cand = candidate points (default: lattice(spec)); repeats=False picks each candidate at most once."""
    from models import DeadZone
    cand = lattice(spec) if cand is None else np.asarray(cand, float)
    m = DeadZone(spec)
    if theta is None:
        d = len(spec.lo)
        m.p0, m.gamma = spec.p0.copy(), spec.gamma.copy()
        m.coef = np.vstack([np.zeros((1, len(spec.outputs))), np.ones((d, len(spec.outputs)))])
    else:
        m.set_theta(theta)
    J = m.jacobian(cand)                                      # (n_cand, n_out, n_par), outputs in noise units
    info = np.einsum("coi,coj->cij", J, J)
    Lam = 1e-3 * np.eye(J.shape[2])
    rng = np.random.default_rng(seed)
    order = rng.permutation(len(cand))                        # random tie-breaking
    chosen = []
    for _ in range(min(n, len(cand)) if not repeats else n):
        v = np.array([np.linalg.slogdet(Lam + info[c])[1] if (repeats or c not in chosen) else -np.inf
                      for c in order])
        c = order[int(np.argmax(v))]
        Lam = Lam + info[c]
        chosen.append(c)
    return cand[chosen]


def next_batch(spec: Spec, X, Y, n, cand=None):
    """The n candidates with the largest GP predictive SD (summed over outputs in noise units) given the points
    measured so far, no two within 5 % of the range of each other. Use between rounds on the robot."""
    from models import GP
    cand = lattice(spec) if cand is None else cand
    sd = GP(spec).fit(X, Y).predict_std(cand)[1] / spec.scale
    pick = []
    for i in np.argsort(-np.sum(sd ** 2, axis=1)):
        if all(np.max(np.abs(cand[i] - cand[j]) / (spec.hi - spec.lo)) > 0.05 for j in pick):
            pick.append(i)
        if len(pick) == n:
            break
    return cand[pick]


def active(spec: Spec, n, oracle, seed=0, batch=4):
    """Uncertainty sampling: n // 2 halton points, then next_batch rounds. oracle(X) -> measured Y. Returns (X, Y)."""
    X = halton(spec, max(n // 2, len(spec.lo) + 2), seed)
    Y = oracle(X)
    cand = lattice(spec)
    while len(X) < n:
        Xn = next_batch(spec, X, Y, min(batch, n - len(X)), cand)
        X, Y = np.vstack([X, Xn]), np.vstack([Y, oracle(Xn)])
    return X, Y


def make(spec: Spec, kind, n, seed=0, oracle=None):
    """A design of about n points. Product grids use k = round(n ** (1/d)) levels per input and repeat the grid
    to reach n (replicates); scattered designs return n distinct-ish points. Returns X, or (X, Y) for 'active'."""
    if kind in PRODUCT:
        k = max(2, int(round(n ** (1 / len(spec.lo)))))
        G = product(spec, kind, k)
        reps = max(1, int(round(n / len(G))))
        return np.vstack([G] * reps)
    if kind == "halton":
        return halton(spec, n, seed)
    if kind == "lhs":
        return lhs(spec, n, seed)
    if kind == "dopt":
        return dopt(spec, n, seed)
    if kind == "active":
        return active(spec, n, oracle, seed)
    raise ValueError(kind)
