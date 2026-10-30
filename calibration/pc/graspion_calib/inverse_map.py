"""Inverse map (v, omega) -> (L, R) and the feasible-set polytope, from a fitted PowerLawMap.

The inverse works in (v, omega). For the 'lin' model v = u1, the forward speed of the map's reference point.
For the 'cor' model v = u1 - d |omega|: the translational speed of EllipticalBot2D / the digital twin, to be
combined with the CoR offset d (v_from_u1 / u1_from_v convert).
Lateral u2 is not controllable with two inputs and is reported, not inverted.

For 'lin' and 'cor' the inverse is a closed-form 2x2 solve (MCU-friendly, no LUT)::

    [v, omega] = c + A [f_L, f_R]      ->    f = A^-1 ([v, omega] - c),   p_k = p0_k + f_k ** (1 / gamma_k)

Both motors in [p_min, p_max] gives f in a box, whose image is a parallelogram: the convex operating region
exported as a polytope H z <= h (z = [v, omega]) for MPC constraints and RL action projection. Single-motor
pivots (one motor off) are separate line segments outside that region; they are listed but not part of the
polytope. For the 'int' model the inverse is solved numerically and the polytope is the convex hull of the image
of the box (flagged if that image is not convex).
"""
from __future__ import annotations

import json

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import ConvexHull

from .forward_map import PowerLawMap


P_MIN_DEFAULT = 60.0      # lowest drive of the operating region unless the dead zone reaches it
P0_MARGIN = 5.0           # automatic p_min keeps this far above p0 (f ~ (p - p0)^gamma is steep just above p0)


class InverseMap:
    def __init__(self, fwd: PowerLawMap, p_min=None, p_max=(120.0, 120.0), scale=(0.005, 0.1)):
        """p_min=None: per motor max(60, p0 + 5), so a bot whose dead zone reaches 60 still gets an inverse."""
        self.fwd = fwd
        p0 = np.asarray(fwd.p0, float)
        self.p_min_auto = p_min is None
        self.p_min = np.maximum(P_MIN_DEFAULT, p0 + P0_MARGIN) if p_min is None else np.asarray(p_min, float)
        self.p_max = np.asarray(p_max, float)
        self.scale = np.asarray(scale, float)
        if np.any(self.p_min <= p0):
            raise ValueError(f"p_min {self.p_min.tolist()} must be above the dead zone p0 {p0.tolist()}")
        if np.any(self.p_min >= self.p_max):
            raise ValueError(f"no operating region: p_min {self.p_min.tolist()} >= p_max {self.p_max.tolist()} "
                             f"(dead zone p0 {p0.tolist()})")
        self.f_min = (self.p_min - np.asarray(fwd.p0)) ** np.asarray(fwd.gamma)
        self.f_max = (self.p_max - np.asarray(fwd.p0)) ** np.asarray(fwd.gamma)
        self.linear = fwd.features in ("lin", "cor")
        self.d = fwd.cor_offset()
        if self.linear:
            C = np.asarray(fwd.coef)
            self.c = C[[0, 2], 0]
            self.A = C[[0, 2], 1:3]
            self.A_inv = np.linalg.inv(self.A)
        self.vertices, self.H, self.h, self.convex_image = self._polytope()

    # ------------------------------------------------------------------ forward helpers
    def v_from_u1(self, u1, w):
        return u1 - self.d * np.abs(w)

    def u1_from_v(self, v, w):
        return v + self.d * np.abs(w)

    def twist(self, L, R):
        """(v, omega) for drive pairs (v = u1 for every model except 'cor')."""
        Y = self.fwd.predict(L, R)
        return np.c_[self.v_from_u1(Y[:, 0], Y[:, 2]), Y[:, 2]]

    def _f_to_p(self, f):
        return np.asarray(self.fwd.p0) + np.maximum(f, 0) ** (1.0 / np.asarray(self.fwd.gamma))

    # ------------------------------------------------------------------ feasible set
    def _polytope(self, n=60):
        s = np.linspace(0, 1, n)
        lo, hi = self.p_min, self.p_max
        edges = [np.c_[lo[0] + (hi[0] - lo[0]) * s, np.full(n, lo[1])],
                 np.c_[np.full(n, hi[0]), lo[1] + (hi[1] - lo[1]) * s],
                 np.c_[hi[0] - (hi[0] - lo[0]) * s, np.full(n, hi[1])],
                 np.c_[np.full(n, lo[0]), hi[1] - (hi[1] - lo[1]) * s]]
        loop = np.vstack(edges)
        img = self.twist(loop[:, 0], loop[:, 1])
        hull = ConvexHull(img)
        V = img[hull.vertices]
        Hm = hull.equations[:, :2]
        hv = -hull.equations[:, 2]
        x, y = img[:, 0], img[:, 1]
        area_loop = 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))
        convex = bool(abs(area_loop - hull.volume) <= 1e-3 * hull.volume)
        if self.linear:                       # exact parallelogram
            corners = np.array([[lo[0], lo[1]], [hi[0], lo[1]], [hi[0], hi[1]], [lo[0], hi[1]]])
            V = self.twist(corners[:, 0], corners[:, 1])
            h2 = ConvexHull(V)
            V, Hm, hv = V[h2.vertices], h2.equations[:, :2], -h2.equations[:, 2]
            convex = True
        return V, Hm, hv, convex

    def contains(self, v, w, tol=0.0):
        z = np.array([v, w])
        return bool(np.all(self.H @ z <= self.h + tol))

    def project(self, v, w):
        """Closest feasible (v, omega) in the scaled metric (scale = (m/s, rad/s) noise levels)."""
        if self.contains(v, w):
            return float(v), float(w)
        z = np.array([v, w]) / self.scale
        P = self.vertices / self.scale
        best, bd = None, np.inf
        for i in range(len(P)):
            a, b = P[i], P[(i + 1) % len(P)]
            t = np.clip(np.dot(z - a, b - a) / max(np.dot(b - a, b - a), 1e-18), 0, 1)
            q = a + t * (b - a)
            d = np.sum((z - q) ** 2)
            if d < bd:
                best, bd = q, d
        q = best * self.scale
        return float(q[0]), float(q[1])

    # ------------------------------------------------------------------ inverse
    def solve(self, v, w):
        """(L, R) as floats for a target inside the polytope (both motors on). No projection, no rounding."""
        if self.linear:
            f = self.A_inv @ (np.array([v, w]) - self.c)
            return self._f_to_p(f)
        z = np.array([v, w])
        p0 = 0.5 * (self.p_min + self.p_max)
        r = least_squares(lambda p: (self.twist([p[0]], [p[1]])[0] - z) / self.scale, p0,
                          bounds=(self.p_min, self.p_max))
        return r.x

    def drive_for(self, v, w, project=True):
        """Integer (L, R) for a target; returns (L, R, feasible_before_projection)."""
        ok = self.contains(v, w, tol=1e-9)
        if not ok and project:
            v, w = self.project(v, w)
        p = np.clip(self.solve(v, w), self.p_min, self.p_max)
        return int(round(p[0])), int(round(p[1])), ok

    def pivots(self, n=5):
        """Single-motor pivots (outside the polytope): drive -> predicted (u1, omega)."""
        out = {}
        for name, (Lf, Rf) in {"left_only": (1, 0), "right_only": (0, 1)}.items():
            ps = np.linspace(self.p_min[0 if Lf else 1], self.p_max[0 if Lf else 1], n)
            Y = self.twist(ps * Lf, ps * Rf)
            out[name] = [{"drive": float(p), "v": float(y[0]), "u1": float(self.u1_from_v(y[0], y[1])),
                          "omega": float(y[1])} for p, y in zip(ps, Y)]
        return out

    # ------------------------------------------------------------------ export
    def to_dict(self):
        d = {"forward": self.fwd.to_dict(), "p_min": self.p_min.tolist(), "p_min_auto": self.p_min_auto,
             "p_max": self.p_max.tolist(),
             "f_min": self.f_min.tolist(), "f_max": self.f_max.tolist(),
             "polytope": {"H": self.H.tolist(), "h": self.h.tolist(), "vertices": self.vertices.tolist(),
                          "variables": ["v (m/s)", "omega (rad/s)"], "convex_image": self.convex_image},
             "v_definition": "u1" if self.fwd.features != "cor" else "u1 - cor_offset_m |omega|",
             "cor_offset_m": self.d,
             "pivots": self.pivots()}
        if self.linear:
            d["closed_form"] = {"c": self.c.tolist(), "A": self.A.tolist(), "A_inv": self.A_inv.tolist()}
        return d

    def save(self, path):
        with open(path, "w") as f:
            json.dump(self.to_dict(), f, indent=2)
