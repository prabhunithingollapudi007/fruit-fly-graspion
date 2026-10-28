"""Fit the calibration forward-map models to the lab's 7x7 (L, R) -> (v, omega) tables and compare with the lab's fits.

    python lab_fit/fit_lab_tables.py --out lab_map.json
    python lab_fit/fit_lab_tables.py --power P.npy --v V.npy --omega W.npy --rows r-desc   # other copies

Inputs are the lab's tables (FEASIBILITY.md 5.1): power (7,), v in m/s and omega in rad/s, both (7, 7), one value
per cell. The defaults are the copies in lab_fit/data/ (rows r-asc), received from the lab: byte for byte the
np.flipud of prajolshrestha/mpc_and_imitation_learning_deployment_src @ 1c2d6a0 imitation_learning_mcu_deployment_src/
task/circle/acados_simple/fitting/{velocity_data,omega_data}/*.npy; the omega table also equals brainbot-digital-twin
generate_LUT/omega_table_latest.npy flipped, whose vCOM table is 0.01-0.10 cm/s lower. --rows says how a table is
laid out, as the copies differ:
    r-desc  rows = right power 120 -> 60, columns = left power 60 -> 120 (the lab repos' npy files)
    r-asc   rows = right power 60 -> 120, columns = left power 60 -> 120 (the same tables with rows flipped)
The script checks the layout: under HARDWARE.md 4.4 omega must fall with L and rise with R (signs of a plane fit),
and it warns otherwise. Left/right itself is INFERRED (HARDWARE.md 4.4).

Compared, all by leave-one-cell-out CV (every cell is one value, so a cell is its own group):
    ours  graspion_calib.forward_map 'lin', 'int', 'cor' (dead-zone power law, shared p0 / gamma), optional --gp:
          once as `analyze` runs them (p0 free in 0..100), and once with p0 below the lowest grid drive (every
          cell moved), plus 'lin' fitted to v only and to omega only (is the shared p0 / gamma the cost?)
    lab   polynomials in raw power: plane, quadratic, cubic (v); plane, quadratic (omega)
The selected map is the best bounded variant.
plus the in-sample error of the lab's hard-coded forward model (brainbot-digital-twin generate_LUT/
gen_inverse_map.py) against a full-precision refit of the same polynomials, the rank test behind "only the mean
CoR offset is identifiable" (FEASIBILITY.md 4), a cell bootstrap of the 'cor' offset d, and the operating region of
the selected map (inverse_map.InverseMap) against the lab's LUT box.

Caveat: the lab's v is a centre-of-mass speed magnitude, fitted here as u1 with u2 = 0, so the 'cor' offset d is
the offset of that point, not of a marker. The --out JSON holds the selected map under "map"
(forward_map.PowerLawMap.from_dict), usable as a sim truth map (sim.SimParams.truth).
"""
import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "calibration", "pc"))
from graspion_calib import forward_map as fm  # noqa: E402
from graspion_calib.inverse_map import InverseMap  # noqa: E402

LAB_LUT_BOX = ((0.048, 0.115), (-3.46, 1.43))   # VL_LUT.h: VMIN .. VMIN + 67 DEL_V; OMEGA_MIN .. + 489 DEL_OMEGA


def lab_forward(VL, VR):
    """The lab's hard-coded forward model (gen_inverse_map.py), coefficients as printed there."""
    v = (-0.26528615 + 0.00364152 * VL + 0.00640563 * VR - 0.00002286 * VL * VL - 0.00005067 * VR * VR
         - 0.00002508 * VL * VR + 0.00000010 * VL ** 3 + 0.00000013 * VR ** 3 + 0.00000017 * VL * VR * VR
         - 0.00000005 * VL * VL * VR)
    w = (-5.088834 + 0.133761 * VR - 0.034432 * VL - 5.73471732e-04 * VR * VR - 6.95066445e-05 * VL * VL
         + 1.00638876e-04 * VR * VL)
    return v, w


def poly(L, R, deg):
    cols = [np.ones_like(L), L, R]
    if deg >= 2:
        cols += [L * L, R * R, L * R]
    if deg >= 3:
        cols += [L ** 3, R ** 3, R * R * L, L * L * R]
    return np.stack(cols, axis=1)


def loo_poly(L, R, z, deg):
    A = poly(L, R, deg)
    e = []
    for i in range(len(z)):
        m = np.arange(len(z)) != i
        c, *_ = np.linalg.lstsq(A[m], z[m], rcond=None)
        e.append(z[i] - A[i] @ c)
    return float(np.sqrt(np.mean(np.square(e))))


def insample_poly(L, R, z, deg):
    A = poly(L, R, deg)
    c, *_ = np.linalg.lstsq(A, z, rcond=None)
    return float(np.sqrt(np.mean((z - A @ c) ** 2)))


def loo_power(L, R, Y, features, scale=fm.DEFAULT_SCALE, p0_max=100.0):
    """Leave-one-cell-out RMSE (u1, u2, omega) of fm.fit_power_law with p0 <= p0_max, warm-started per fold."""
    full = fm.fit_power_law(L, R, Y, features=features, scale=scale, p0_bounds=(0.0, p0_max))
    start = [(full.p0[0], full.p0[1], full.gamma[0], full.gamma[1])]
    pred = np.zeros_like(Y)
    for i in range(len(L)):
        m = np.arange(len(L)) != i
        f = fm.fit_power_law(L[m], R[m], Y[m], features=features, scale=scale, p0_bounds=(0.0, p0_max), starts=start)
        pred[i] = f.predict(L[i:i + 1], R[i:i + 1])[0]
    return np.sqrt(np.mean((Y - pred) ** 2, axis=0)).tolist(), full


def boot_offset(L, R, Y, full, p0_max, n, seed=0):
    """Cell bootstrap of the 'cor' offset d with the same p0 bound as `full` (fm.bootstrap fits p0 unbounded)."""
    rng = np.random.default_rng(seed)
    start = [(full.p0[0], full.p0[1], full.gamma[0], full.gamma[1])]
    out = []
    for _ in range(n):
        idx = rng.integers(0, len(L), len(L))            # one value per cell, so a cell is one row
        f = fm.fit_power_law(L[idx], R[idx], Y[idx], features="cor", p0_bounds=(0.0, p0_max), starts=start)
        out.append(f.cor_offset())
    return out


def load_tables(a):
    p = np.load(a.power, allow_pickle=False).astype(float)
    v = np.load(a.v, allow_pickle=False).astype(float)
    w = np.load(a.omega, allow_pickle=False).astype(float)
    n = len(p)
    if v.shape != (n, n) or w.shape != (n, n):
        sys.exit(f"tables must be ({n}, {n}); got v {v.shape}, omega {w.shape}")
    i, j = np.meshgrid(np.arange(n), np.arange(n), indexing="ij")
    L = p[j]
    R = p[n - 1 - i] if a.rows == "r-desc" else p[i]
    return L.ravel(), R.ravel(), v.ravel(), w.ravel()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    data = os.path.join(os.path.dirname(os.path.abspath(__file__)), "data")
    ap.add_argument("--power", default=os.path.join(data, "power.npy"))
    ap.add_argument("--v", default=os.path.join(data, "velocity_table.npy"), help="CoM speed table, m/s")
    ap.add_argument("--omega", default=os.path.join(data, "omega_table.npy"), help="omega table, rad/s")
    ap.add_argument("--rows", default="r-asc", choices=("r-desc", "r-asc"),
                    help="table layout (default r-asc, the layout of lab_fit/data)")
    ap.add_argument("--gp", action="store_true", help="add the GP comparator to the CV")
    ap.add_argument("--p0-margin", type=float, default=5.0,
                    help="bounded fits keep p0 this far below the lowest grid drive (default 5)")
    ap.add_argument("--boot", type=int, default=200, help="cell-bootstrap draws for the 'cor' offset (0 = off)")
    ap.add_argument("--out", help="JSON with the selected map and all scores")
    a = ap.parse_args()

    L, R, v, w = load_tables(a)
    at = lambda l, r: w[(L == l) & (R == r)][0]
    sl = np.linalg.lstsq(poly(L, R, 1), w, rcond=None)[0]
    print(f"orientation: omega(L120, R60) = {at(120, 60):+.2f} rad/s, omega(L60, R120) = {at(60, 120):+.2f} rad/s; "
          f"plane slopes d omega/dL = {sl[1]:+.4f}, d omega/dR = {sl[2]:+.4f} rad/s per unit"
          + ("" if sl[1] < 0 < sl[2] else "\n  WARNING: more left drive should turn CW (slope < 0) and more right "
             "drive CCW (> 0), HARDWARE.md 4.4: check --rows"))
    print(f"data: {len(L)} cells, v {v.min() * 100:.2f}..{v.max() * 100:.2f} cm/s, omega {w.min():+.2f}..{w.max():+.2f} rad/s")

    Y = np.c_[v, np.zeros_like(v), w]
    cv = fm.grouped_cv(L, R, Y, variants=("lin", "int", "cor"), k_folds=None, gp=a.gp)
    lab = {"v": {f"poly{d}": loo_poly(L, R, v, d) for d in (1, 2, 3)},
           "omega": {f"poly{d}": loo_poly(L, R, w, d) for d in (1, 2)}}

    # p0 below the lowest grid drive: every cell moved, so neither motor can be in its dead zone there.
    p0_max = float(min(L.min(), R.min())) - a.p0_margin
    bounded, fits = {}, {}
    for k in ("lin", "int", "cor"):
        bounded[k], fits[k] = loo_power(L, R, Y, k, p0_max=p0_max)
    sv_ = (fm.DEFAULT_SCALE[0], fm.DEFAULT_SCALE[1], 1e6)       # omega residuals ignored
    sw_ = (1e6, 1e6, fm.DEFAULT_SCALE[2])                       # u1, u2 residuals ignored
    bounded["lin v-only"] = loo_power(L, R, Y, "lin", scale=sv_, p0_max=p0_max)[0]
    bounded["lin omega-only"] = loo_power(L, R, Y, "lin", scale=sw_, p0_max=p0_max)[0]

    print("\nleave-one-cell-out RMSE        v [cm/s]   omega [rad/s]")
    for k, r in cv.items():
        print(f"  ours {k:4s} (as in analyze)     {r['rmse'][0] * 100:6.3f}     {r['rmse'][2]:6.3f}")
    for k, r in bounded.items():
        sel = (f"{r[0] * 100:6.3f}" if "omega-only" not in k else "     -") + "     " + \
              (f"{r[2]:6.3f}" if "v-only" not in k else "     -")
        print(f"  ours {k:14s} p0<={p0_max:.0f} {sel}")
    names = {1: "plane", 2: "quadratic", 3: "cubic"}
    for d in (1, 2, 3):
        wcol = f"{lab['omega'][f'poly{d}']:6.3f}" if d < 3 else "     -"
        print(f"  lab polynomial, {names[d]:9s}      {lab['v'][f'poly{d}'] * 100:6.3f}     {wcol}")

    lv, lw = lab_forward(L, R)
    pub = {"v": float(np.sqrt(np.mean((v - lv) ** 2))), "omega": float(np.sqrt(np.mean((w - lw) ** 2)))}
    ref = {"v": insample_poly(L, R, v, 3), "omega": insample_poly(L, R, w, 2)}
    print(f"\nlab's hard-coded model, in-sample: v {pub['v'] * 100:.3f} cm/s (full-precision cubic refit "
          f"{ref['v'] * 100:.3f}), omega {pub['omega']:.3f} rad/s (quadratic refit {ref['omega']:.3f})")

    for k, r in cv.items():
        if k != "gp":
            f = fm.fit_power_law(L, R, Y, features=k)
            print(f"  unbounded '{k}': p0 = ({f.p0[0]:.1f}, {f.p0[1]:.1f}), gamma = ({f.gamma[0]:.2f}, {f.gamma[1]:.2f})")
    score = lambda r: float(np.sqrt(np.mean((np.asarray(r) / np.asarray(fm.DEFAULT_SCALE)) ** 2)))
    best = min(fits, key=lambda k: score(bounded[k]))
    m = fits[best]
    lin = fits["lin"]
    print(f"\nselected '{best}' (p0 <= {p0_max:.0f}): p0 = ({m.p0[0]:.1f}, {m.p0[1]:.1f}), "
          f"gamma = ({m.gamma[0]:.2f}, {m.gamma[1]:.2f})")
    at_bound = [f"{n} = {x:.2f}" for n, x, lo_, hi_ in
                [("p0_L", m.p0[0], 0, p0_max), ("p0_R", m.p0[1], 0, p0_max),
                 ("gamma_L", m.gamma[0], 0.2, 2.5), ("gamma_R", m.gamma[1], 0.2, 2.5)]
                if min(abs(x - lo_), abs(x - hi_)) < 1e-3]
    if at_bound:
        print(f"  WARNING: {', '.join(at_bound)} at a bound: dead zone and exponent are not identified by a grid that "
              f"starts at {min(L.min(), R.min()):.0f}; use the map only inside the grid")

    fL, fR = lin.activations(L, R)
    w_hat = lin.predict(L, R)[:, 2]
    X = np.c_[np.ones_like(fL), fL, fR, np.maximum(w_hat, 0), np.maximum(-w_hat, 0)]
    sv = np.linalg.svd(X / np.linalg.norm(X, axis=0), compute_uv=False)
    print(f"split CoR design [1, fL, fR, w+, w-]: rank {np.linalg.matrix_rank(X)} of 5, "
          f"smallest/largest singular value {sv[-1] / sv[0]:.1e} (w+ - w- = w_hat is linear in [1, fL, fR])")

    cor = fits["cor"]
    d_boot = []
    if a.boot:
        d_boot = boot_offset(L, R, Y, cor, p0_max, a.boot)
        lo, hi = np.percentile(d_boot, [2.5, 97.5])
        print(f"'cor' mean CoR offset d (p0 <= {p0_max:.0f}) = {cor.cor_offset() * 1000:.1f} mm, cell bootstrap 95 % "
              f"[{lo * 1000:.1f}, {hi * 1000:.1f}] mm ({a.boot} draws)")
    else:
        print(f"'cor' mean CoR offset d = {cor.cor_offset() * 1000:.1f} mm")

    inv = InverseMap(m)
    (v0, v1), (w0, w1) = LAB_LUT_BOX
    gv, gw = np.meshgrid(np.linspace(v0, v1, 200), np.linspace(w0, w1, 200))
    inside = np.array([inv.contains(x, y) for x, y in zip(gv.ravel(), gw.ravel())])
    print(f"operating region (L {inv.p_min[0]:.0f}..{inv.p_max[0]:.0f}, R {inv.p_min[1]:.0f}..{inv.p_max[1]:.0f}): "
          f"{100 * inside.mean():.0f} % of the lab's LUT box; vertices (v cm/s, omega rad/s): "
          + ", ".join(f"({x * 100:.1f}, {y:+.2f})" for x, y in inv.vertices))

    if a.out:
        m.meta.update({"cv": cv, "selected": best, "source": {"power": a.power, "v": a.v, "omega": a.omega,
                                                              "rows": a.rows}})
        res = {"map": m.to_dict(), "cv_ours": cv, "cv_ours_bounded": {"p0_max": p0_max, "rmse": bounded},
               "cv_lab_poly": lab, "lab_hardcoded_insample": pub,
               "lab_refit_insample": ref, "cor_offset_m": cor.cor_offset(), "cor_offset_boot_m": d_boot,
               "operating_region": {"vertices": inv.vertices.tolist(), "lab_lut_box_fraction": float(inside.mean())}}
        with open(a.out, "w") as f:
            json.dump(res, f, indent=2)
        print(f"\nwrote {a.out}")


if __name__ == "__main__":
    main()
