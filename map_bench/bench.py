"""map_bench: which commands to calibrate at, which model to fit, and how to calibrate another robot.

Any robot (a spec JSON like robots/graspion.json: inputs, outputs, ranges, noise scale, rough dead-zone prior):
    python map_bench/bench.py design --spec S.json --kind warped --n 49 [--out d.csv] [--schedule s.json --blocks 3]
    python map_bench/bench.py design --spec S.json --kind active --data so_far.csv --n 8      # next batch to measure
    python map_bench/bench.py fit    --spec S.json --data segments.csv [--models gp mlp ...] [--out map.pkl]
    python map_bench/bench.py fit    --spec S.json --data new_robot.csv --base fleet.pkl      # few-shot transfer
A data CSV has one row per segment with columns named as the spec's inputs and outputs (graspion_calib's
segments_fit.csv already has L, R, u1, u2, omega). `fit` scores every model by grouped CV (a group = all replicates
of one command), refits the best (or --model) on all rows and pickles it (models.load). --schedule writes a
graspion_calib schedule (randomised blocks) for `python -m graspion_calib run`.

Experiments on GRASPion (results/):
    designs   designs x budgets x models on synthetic robots: 'sim' (popsim population, the dead-zone family),
              'bump' (the same with a resonance dip at drive 100 that the dead-zone model cannot follow) and 'lab'
              (a smooth surface through the lab's measured 7x7 tables, drive 60..120). Score: RMSE against the
              noise-free truth over the operating box, in noise units (spec.scale), median over robots.
    lab       the lab's real 7x7 tables: leave-one-cell-out CV of every model, and level subsets / scattered subsets
              of the 49 cells scored on the cells they leave out.
    transfer  a new robot from k segments, given 7 fully gridded robots: from scratch, fleet model as is, fleet +
              affine / GP correction, and popsim's hierarchical prior.
    selftest
"""
from __future__ import annotations

import argparse
import csv
import os
import sys
import time
from types import SimpleNamespace

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.join(HERE, "..")
sys.path[:0] = [HERE, os.path.join(ROOT, "calibration", "pc"), os.path.join(ROOT, "calibration_paper"),
                os.path.join(ROOT, "lab_fit")]
import designs as D  # noqa: E402
import models as M  # noqa: E402

GRASPION = os.path.join(HERE, "robots", "graspion.json")
DESIGNS = ("uniform", "cheb", "warped", "halton", "lhs", "dopt", "active")
MODELS = ("deadzone", "poly3", "lut", "tps", "gp", "mlp", "dz+gp")


# ---------------------------------------------------------------- data in / out

def read_csv(path, spec):
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    cols = list(spec.inputs) + list(spec.outputs)
    missing = [c for c in cols if rows and c not in rows[0]]
    if missing:
        raise SystemExit(f"{path}: no column(s) {missing}")
    A = np.array([[float(r[c]) if r[c] not in ("", None) else np.nan for c in cols] for r in rows])
    A = A[~np.any(np.isnan(A), axis=1)]
    d = len(spec.inputs)
    return A[:, :d], A[:, d:]


def write_csv(path, spec, X):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(spec.inputs)
        w.writerows([[f"{v:g}" for v in x] for x in X])


def score(model, E, YE, scale):
    """(score, rmse per output): RMSE against the truth, score = RMS over outputs of rmse / scale."""
    rmse = np.sqrt(np.mean((model.predict(E) - YE) ** 2, axis=0))
    return float(np.sqrt(np.mean((rmse / scale) ** 2))), rmse


def operate_grid(spec, k=25):
    axes = [np.linspace(spec.operate[0][j], spec.operate[1][j], k) for j in range(len(spec.lo))]
    return np.array(np.meshgrid(*axes, indexing="ij")).reshape(len(axes), -1).T


# ---------------------------------------------------------------- GRASPion truths

def lab_tables():
    import fit_lab_tables as lf
    data = os.path.join(ROOT, "lab_fit", "data")
    a = SimpleNamespace(power=os.path.join(data, "power.npy"), v=os.path.join(data, "velocity_table.npy"),
                        omega=os.path.join(data, "omega_table.npy"), rows="r-asc")
    L, R, v, w = lf.load_tables(a)
    return np.c_[L, R], np.c_[v, np.zeros_like(v), w]


def lab_spec():
    """GRASPion spec restricted to the lab tables' drive box, 60..120."""
    import json
    with open(GRASPION) as f:
        d = json.load(f)
    return D.Spec(**{**d, "lo": [60, 60], "operate": [[60, 60], [120, 120]]})


def _bumped(m, depth=0.4, at=100.0, width=8.0):
    """Dead-zone truth whose activation dips by `depth` around drive `at` (resonance; not in the model family)."""
    def f(X):
        F = np.maximum(0.0, np.abs(X) - np.asarray(m.p0)) ** np.asarray(m.gamma)
        F = F * (1 - depth * np.exp(-((np.abs(X) - at) / width) ** 2))
        Y = np.c_[np.ones(len(X)), F] @ np.asarray(m.coef).T
        Y[~np.any(F > 0, axis=1)] = 0.0
        return Y
    return f


def truths(kind, n, seed=0):
    """[(spec, f)] with f(X) the noise-free twist; noise is popsim.NOISE."""
    import popsim as ps
    if kind == "lab":
        spec = lab_spec()
        X, Y = lab_tables()
        base = M.TPS(spec).fit(X, Y)
        rng = np.random.default_rng(seed)
        out = []
        for _ in range(n):                    # robot-to-robot spread: per-output gain and omega bias (popsim sizes)
            g, b = np.exp(rng.normal(0, 0.05, 3)), rng.normal(0, 0.05)
            out.append((spec, lambda X, g=g, b=b: base.predict(X) * g + [0, 0, b]))
        return out
    spec = D.Spec.load(GRASPION)
    robots = ps.draw_population(n, spread=1.0, seed=seed)
    maps = [r["days"][0] for r in robots]
    if kind == "sim":
        return [(spec, lambda X, m=m: m.predict(X[:, 0], X[:, 1])) for m in maps]
    if kind == "bump":
        return [(spec, _bumped(m)) for m in maps]
    raise ValueError(kind)


# ---------------------------------------------------------------- experiments

def run_designs(a):
    import popsim as ps
    noise = np.asarray(ps.NOISE)
    rows = []
    t0 = time.time()
    for tk in a.truths:
        tr = truths(tk, a.robots, a.seed)
        spec = tr[0][0]
        E = operate_grid(spec)
        for n in a.budgets:
            kinds = list(a.designs) + (["current"] if tk != "lab" and n >= 81 else [])
            for kind in kinds:
                res = {m: [] for m in a.models}
                for r, (spec, f) in enumerate(tr):
                    rng = np.random.default_rng(1000 * r + n)
                    oracle = lambda X: f(X) + rng.normal(0, noise, (len(X), len(noise)))
                    if kind == "active":
                        X, Y = D.active(spec, n, oracle, seed=r)
                    else:
                        if kind == "current":
                            from graspion_calib.schedules import GRID
                            G = np.array([(x, y) for x in GRID for y in GRID], float)
                            X = np.vstack([G] * max(1, round(n / len(G))))
                        else:
                            X = D.make(spec, kind, n, seed=r)
                        Y = oracle(X)
                    YE = f(E)
                    for m in a.models:
                        try:
                            res[m].append(score(M.make(m, spec).fit(X, Y), E, YE, spec.scale)[0])
                        except Exception as e:               # a model that cannot fit this design scores nan
                            print(f"  {tk} n={n} {kind} {m}: {type(e).__name__}: {e}")
                            res[m].append(np.nan)
                rows.append({"truth": tk, "n": n, "design": kind, "n_points": len(X),
                             **{m: float(np.nanmedian(v)) for m, v in res.items()}})
                print(_fmt_row(rows[-1], a.models), f"  [{time.time() - t0:.0f}s]", flush=True)
    return {"rows": rows, "models": list(a.models)}


def _fmt_row(r, models):
    return f"{r['truth']:5s} {r['n']:4d} {r['design']:8s} {r['n_points']:4d} " + " ".join(
        f"{r[m]:8.3f}" for m in models)


def _header(models):
    return "truth    n design   pts " + " ".join(f"{m:>8s}" for m in models)


def run_lab(a):
    spec = lab_spec()
    X, Y = lab_tables()
    out = {"loo": {}, "subsets": []}
    print("leave-one-cell-out on the lab's 7x7 tables (one value per cell)\n  model         v [cm/s]  omega [rad/s]")
    for m in a.models_lab:
        P = np.zeros_like(Y)
        for i in range(len(X)):
            tr = np.arange(len(X)) != i
            P[i] = M.make(m, spec).fit(X[tr], Y[tr]).predict(X[i:i + 1])[0]
        r = np.sqrt(np.mean((P - Y) ** 2, axis=0))
        out["loo"][m] = r.tolist()
        print(f"  {m:12s} {r[0] * 100:8.3f}  {r[2]:8.3f}", flush=True)

    subsets = {"uniform3": (60, 90, 120), "uniform4": (60, 80, 100, 120), "low4": (60, 70, 90, 120),
               "cheb4": (60, 70, 110, 120), "high4": (60, 90, 110, 120), "cheb5": (60, 70, 90, 110, 120),
               "mid5": (60, 80, 90, 100, 120)}
    designs = {}
    for name, s in subsets.items():
        designs[name] = [np.flatnonzero(np.isin(X[:, 0], s) & np.isin(X[:, 1], s))]
    snap = lambda P: np.unique([int(np.argmin(np.sum((X - p) ** 2, 1))) for p in P])
    for n in (16, 25):
        rng = np.random.default_rng(a.seed)
        designs[f"random{n}"] = [rng.choice(len(X), n, replace=False) for _ in range(a.draws)]
        designs[f"halton{n}"] = [snap(D.halton(spec, n, seed=s)) for s in range(a.draws)]
        designs[f"dopt{n}"] = [snap(D.dopt(spec, n, cand=X, repeats=False))]
    print("\nlevel / cell subsets of the 7x7 tables, scored on the cells left out (levels: "
          + ", ".join(f"{k} {v}" for k, v in subsets.items()) + ")")
    print(f"  {'design':10s} cells " + " ".join(f"{m:>16s}" for m in a.models_lab) + "   (v cm/s / omega rad/s)")
    for name, idx_list in designs.items():
        r_all = {m: [] for m in a.models_lab}
        for idx in idx_list:
            te = np.setdiff1d(np.arange(len(X)), idx)
            for m in a.models_lab:
                try:
                    P = M.make(m, spec).fit(X[idx], Y[idx]).predict(X[te])
                    r_all[m].append(np.sqrt(np.mean((P - Y[te]) ** 2, axis=0)))
                except Exception:
                    r_all[m].append(np.full(3, np.nan))
        med = {m: np.nanmedian(np.array(v), axis=0) for m, v in r_all.items()}
        out["subsets"].append({"design": name, "cells": int(np.mean([len(i) for i in idx_list])),
                               "rmse": {m: v.tolist() for m, v in med.items()}})
        print(f"  {name:10s} {out['subsets'][-1]['cells']:5d} "
              + " ".join(f"{med[m][0] * 100:7.3f}/{med[m][2]:7.3f}" + " " for m in a.models_lab), flush=True)
    return out


def run_transfer(a):
    import popsim as ps
    spec = D.Spec.load(GRASPION)
    robots = [r["days"][0] for r in ps.draw_population(a.robots, spread=1.0, seed=a.seed)]
    pools = [ps.segments(m, blocks=5, seed=100 + i) for i, m in enumerate(robots)]
    E = operate_grid(spec)
    full = [ps.full_fit(*p) for p in pools]
    methods = ("scratch dz", "scratch gp", "fleet dz", "fleet dz+affine", "fleet dz+gp", "fleet mlp+gp",
               "popsim hier")
    res = {m: {k: [] for k in a.ks} for m in methods}
    t0 = time.time()
    for h in range(len(robots)):
        others = [i for i in range(len(robots)) if i != h]
        Xo = np.vstack([np.c_[pools[i][0], pools[i][1]] for i in others])
        Yo = np.vstack([pools[i][2] for i in others])
        Uo, Mo, _ = M.unique_means(Xo, Yo)                     # fleet = per-cell means over the other robots
        fleet = {"dz": M.make("deadzone", spec).fit(Uo, Mo), "mlp": M.make("mlp", spec).fit(Uo, Mo)}
        prior = ps.Prior.from_maps([full[i] for i in others])
        YE = robots[h].predict(E[:, 0], E[:, 1])
        L, R, Y = pools[h]
        for k in a.ks:
            for rep in range(a.reps):
                idx = ps.pick_random(L, R, k, np.random.default_rng(1000 * h + 10 * k + rep))
                X = np.c_[L[idx], R[idx]]
                fits = {
                    "scratch dz": lambda: M.make("deadzone", spec).fit(X, Y[idx]) if k >= 8 else None,
                    "scratch gp": lambda: M.make("gp", spec).fit(X, Y[idx]) if k >= 8 else None,
                    "fleet dz": lambda: fleet["dz"],
                    "fleet dz+affine": lambda: M.Residual(spec, fleet["dz"], M.make("affine", spec), True).fit(X, Y[idx]),
                    "fleet dz+gp": lambda: M.Residual(spec, fleet["dz"], M.make("gp", spec), True).fit(X, Y[idx]),
                    "fleet mlp+gp": lambda: M.Residual(spec, fleet["mlp"], M.make("gp", spec), True).fit(X, Y[idx]),
                }
                for m, mk in fits.items():
                    model = mk()
                    res[m][k].append(np.nan if model is None else score(model, E, YE, spec.scale)[0])
                hm = ps.fit_map(L[idx], R[idx], Y[idx], prior)
                res["popsim hier"][k].append(ps.score(hm, E[:, 0], E[:, 1], YE)[1])
        print(f"  robot {h} done [{time.time() - t0:.0f}s]", flush=True)
    print(f"\nnew robot from k random segments, prior / fleet from {a.robots - 1} gridded robots "
          f"(score in noise units, median over robots x {a.reps} draws)")
    print(f"  {'method':16s} " + " ".join(f"k={k:<5d}" for k in a.ks))
    for m in methods:
        print(f"  {m:16s} " + " ".join(f"{np.nanmedian(res[m][k]):7.3f}" if np.any(np.isfinite(res[m][k]))
                                       else "      -" for k in a.ks))
    return {m: {k: [float(x) for x in v] for k, v in d.items()} for m, d in res.items()}


# ---------------------------------------------------------------- robot-generic commands

def cmd_design(a):
    spec = D.Spec.load(a.spec)
    if a.kind == "active":
        if not a.data:
            raise SystemExit("--kind active proposes the next batch from measurements so far: give --data")
        X = D.next_batch(spec, *read_csv(a.data, spec), a.n)
    elif a.kind == "current":
        from graspion_calib.schedules import GRID
        X = np.array([(x, y) for x in GRID for y in GRID], float)
    else:
        X = D.make(spec, a.kind, a.n, seed=a.seed)
        X = np.unique(X, axis=0) if a.kind in D.PRODUCT else X
    print(f"{len(X)} points ({a.kind}); per input levels used: "
          + "; ".join(f"{nm}: {len(np.unique(X[:, j]))}" for j, nm in enumerate(spec.inputs)))
    if a.kind in D.PRODUCT:
        for j, nm in enumerate(spec.inputs):
            print(f"  {nm}: {' '.join(f'{v:g}' for v in np.unique(X[:, j]))}")
    if a.out:
        write_csv(a.out, spec, X)
        print(f"wrote {a.out}")
    if a.schedule:
        from graspion_calib import schedules as S
        rng = np.random.default_rng(a.seed)
        specs = [S.drive(int(x[0]), int(x[1]), a.drive_s, block=b, tag=f"design-{a.kind}")
                 for b in range(a.blocks) for x in X[rng.permutation(len(X))] if (x[0], x[1]) != (0, 0)]
        S.save(specs, a.schedule, {"kind": f"map_bench {a.kind}", "seed": a.seed, "n": a.n})
        print(f"wrote {a.schedule}: {S.summary(specs)}")


def cmd_fit(a):
    spec = D.Spec.load(a.spec)
    X, Y = read_csv(a.data, spec)
    names = a.models or (("zero", "affine", "gp") if a.base else tuple(M.ZOO))
    base = M.load(a.base) if a.base else None
    if base is not None:                                      # transfer: candidates are corrections on the fleet map
        for nm in names:
            M.ZOO[f"base+{nm}"] = (lambda s, nm=nm: M.Residual(s, base, M.ZOO[nm](s), frozen=True))
        names = [f"base+{nm}" for nm in names]
    n_pts = len(np.unique(X, axis=0))
    print(f"{len(X)} rows, {n_pts} distinct commands; grouped {min(10, n_pts)}-fold CV RMSE:")
    print(f"  {'model':16s} " + " ".join(f"{o:>9s}" for o in spec.outputs) + "    score")
    cv = M.grouped_cv(spec, X, Y, names)
    sc = {nm: float(np.sqrt(np.mean((r / spec.scale) ** 2))) for nm, r in cv.items()}
    for nm in sorted(cv, key=sc.get):
        print(f"  {nm:16s} " + " ".join(f"{v:9.4g}" for v in cv[nm]) + f"  {sc[nm]:7.3f}")
    pick = a.model or min(sc, key=sc.get)
    if a.base and a.model:
        pick = f"base+{a.model}"
    model = M.make(pick, spec).fit(X, Y)
    print(f"selected {pick}")
    if a.out:
        model.save(a.out)
        print(f"wrote {a.out} (models.load)")


# ---------------------------------------------------------------- main

def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("design")
    p.add_argument("--spec", default=GRASPION)
    p.add_argument("--kind", default="warped", choices=D.PRODUCT + D.SCATTERED + ("current",))
    p.add_argument("--n", type=int, default=49)
    p.add_argument("--data", help="measurements so far (active)")
    p.add_argument("--out")
    p.add_argument("--schedule", help="also write a graspion_calib schedule JSON (two-input robots)")
    p.add_argument("--blocks", type=int, default=1)
    p.add_argument("--drive-s", type=float, default=3.0)
    p.add_argument("--seed", type=int, default=0)
    p = sub.add_parser("fit")
    p.add_argument("--spec", default=GRASPION)
    p.add_argument("--data", required=True)
    p.add_argument("--models", nargs="+", choices=tuple(M.ZOO))
    p.add_argument("--model", choices=tuple(M.ZOO), help="refit this one instead of the CV winner")
    p.add_argument("--base", help="fleet model (pickle from fit --out) to correct with few points")
    p.add_argument("--out")
    p = sub.add_parser("designs")
    p.add_argument("--truths", nargs="+", default=["sim", "bump", "lab"], choices=("sim", "bump", "lab"))
    p.add_argument("--budgets", type=int, nargs="+", default=[25, 49, 81, 169])
    p.add_argument("--designs", nargs="+", default=list(DESIGNS), choices=DESIGNS)
    p.add_argument("--models", nargs="+", default=list(MODELS), choices=tuple(M.ZOO))
    p.add_argument("--robots", type=int, default=3)
    p.add_argument("--seed", type=int, default=0)
    p = sub.add_parser("lab")
    p.add_argument("--models-lab", nargs="+", default=list(MODELS) + ["poly2", "dz+mlp"], choices=tuple(M.ZOO))
    p.add_argument("--draws", type=int, default=10)
    p.add_argument("--seed", type=int, default=0)
    p = sub.add_parser("transfer")
    p.add_argument("--robots", type=int, default=8)
    p.add_argument("--ks", type=int, nargs="+", default=[4, 8, 16, 32])
    p.add_argument("--reps", type=int, default=2)
    p.add_argument("--seed", type=int, default=0)
    sub.add_parser("selftest")
    a = ap.parse_args(argv)
    if a.cmd == "selftest":
        import unittest
        ok = unittest.TextTestRunner(verbosity=1).run(
            unittest.defaultTestLoader.discover(HERE, "test_map_bench.py")).wasSuccessful()
        sys.exit(0 if ok else 1)
    if a.cmd == "design":
        return cmd_design(a)
    if a.cmd == "fit":
        return cmd_fit(a)
    if a.cmd == "designs":
        print(_header(a.models))
    return {"designs": run_designs, "lab": run_lab, "transfer": run_transfer}[a.cmd](a)


if __name__ == "__main__":
    main()
