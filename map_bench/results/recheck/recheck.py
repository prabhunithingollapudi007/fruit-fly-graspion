"""Robustness re-checks of map_bench claims: fresh robots (seed 7), more of them, IQR, wrong design prior.

    python map_bench/results/recheck/recheck.py sim|prior|bump      (outputs: sim.txt, prior.txt, bump.txt here)
"""
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
import bench, designs as D, models as M
import popsim as ps
from graspion_calib.schedules import GRID

part = sys.argv[1]
noise = np.asarray(ps.NOISE)
GR = np.array([(x, y) for x in GRID for y in GRID], float)

def run(truth, n_rob, budgets, kinds, models, spec_mod=None, seed=7):
    tr = bench.truths(truth, n_rob, seed)
    t0 = time.time()
    for n in budgets:
        for kind in kinds:
            if kind == "current" and n < 81:
                continue
            res = {m: [] for m in models}
            for r, (spec, f) in enumerate(tr):
                sp = spec_mod(spec) if spec_mod else spec
                rng = np.random.default_rng(1000 * r + n + 5)
                orc = lambda X: f(X) + rng.normal(0, noise, (len(X), 3))
                if kind == "active":
                    X, Y = D.active(sp, n, orc, seed=r)
                else:
                    X = np.vstack([GR] * max(1, round(n / 81))) if kind == "current" else D.make(sp, kind, n, seed=r)
                    Y = orc(X)
                E = bench.operate_grid(spec); YE = f(E)
                for m in models:
                    res[m].append(bench.score(M.make(m, spec).fit(X, Y), E, YE, spec.scale)[0])
            print(f"{truth:4s} n={n:3d} {kind:8s} " + "  ".join(
                f"{m}: {np.median(v):.3f} [{np.percentile(v,25):.3f},{np.percentile(v,75):.3f}]" for m, v in res.items()),
                f"[{time.time()-t0:.0f}s]", flush=True)

def wrong(p0, g):
    def mod(spec):
        return D.Spec(**{"inputs": spec.inputs, "outputs": spec.outputs, "lo": list(spec.lo), "hi": list(spec.hi),
                         "scale": list(spec.scale), "off": list(spec.off),
                         "operate": [list(spec.operate[0]), list(spec.operate[1])],
                         "prior": {"p0": [p0, p0], "gamma": [g, g]}})
    return mod

if part == "sim":
    run("sim", 10, [25, 49, 81, 169], ["uniform", "warped", "halton", "lhs", "current", "dopt"], ["deadzone"])
elif part == "prior":
    for p0, g in ((35, 1.0), (65, 1.0), (50, 1.2)):
        print(f"-- design prior p0={p0} gamma={g} (truth mean p0~51, gamma~0.62)")
        run("sim", 10, [49, 81], ["warped", "dopt"], ["deadzone"], spec_mod=wrong(p0, g))
elif part == "bump":
    run("bump", 6, [49, 81, 169], ["uniform", "warped", "lhs", "current"], ["deadzone", "tps", "gp", "mlp", "dz+gp"])
