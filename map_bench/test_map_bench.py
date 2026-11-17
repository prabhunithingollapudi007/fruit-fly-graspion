"""Offline tests of map_bench:  python map_bench/bench.py selftest"""
import os
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import bench  # noqa: E402  (sets the import paths)
import designs as D  # noqa: E402
import models as M  # noqa: E402
from graspion_calib import forward_map as fm  # noqa: E402
from graspion_calib.sim import default_truth  # noqa: E402


def graspion():
    return D.Spec.load(bench.GRASPION)


def three_input():
    """A made-up 3-motor robot with 2 outputs: the tools must not assume GRASPion's shape."""
    return D.Spec(inputs=["a", "b", "c"], outputs=["vx", "wz"], lo=[0, 0, 0], hi=[255, 255, 255],
                  scale=[0.01, 0.1], prior={"p0": [40, 40, 40], "gamma": [0.8, 0.8, 0.8]})


def three_truth(X):
    F = np.maximum(0, X - [30, 45, 50]) ** np.array([0.7, 0.9, 0.8])
    Y = np.c_[0.002 * F.sum(1), 0.02 * (F[:, 0] - F[:, 1]) + 0.01 * F[:, 2]]
    Y[~np.any(F > 0, 1)] = 0
    return Y


class TestDesigns(unittest.TestCase):
    def test_levels(self):
        np.testing.assert_allclose(D.levels("uniform", 13, 0, 120), np.arange(0, 121, 10))
        c = D.levels("cheb", 5, 0, 120)
        self.assertEqual((c[0], c[-1]), (0, 120))
        self.assertLess(c[1] - c[0], c[2] - c[1])                      # denser at the ends
        w = D.levels("warped", 7, 0, 120, off=0, p0=50, gamma=0.6)
        self.assertEqual(w[0], 0)
        self.assertTrue(np.all(w[1:] > 50))
        self.assertTrue(np.all(np.diff(np.diff(w[1:])) > 0))          # steps grow away from the threshold

    def test_shapes_bounds_snapping(self):
        for spec in (graspion(), three_input()):
            for kind in ("uniform", "cheb", "warped", "halton", "lhs", "dopt"):
                X = D.make(spec, kind, 30, seed=1)
                self.assertEqual(X.shape[1], len(spec.inputs))
                self.assertTrue(np.all(X >= spec.lo) and np.all(X <= spec.hi), kind)
                np.testing.assert_allclose(X, np.round(X))
        self.assertEqual(len(D.make(graspion(), "uniform", 49)), 49)
        self.assertEqual(len(D.make(graspion(), "uniform", 4 * 49)), 4 * 49)   # 14 x 14 levels, not 7x7 repeated

    def test_dopt_without_repeats_from_a_candidate_set(self):
        spec = graspion()
        cand = np.array([(a, b) for a in range(60, 121, 10) for b in range(60, 121, 10)], float)
        X = D.dopt(spec, 20, cand=cand, repeats=False)
        self.assertEqual(len(np.unique(X, axis=0)), 20)

    def test_active_and_next_batch(self):
        spec = graspion()
        t = default_truth()
        X, Y = D.active(spec, 16, lambda X: t.predict(X[:, 0], X[:, 1]), batch=4)
        self.assertEqual(X.shape, (16, 2))
        self.assertEqual(Y.shape, (16, 3))
        self.assertEqual(len(D.next_batch(spec, X, Y, 5)), 5)


class TestModels(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.spec = graspion()
        t = default_truth()
        cls.X = D.make(cls.spec, "warped", 81)
        cls.Y = t.predict(cls.X[:, 0], cls.X[:, 1])
        cls.E = bench.operate_grid(cls.spec, 15)
        cls.YE = t.predict(cls.E[:, 0], cls.E[:, 1])

    def test_deadzone_matches_forward_map(self):
        m = M.DeadZone(self.spec).fit(self.X, self.Y)
        ref = fm.fit_power_law(self.X[:, 0], self.X[:, 1], self.Y)
        np.testing.assert_allclose(m.predict(self.E), ref.predict(self.E[:, 0], self.E[:, 1]), atol=2e-4)
        np.testing.assert_allclose(m.p0, (52, 50), atol=0.5)
        np.testing.assert_allclose(m.gamma, (0.65, 0.60), atol=0.01)

    def test_every_model_learns_the_noise_free_map(self):
        limit = {"zero": None, "affine": None, "poly2": 2.0}
        for name in M.ZOO:
            if name in limit and limit[name] is None:
                continue
            s, _ = bench.score(M.make(name, self.spec).fit(self.X, self.Y), self.E, self.YE, self.spec.scale)
            self.assertLess(s, limit.get(name, 0.8), name)

    def test_uncertainty_and_pickle(self):
        for name in ("gp", "mlp", "dz+gp"):
            m = M.make(name, self.spec).fit(self.X, self.Y)
            mu, sd = m.predict_std(self.E)
            self.assertEqual(sd.shape, self.YE.shape)
            self.assertTrue(np.all(sd >= 0))
            with tempfile.TemporaryDirectory() as d:
                m.save(os.path.join(d, "m.pkl"))
                np.testing.assert_allclose(M.load(os.path.join(d, "m.pkl")).predict(self.E), mu)

    def test_three_input_robot(self):
        spec = three_input()
        X = D.make(spec, "lhs", 120, seed=0)
        Y = three_truth(X)
        E = D.make(spec, "halton", 200, seed=5)
        for name in ("deadzone", "poly3", "lut", "tps", "gp", "mlp", "dz+gp"):
            P = M.make(name, spec).fit(X, Y).predict(E)
            self.assertEqual(P.shape, (200, 2), name)
        dz = M.DeadZone(spec).fit(X, Y)
        np.testing.assert_allclose(dz.p0, [30, 45, 50], atol=1.0)

    def test_one_input_robot(self):
        spec = D.Spec(inputs=["pwm"], outputs=["thrust"], lo=[0], hi=[100], scale=[0.01])
        X = np.linspace(0, 100, 21)[:, None]
        Y = np.maximum(0, X - 20) ** 1.5 * 1e-3
        for name in ("deadzone", "poly3", "lut", "tps", "gp", "mlp"):
            P = M.make(name, spec).fit(X, Y).predict(X)
            self.assertLess(np.sqrt(np.mean((P - Y) ** 2)), 0.02, name)

    def test_frozen_residual_corrects_a_gain_change(self):
        fleet = M.DeadZone(self.spec).fit(self.X, self.Y)
        idx = np.random.default_rng(0).choice(len(self.X), 10, replace=False)
        Ynew = self.Y * [1.1, 1.0, 0.9] + [0, 0, 0.1]                  # a robot 10 % off in speed and yaw rate
        r = M.Residual(self.spec, fleet, M.make("affine", self.spec), frozen=True).fit(self.X[idx], Ynew[idx])
        before = np.abs(fleet.predict(self.X) - Ynew).mean(0) / self.spec.scale
        after = np.abs(r.predict(self.X) - Ynew).mean(0) / self.spec.scale
        self.assertLess(after[2], 0.5 * before[2])
        np.testing.assert_allclose(fleet.predict(self.X), self.Y, atol=1e-3)   # base untouched

    def test_grouped_cv_keeps_replicates_together(self):
        X = np.vstack([self.X] * 2)
        Y = np.vstack([self.Y] * 2)
        cv = M.grouped_cv(self.spec, X, Y, ["lut"])
        self.assertGreater(cv["lut"][2], 1e-3)        # replicates leaked into training would give ~0


class TestCLI(unittest.TestCase):
    def test_design_then_fit_then_transfer(self):
        t = default_truth()
        with tempfile.TemporaryDirectory() as d:
            dcsv, data, out = (os.path.join(d, f) for f in ("d.csv", "data.csv", "m.pkl"))
            sched = os.path.join(d, "s.json")
            bench.main(["design", "--kind", "warped", "--n", "36", "--out", dcsv, "--schedule", sched,
                        "--blocks", "2"])
            X = np.loadtxt(dcsv, delimiter=",", skiprows=1)
            Y = t.predict(X[:, 0], X[:, 1]) + np.random.default_rng(0).normal(0, [0.003, 0.002, 0.05], (len(X), 3))
            with open(data, "w") as f:
                f.write("L,R,u1,u2,omega\n" + "\n".join(",".join(f"{v:g}" for v in r) for r in np.c_[X, Y]))
            bench.main(["fit", "--data", data, "--models", "deadzone", "lut", "--out", out])
            self.assertIsInstance(M.load(out), M.DeadZone)
            bench.main(["fit", "--data", data, "--base", out, "--models", "zero", "affine"])
            from graspion_calib import schedules as S
            specs = S.load(sched)
            self.assertEqual(len(specs), 2 * (len(X) - 1))                  # (0, 0) dropped, two blocks


if __name__ == "__main__":
    unittest.main()
