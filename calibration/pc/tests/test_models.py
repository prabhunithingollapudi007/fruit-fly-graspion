"""Forward map, inverse map + polytope, heading diffusion, lag / dead time on synthetic data."""
import unittest

import numpy as np

from graspion_calib import dynamics, forward_map as fm, noise
from graspion_calib.inverse_map import InverseMap
from graspion_calib.sim import default_truth

GRID = (0, 55, 60, 70, 80, 90, 100, 110, 120)


def grid_data(truth, reps=5, noise_sd=(0.004, 0.003, 0.08), seed=0):
    rng = np.random.default_rng(seed)
    cells = [(a, b) for a in GRID for b in GRID if (a, b) != (0, 0)]
    L = np.array([c[0] for c in cells] * reps, float)
    R = np.array([c[1] for c in cells] * reps, float)
    Y = truth.predict(L, R) + rng.normal(0, noise_sd, (len(L), 3))
    return L, R, Y


class TestForwardMap(unittest.TestCase):
    def test_recovers_parameters(self):
        truth = default_truth()
        L, R, Y = grid_data(truth)
        m = fm.fit_power_law(L, R, Y)
        np.testing.assert_allclose(m.p0, truth.p0, atol=2.0)
        np.testing.assert_allclose(m.gamma, truth.gamma, atol=0.06)
        g = np.linspace(60, 120, 7)
        GL, GR = [a.ravel() for a in np.meshgrid(g, g)]
        err = m.predict(GL, GR) - truth.predict(GL, GR)
        self.assertLess(np.sqrt(np.mean(err[:, 0] ** 2)), 0.002)     # u1 within 2 mm/s
        self.assertLess(np.sqrt(np.mean(err[:, 2] ** 2)), 0.05)      # omega within 0.05 rad/s
        self.assertTrue(np.all(m.predict([0, 30], [0, 40]) == 0))    # dead zone -> no motion

    def test_cv_prefers_interaction_when_present(self):
        truth = default_truth()
        truth = fm.PowerLawMap(truth.p0, truth.gamma, np.c_[truth.coef, [0.0, 0.0, -0.012]], features="int")
        L, R, Y = grid_data(truth, seed=1)
        cv = fm.grouped_cv(L, R, Y, variants=("lin", "int"))
        self.assertLess(cv["int"]["score"], cv["lin"]["score"])
        m = fm.select_and_fit(L, R, Y)
        self.assertEqual(m.meta["selected"], "int")

    def test_battery_feature_and_gp(self):
        truth = default_truth()
        L, R, Y = grid_data(truth, reps=2, seed=2)
        rng = np.random.default_rng(3)
        V = rng.uniform(3.5, 4.1, len(L))
        Y[:, 0] *= 1 + 0.3 * (V - 3.8)                                # speed rises with battery voltage
        cv = fm.grouped_cv(L, R, Y, V=V, variants=("lin", "bat"), gp=True, k_folds=5)
        self.assertLess(cv["bat"]["score"], cv["lin"]["score"])
        self.assertTrue(np.isfinite(cv["gp"]["score"]))

    def test_bootstrap(self):
        L, R, Y = grid_data(default_truth(), reps=2)
        draws = fm.bootstrap(L, R, Y, n=4)
        self.assertEqual(len(draws), 4)
        self.assertTrue(all(len(d["p0"]) == 2 for d in draws))


class TestInverseMap(unittest.TestCase):
    def setUp(self):
        self.fwd = default_truth()
        self.inv = InverseMap(self.fwd)

    def test_round_trip_inside_polytope(self):
        rng = np.random.default_rng(0)
        n = 0
        for _ in range(300):
            L, R = rng.uniform(60, 120, 2)
            v, w = self.fwd.predict([L], [R])[0][[0, 2]]
            self.assertTrue(self.inv.contains(v, w, tol=1e-9))
            Ls, Rs = self.inv.solve(v, w)
            self.assertAlmostEqual(Ls, L, delta=1e-6)
            self.assertAlmostEqual(Rs, R, delta=1e-6)
            n += 1
        self.assertEqual(n, 300)

    def test_polytope_is_parallelogram_of_corners(self):
        self.assertEqual(len(self.inv.vertices), 4)
        self.assertEqual(self.inv.H.shape, (4, 2))
        for p in (60, 120):
            for q in (60, 120):
                v, w = self.fwd.predict([p], [q])[0][[0, 2]]
                self.assertTrue(self.inv.contains(v, w, tol=1e-9))

    def test_projection_and_drive(self):
        L, R, ok = self.inv.drive_for(0.30, 0.0)                    # far too fast
        self.assertFalse(ok)
        self.assertTrue(60 <= L <= 120 and 60 <= R <= 120)
        v, w = self.inv.project(0.30, 0.0)
        self.assertTrue(self.inv.contains(v, w, tol=1e-6))
        vin, win = self.fwd.predict([90], [80])[0][[0, 2]]
        self.assertEqual(self.inv.project(vin, win), (float(vin), float(win)))

    def test_numeric_inverse_for_interaction_model(self):
        f = fm.PowerLawMap(self.fwd.p0, self.fwd.gamma, np.c_[self.fwd.coef, [0.0, 0.0, -0.01]], features="int")
        inv = InverseMap(f)
        v, w = f.predict([85], [105])[0][[0, 2]]
        Ls, Rs = inv.solve(v, w)
        self.assertAlmostEqual(Ls, 85, delta=0.05)
        self.assertAlmostEqual(Rs, 105, delta=0.05)

    def test_cor_model_closed_form(self):
        t = default_truth()
        f = fm.PowerLawMap(t.p0, t.gamma, np.c_[t.coef, [0.015, 0.0, 0.0]], features="cor")
        inv = InverseMap(f)
        self.assertTrue(inv.linear)
        rng = np.random.default_rng(5)
        for _ in range(50):
            L, R = rng.uniform(60, 120, 2)
            u1, _, w = f.predict([L], [R])[0]
            v = inv.v_from_u1(u1, w)
            self.assertTrue(inv.contains(v, w, tol=1e-9))
            np.testing.assert_allclose(inv.solve(v, w), [L, R], atol=1e-6)
        self.assertIn("_v_from_u1", inv.c_header("b"))

    def test_c_header(self):
        h = self.inv.c_header("bot90")
        for s in ("bot90_drive", "bot90_feasible", "bot90_Ainv", "#pragma once"):
            self.assertIn(s, h)


class TestNoise(unittest.TestCase):
    def _runs(self, D, sigma, n_seg=40, T=3.0, fps=60.0, seed=0):
        rng = np.random.default_rng(seed)
        runs = []
        dt = 1 / fps
        for i in range(n_seg):
            w = rng.uniform(-3, 2)                                  # every segment has its own turn rate
            n = int(T * fps)
            th = np.cumsum(w * dt + np.sqrt(2 * D * dt) * rng.normal(size=n))
            runs.append((np.arange(n) * dt, th + rng.normal(0, sigma, n)))
        return runs

    def test_brownian_recovery_pooled_segments(self):
        for D in (0.005, 0.05):
            r = noise.heading_diffusion(self._runs(D, 0.004), max_lag_s=0.5)
            self.assertAlmostEqual(r["D_r"] / D, 1.0, delta=0.15, msg=f"D={D}: {r['D_r']}")
            if D == 0.005:                                   # at high D the 6 sigma^2 term is not identifiable
                self.assertAlmostEqual(r["sigma_meas"], 0.004, delta=0.0015)

    def test_ou_fit(self):
        rng = np.random.default_rng(4)
        dt, n, tc, s2 = 1 / 60, 60 * 120, 0.4, 0.05
        eta = np.zeros(n)
        a = np.exp(-dt / tc)
        for k in range(1, n):
            eta[k] = a * eta[k - 1] + np.sqrt(s2 * (1 - a * a)) * rng.normal()
        th = np.cumsum((0.7 + eta) * dt) + rng.normal(0, 0.003, n)
        r = noise.heading_diffusion([(np.arange(n) * dt, th)], max_lag_s=2.0, ou=True)
        self.assertLess(r["ou"]["cost_ou"], r["ou"]["cost_brownian"])
        self.assertAlmostEqual(r["ou"]["tc"], tc, delta=0.2)
        self.assertAlmostEqual(r["ou"]["D_r"], s2 * tc, delta=0.5 * s2 * tc)


class TestDynamics(unittest.TestCase):
    def test_lag_and_dead_time(self):
        """Direct synthetic data: noisy poses of a bot whose twist follows a delayed first-order response."""
        truth = default_truth()
        rng = np.random.default_rng(0)
        d_true, tau_true, fps = 0.04, 0.07, 60.0
        segs = []
        for i, (a, b) in enumerate([((60, 60), (110, 110)), ((90, 0), (90, 90)), ((0, 90), (70, 100)),
                                    ((110, 110), (60, 60)), ((70, 100), (100, 70)), ((80, 80), (0, 0))]):
            levels = [(0.0, *a), (1.5, *b), (3.0, 0, 0)]
            dtf = 0.001
            tf = np.arange(-0.5, 4.0, dtf)
            ss = truth.predict(np.array([lv[1] for lv in levels]), np.array([lv[2] for lv in levels]))
            u = np.zeros((len(tf), 3))
            for k in range(3):
                u[:, k] = dynamics.step_response(tf, [lv[0] for lv in levels], np.diff(np.r_[0, ss[:, k]]),
                                                 d_true, tau_true)
            th = np.cumsum(u[:, 2]) * dtf + rng.uniform(-3, 3)
            c, s = np.cos(th), np.sin(th)
            x = np.cumsum(c * u[:, 0] - s * u[:, 1]) * dtf
            y = np.cumsum(s * u[:, 0] + c * u[:, 1]) * dtf
            idx = np.arange(0, len(tf), int(1 / fps / dtf))
            segs.append(dynamics.DynSegment(tf[idx], x[idx] + rng.normal(0, 2e-4, len(idx)),
                                            y[idx] + rng.normal(0, 2e-4, len(idx)),
                                            th[idx] + rng.normal(0, 0.003, len(idx)), levels, truth))
        r = dynamics.fit_lag(segs)
        for k in ("omega", "joint"):
            self.assertAlmostEqual(r[k]["dead_time"], d_true, delta=0.01, msg=k)
            self.assertAlmostEqual(r[k]["tau"], tau_true, delta=0.015, msg=k)


if __name__ == "__main__":
    unittest.main()
