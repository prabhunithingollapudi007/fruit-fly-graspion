"""Forward map, inverse map + polytope and early-stop detection on synthetic data."""
import unittest

import numpy as np

from graspion_calib import extract, forward_map as fm
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

    def test_bootstrap(self):
        L, R, Y = grid_data(default_truth(), reps=2)
        draws = fm.bootstrap(L, R, Y, n=4)
        self.assertEqual(len(draws), 4)
        self.assertTrue(all(len(d["p0"]) == 2 for d in draws))

    def test_save_load_round_trip(self):
        m = default_truth()
        d = fm.PowerLawMap.from_dict(m.to_dict())
        np.testing.assert_allclose(d.predict([80, 100], [80, 60]), m.predict([80, 100], [80, 60]))


class TestInverseMap(unittest.TestCase):
    def setUp(self):
        self.fwd = default_truth()
        self.inv = InverseMap(self.fwd)

    def test_round_trip_inside_polytope(self):
        rng = np.random.default_rng(0)
        for _ in range(300):
            L, R = rng.uniform(60, 120, 2)
            v, w = self.fwd.predict([L], [R])[0][[0, 2]]
            self.assertTrue(self.inv.contains(v, w, tol=1e-9))
            Ls, Rs = self.inv.solve(v, w)
            self.assertAlmostEqual(Ls, L, delta=1e-6)
            self.assertAlmostEqual(Rs, R, delta=1e-6)

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

    def test_dead_zone_above_default_p_min(self):
        """A worn bot whose left motor starts at 63 used to get no inverse at all (p_min 60 <= p0)."""
        t = default_truth()
        f = fm.PowerLawMap((63.0, 50.0), t.gamma, t.coef)
        inv = InverseMap(f)
        np.testing.assert_allclose(inv.p_min, [68.0, 60.0])
        self.assertTrue(inv.to_dict()["p_min_auto"])
        v, w = f.predict([90], [100])[0][[0, 2]]
        np.testing.assert_allclose(inv.solve(v, w), [90, 100], atol=1e-6)
        with self.assertRaises(ValueError):
            InverseMap(f, p_min=(60.0, 60.0))                       # an explicit p_min inside the dead zone
        with self.assertRaises(ValueError):
            InverseMap(fm.PowerLawMap((118.0, 50.0), t.gamma, t.coef))   # no operating region left


def drive_segment(stop_at=None, drop_at=(), T=3.0, fps=60.0, u1=0.08, omega=0.8, tau=0.06, seed=0):
    """A tracked drive segment from t_cmd = 5 s: first-order lag tau to (u1, omega), stopping at t_cmd + stop_at
    (e.g. a watchdog trip) or at the STOP; frames at t_cmd + drop_at are missing (marker not detected)."""
    rng = np.random.default_rng(seed)
    t_cmd = 5.0
    t_off = t_cmd + (T if stop_at is None else stop_at)
    t = np.arange(t_cmd - 0.5, t_cmd + T + 1.0, 1 / fps)
    g = np.where(t < t_cmd, 0.0, 1 - np.exp(-(t - t_cmd) / tau))
    g = np.where(t < t_off, g, (1 - np.exp(-(t_off - t_cmd) / tau)) * np.exp(-(t - t_off) / tau))
    th = 0.3 + np.cumsum(omega * g) / fps
    x = np.cumsum(u1 * g * np.cos(th)) / fps + rng.normal(0, 3e-4, len(t))
    y = np.cumsum(u1 * g * np.sin(th)) / fps + rng.normal(0, 3e-4, len(t))
    th = th + rng.normal(0, 0.004, len(t))
    keep = np.ones(len(t), bool)
    for d in drop_at:
        keep[np.argmin(np.abs(t - t_cmd - d))] = False
    return extract.SegmentData(0, 0, "drive", "grid", 0, 80, 80, "ok", True, 0.05, t_cmd, t_cmd + T,
                               [(t_cmd, 80, 80), (t_cmd + T, 0, 0)], t[keep], x[keep], y[keep], th[keep])


class TestEarlyStop(unittest.TestCase):
    """extract.steady_window: a segment the bot stopped early (watchdog trip) is cut short at the stop."""

    def test_dropped_frames_at_the_end_are_not_a_stop(self):
        """Two dropped frames left tail runs shorter than the velocity filter, which used to read as a stop."""
        sd = drive_segment(drop_at=(2.68, 2.85))
        t0, t1, truncated = extract.steady_window(sd)
        self.assertIsNone(sd.stop)
        self.assertFalse(truncated)

    def test_stop_early_in_the_window(self):
        """A trip 0.3 s into the steady window: the median speed of the first half was ~0 and missed it."""
        sd = drive_segment(stop_at=0.8)
        t0, t1, truncated = extract.steady_window(sd)
        self.assertTrue(truncated)
        self.assertAlmostEqual(sd.stop, sd.t_cmd + 0.8, delta=0.1)
        for stop_at in (1.5, 2.4):
            sd = drive_segment(stop_at=stop_at, seed=1)
            extract.steady_window(sd)
            self.assertAlmostEqual(sd.stop, sd.t_cmd + stop_at, delta=0.1, msg=stop_at)


if __name__ == "__main__":
    unittest.main()
