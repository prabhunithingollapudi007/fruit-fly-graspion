"""Forward map, inverse map + polytope, heading diffusion, lag / dead time and early-stop detection on synthetic
data."""
import unittest

import numpy as np

from graspion_calib import dynamics, extract, forward_map as fm, noise
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

    def test_battery_model_without_voltages(self):
        """A loaded 'bat' map (schedule validation) is evaluated at its median training voltage, not a crash."""
        t = default_truth()
        f = fm.PowerLawMap(t.p0, t.gamma, np.c_[t.coef, [0.002, 0.0, 0.0], [0.002, 0.0, 0.0]], features="bat",
                           meta={"v_bat_median": 3.9})
        inv = InverseMap(f)
        self.assertEqual(inv.v_bat, 3.9)
        v, w = f.predict([85], [105], [3.9])[0][[0, 2]]
        np.testing.assert_allclose(inv.solve(v, w), [85, 105], atol=0.05)
        self.assertEqual(InverseMap(fm.PowerLawMap(t.p0, t.gamma, f.coef, features="bat")).v_bat, f.v_ref)


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

    def test_pi_flip_does_not_inflate_diffusion(self):
        """One pi-flipped detection used to unwrap into a 2 pi step and read as ~10x the true D_r."""
        D = 0.01
        runs = self._runs(D, 0.004, n_seg=100, T=2.5, seed=3)
        clean = noise.heading_diffusion(runs, max_lag_s=0.5)["D_r"]
        for k in (0, 7):                                           # a turning segment, flip in mid-run
            t, th = runs[k]
            th = th.copy()
            th[60] += np.pi
            runs[k] = (t, np.mod(th + np.pi, 2 * np.pi) - np.pi)   # wrapped, as the tracker reports it
        r = noise.heading_diffusion(runs, max_lag_s=0.5)
        self.assertAlmostEqual(r["D_r"] / D, 1.0, delta=0.15)
        self.assertAlmostEqual(r["D_r"] / clean, 1.0, delta=0.05)
        th = np.random.default_rng(0).normal(0, 0.003, 600)       # static run with one flip
        th[300] += np.pi
        st = noise.static_noise(np.arange(600) / 60, np.zeros(600), np.zeros(600), th)
        self.assertLess(st["sd_th"], 0.01)
        self.assertEqual(st["n_spikes"], 1)

    def test_velocity_jitter(self):
        """A constant twist gives ~0 jitter; a 0.5 Hz speed modulation of amplitude a gives sd_u1 ~ a / sqrt 2."""
        dt, a = 1 / 60, 0.01
        t = np.arange(0, 20, dt)
        for amp in (0.0, a):
            u1 = 0.08 + amp * np.sin(2 * np.pi * 0.5 * t)
            th = 1.2 * t
            x = np.cumsum(u1 * np.cos(th)) * dt
            y = np.cumsum(u1 * np.sin(th)) * dt
            j = noise.velocity_jitter(t, x, y, th, 0.08, 0.0, 1.2)
            if amp == 0:
                self.assertLess(max(j["sd_u1"], j["sd_u2"], j["sd_omega"]), 2e-3)
            else:
                self.assertAlmostEqual(j["sd_u1"] / (a / np.sqrt(2)), 1.0, delta=0.15)
                self.assertLess(j["sd_u2"], 2e-3)

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


def step_segments(rng, d_true, tau_true, D=0.0, pos_sd=2e-4, th_sd=0.003, fps=60.0, late=None, period=0.11):
    """Noisy poses of a bot whose twist follows a delayed first-order response to two-level step segments;
    heading diffusion D [rad^2/s] while it moves. late: (segment, level) pairs whose first frame the bot missed,
    so that change took effect one blaster period later; with late given, the levels carry the repeats' times."""
    truth = default_truth()
    segs = []
    for i, (a, b) in enumerate([((60, 60), (110, 110)), ((90, 0), (90, 90)), ((0, 90), (70, 100)),
                                ((110, 110), (60, 60)), ((70, 100), (100, 70)), ((80, 80), (0, 0))]):
        levels = [(0.0, *a), (1.5, *b), (3.0, 0, 0)]
        t_true = [lv[0] + period * ((i, j) in (late or ())) for j, lv in enumerate(levels)]
        if late is not None:
            levels = levels[:1] + [(t, L, R, (t + period, t + 2 * period)) for t, L, R in levels[1:]]
        dtf = 0.001
        tf = np.arange(-0.5, 4.0, dtf)
        ss = truth.predict(np.array([lv[1] for lv in levels]), np.array([lv[2] for lv in levels]))
        u = np.zeros((len(tf), 3))
        for k in range(3):
            u[:, k] = dynamics.step_response(tf, t_true, np.diff(np.r_[0, ss[:, k]]), d_true, tau_true)
        dth = u[:, 2] * dtf
        if D:
            dth = dth + np.sqrt(2 * D * dtf) * rng.normal(size=len(tf)) * (np.abs(u[:, 0]) > 1e-3)
        th = np.cumsum(dth) + rng.uniform(-3, 3)
        c, s = np.cos(th), np.sin(th)
        x = np.cumsum(c * u[:, 0] - s * u[:, 1]) * dtf
        y = np.cumsum(s * u[:, 0] + c * u[:, 1]) * dtf
        idx = np.arange(0, len(tf), int(1 / fps / dtf))
        segs.append(dynamics.DynSegment(tf[idx], x[idx] + rng.normal(0, pos_sd, len(idx)),
                                        y[idx] + rng.normal(0, pos_sd, len(idx)),
                                        th[idx] + rng.normal(0, th_sd, len(idx)), levels, truth))
    return segs


class TestDynamics(unittest.TestCase):
    def test_lag_and_dead_time(self):
        """Direct synthetic data: noisy poses of a bot whose twist follows a delayed first-order response."""
        d_true, tau_true = 0.04, 0.07
        r = dynamics.fit_lag(step_segments(np.random.default_rng(0), d_true, tau_true))
        for k in ("omega", "joint"):
            self.assertAlmostEqual(r[k]["dead_time"], d_true, delta=0.01, msg=k)
            self.assertAlmostEqual(r[k]["tau"], tau_true, delta=0.015, msg=k)

    def test_joint_fit_with_heading_diffusion(self):
        """Heading diffusion makes the omega series far noisier than u1. Normalising each output by its signal
        variance let omega dominate the joint fit (here d 38 ms, tau 56 ms); weighting by the residual variance
        keeps the joint fit as good as the u1 fit."""
        d_true, tau_true = 0.03, 0.06
        segs = step_segments(np.random.default_rng(0), d_true, tau_true, D=0.02, pos_sd=3e-4, th_sd=0.004)
        r = dynamics.fit_lag(segs)
        self.assertAlmostEqual(r["joint"]["dead_time"], d_true, delta=0.003)
        self.assertAlmostEqual(r["joint"]["tau"], tau_true, delta=0.004)
        for q in ("dead_time", "tau"):
            self.assertAlmostEqual(r["joint"][q], r["u1"][q], delta=0.0015, msg=q)

    def test_lost_frames_are_retimed(self):
        """Level changes and a STOP whose first frame the bot missed took effect at the repeat one period later.
        Fitted at the first frame they read as lag; fit_lag moves exactly those to the repeat."""
        d_true, tau_true, late = 0.03, 0.06, ((0, 1), (2, 2), (4, 1))
        segs = step_segments(np.random.default_rng(1), d_true, tau_true, late=late)
        r0 = dynamics.fit_lag(segs, max_reassign=0)
        self.assertGreater(r0["joint"]["tau"] - tau_true, 0.01)
        r = dynamics.fit_lag(segs)
        self.assertEqual(r["joint"]["late_changes"], len(late))
        for i, j in late:
            self.assertAlmostEqual(segs[i].t_steps[j] - segs[i].t_first[j], 0.11, msg=(i, j))
        self.assertAlmostEqual(r["joint"]["dead_time"], d_true, delta=0.005)
        self.assertAlmostEqual(r["joint"]["tau"], tau_true, delta=0.008)
        clean = step_segments(np.random.default_rng(1), d_true, tau_true, late=())
        self.assertEqual(dynamics.fit_lag(clean)["joint"]["late_changes"], 0)    # no loss: nothing moved

    def test_flip_and_battery_model(self):
        """A pi flip must not become a 2 pi / dt omega spike, and a 'bat' map must not crash DynSegment."""
        truth = default_truth()
        dt = 1 / 60
        t = np.arange(0, 3.0, dt)
        th = 0.3 + 1.5 * t
        x, y = 0.05 * np.cos(th), 0.05 * np.sin(th)
        th_flip = th.copy()
        th_flip[90] += np.pi
        levels = [(0.0, 90, 70), (2.5, 0, 0)]
        seg = dynamics.DynSegment(t, x, y, np.mod(th_flip + np.pi, 2 * np.pi) - np.pi, levels, truth)
        w = np.concatenate([r[1]["omega"] for r in seg.runs])
        self.assertLess(np.max(np.abs(w - 1.5)), 0.05)
        bat = fm.PowerLawMap(truth.p0, truth.gamma, np.c_[truth.coef, np.zeros((3, 2))], features="bat",
                             meta={"v_bat_median": 3.9})
        s2 = dynamics.DynSegment(t, x, y, th, levels, bat)          # used to raise "needs battery voltages"
        np.testing.assert_allclose(s2.ss["omega"], seg.ss["omega"], atol=1e-12)
        dynamics.DynSegment(t, x, y, th, levels, bat, v_bat=3.7)


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
    """extract.steady_window / abandoned: segments the bot stopped early are cut short and kept out of the lag fit."""

    def test_dropped_frames_at_the_end_are_not_a_stop(self):
        """Two dropped frames left tail runs shorter than the velocity filter, which used to read as a stop."""
        sd = drive_segment(drop_at=(2.68, 2.85))
        t0, t1, truncated = extract.steady_window(sd)
        self.assertIsNone(sd.stop)
        self.assertFalse(truncated)
        self.assertFalse(extract.abandoned(sd))

    def test_stop_early_in_the_window(self):
        """A trip 0.3 s into the steady window: the median speed of the first half was ~0 and missed it."""
        sd = drive_segment(stop_at=0.8)
        t0, t1, truncated = extract.steady_window(sd)
        self.assertTrue(truncated)
        self.assertAlmostEqual(sd.stop, sd.t_cmd + 0.8, delta=0.1)
        self.assertTrue(extract.abandoned(sd))
        for stop_at in (1.5, 2.4):
            sd = drive_segment(stop_at=stop_at, seed=1)
            extract.steady_window(sd)
            self.assertAlmostEqual(sd.stop, sd.t_cmd + stop_at, delta=0.1, msg=stop_at)


if __name__ == "__main__":
    unittest.main()
