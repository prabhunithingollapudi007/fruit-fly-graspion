"""Offline tests of popsim against known truth:  python calibration_paper/popsim.py selftest"""
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import popsim as ps  # noqa: E402


class TestPopulation(unittest.TestCase):
    def test_zero_spread_is_the_mean(self):
        robots = ps.draw_population(3, n_days=2, spread=0.0, seed=1)
        mean = ps.population_mean()
        for r in robots:
            for m in r["days"]:
                np.testing.assert_allclose(ps.theta_of(m), ps.theta_of(mean), atol=1e-12)

    def test_theta_roundtrip_cor(self):
        m = ps.population_mean("cor")
        back = ps.map_of(ps.theta_of(m), "cor")
        np.testing.assert_allclose(back.coef, m.coef)
        L, R, Y = ps.truth_on_grid(m)
        np.testing.assert_allclose(back.predict(L, R), Y)


class TestPrior(unittest.TestCase):
    def test_single_source_floor_and_mean(self):
        m = ps.population_mean()
        m.meta["rmse"] = list(ps.NOISE)
        pr = ps.Prior.from_maps([m])
        np.testing.assert_allclose(pr.mu, ps.theta_of(m))
        np.testing.assert_allclose(np.sqrt(np.diag(pr.cov_nl)), ps.NL_FLOOR)
        np.testing.assert_allclose(pr.noise, ps.NOISE)

    def test_many_sources_recover_spread(self):
        robots = ps.draw_population(40, spread=1.0, seed=3)
        maps = [r["days"][0] for r in robots]
        pr = ps.Prior.from_maps(maps, shrink=0.0)
        T = np.stack([ps.theta_of(m) for m in maps])
        np.testing.assert_allclose(pr.mu, T.mean(axis=0))
        # p0 SD of the draws is about Spread.p0; the prior sees it plus the floor / sqrt(40)
        sd = np.sqrt(pr.cov_nl[0, 0])
        self.assertGreater(sd, 0.6 * ps.Spread().p0)
        self.assertLess(sd, 1.6 * ps.Spread().p0)


class TestFits(unittest.TestCase):
    def setUp(self):
        self.robots = ps.draw_population(6, spread=1.0, seed=5)
        self.maps = [r["days"][0] for r in self.robots]
        for m in self.maps:
            m.meta["rmse"] = list(ps.NOISE)
        self.prior = ps.Prior.from_maps(self.maps[:5])
        self.target = self.maps[5]
        self.L, self.R, self.Y = ps.segments(self.target, blocks=5, seed=11)
        self.Le, self.Re, self.Ye = ps.truth_on_grid(self.target)

    def test_k0_is_prior_mean(self):
        m = ps.fit_map(self.L[:0], self.R[:0], self.Y[:0], self.prior)
        np.testing.assert_allclose(ps.theta_of(m), self.prior.mu)

    def test_more_data_beats_prior_and_approaches_full(self):
        s_ref = ps.score(self.prior.mean_map(), self.Le, self.Re, self.Ye)[1]
        rng = np.random.default_rng(0)
        idx = ps.pick_random(self.L, self.R, 32, rng)
        s32 = ps.score(ps.fit_map(self.L[idx], self.R[idx], self.Y[idx], self.prior), self.Le, self.Re, self.Ye)[1]
        s_all = ps.score(ps.fit_map(self.L, self.R, self.Y, self.prior), self.Le, self.Re, self.Ye)[1]
        s_full = ps.score(ps.full_fit(self.L, self.R, self.Y), self.Le, self.Re, self.Ye)[1]
        self.assertLess(s32, s_ref)
        self.assertLess(s_all, s32)
        self.assertLess(abs(s_all - s_full), 0.5 * max(s_full, 0.05))

    def test_predictive_sd_shrinks_with_data(self):
        rng = np.random.default_rng(1)
        idx = ps.pick_random(self.L, self.R, 16, rng)
        sd0 = ps.predictive_sd(ps.fit_map(self.L[:0], self.R[:0], self.Y[:0], self.prior), self.Le, self.Re)
        sd16 = ps.predictive_sd(ps.fit_map(self.L[idx], self.R[idx], self.Y[idx], self.prior), self.Le, self.Re)
        self.assertLess(np.mean(sd16), np.mean(sd0))

    def test_affine_baseline_exact_on_scaled_reference(self):
        ref = self.prior.mean_map()
        Yr = ref.predict(self.L, self.R) * np.array([1.2, 1.0, 0.8]) + np.array([0.0, 0.0, -0.1])
        m = ps.AffineOnRef.fit(self.L, self.R, Yr, ref)
        np.testing.assert_allclose(m.a, [1.2, 1.0, 0.8], atol=1e-9)
        np.testing.assert_allclose(m.b, [0.0, 0.0, -0.1], atol=1e-9)

    def test_scratch_refuses_tiny_sets(self):
        self.assertIsNone(ps.fit_scratch(self.L[:4], self.R[:4], self.Y[:4]))

    def test_cor_prior_fit_runs(self):
        robots = ps.draw_population(3, spread=0.5, seed=9, features="cor")
        maps = [r["days"][0] for r in robots]
        for m in maps:
            m.meta["rmse"] = list(ps.NOISE)
        pr = ps.Prior.from_maps(maps[:2])
        L, R, Y = ps.segments(maps[2], blocks=2, seed=4)
        m = ps.fit_map(L, R, Y, pr)
        self.assertEqual(m.features, "cor")
        self.assertLess(ps.score(m, *ps.truth_on_grid(maps[2]))[1],
                        ps.score(pr.mean_map(), *ps.truth_on_grid(maps[2]))[1])


class TestDesign(unittest.TestCase):
    def test_dopt_beats_random_information(self):
        robots = ps.draw_population(4, spread=1.0, seed=7)
        maps = [r["days"][0] for r in robots]
        for m in maps:
            m.meta["rmse"] = list(ps.NOISE)
        prior = ps.Prior.from_maps(maps[:3])
        L, R, Y = ps.segments(maps[3], blocks=5, seed=2)
        rng = np.random.default_rng(0)
        d_idx = ps.pick_dopt(L, R, 12, prior, rng)
        self.assertEqual(len(d_idx), 12)
        cells = list(ps._pool_by_cell(L, R))
        J = ps.twist_jacobian(prior, cells)
        Rinv = np.diag(1.0 / prior.noise ** 2)
        info = np.einsum("coi,op,cpj->cij", J, Rinv, J)
        cell_of = {i: c for c, idx in ps._pool_by_cell(L, R).items() for i in idx}
        pos = {c: i for i, c in enumerate(cells)}

        def logdet(idx):
            Lam = prior.full_precision() + sum(info[pos[cell_of[i]]] for i in idx)
            return np.linalg.slogdet(Lam)[1]

        rand = [logdet(ps.pick_random(L, R, 12, np.random.default_rng(s))) for s in range(20)]
        self.assertGreater(logdet(d_idx), max(rand))
        # the chosen cells are not all in the dead zone and not all one cell
        chosen = {cell_of[i] for i in d_idx}
        self.assertGreater(len(chosen), 2)
        self.assertTrue(any(c != (0.0, 0.0) for c in chosen))


class TestVariance(unittest.TestCase):
    def test_components_match_truth(self):
        robots, sessions, _, _ = ps.simulate(12, n_days=3, spread=1.0, blocks=4, seed=21)
        est = ps.variance_components(sessions)
        tru = ps.true_components(robots)
        for o in (0, 2):                           # u1 and omega; u2 is nearly zero in the synthetic truth
            self.assertLess(abs(est["sd"]["robot"][o] - tru["robot"][o]), 0.35 * tru["robot"][o])
            self.assertLess(abs(est["sd"]["day"][o] - tru["day"][o]), 0.35 * tru["day"][o])
            self.assertLess(abs(est["sd"]["segment"][o] - ps.NOISE[o]), 0.15 * ps.NOISE[o])

    def test_zero_spread_gives_no_robot_component(self):
        robots, sessions, _, _ = ps.simulate(5, n_days=2, spread=0.0, blocks=4, seed=22)
        est = ps.variance_components(sessions)
        for o in range(3):
            self.assertLess(est["fraction"]["robot"][o], 0.1)
            self.assertGreater(est["fraction"]["segment"][o], 0.8)


class TestLoro(unittest.TestCase):
    def test_quick_study_orders_methods(self):
        robots, sessions, pools, evals = ps.simulate(4, spread=1.0, blocks=3, seed=31)
        out = ps.loro(pools, evals, prior_sizes=(1, 3), ks=(0, 8, 32), designs=("random",), reps=1, quick=True)
        s = ps.summarize_loro(out)
        hier = s["hier"][3]["random"]
        self.assertLess(hier[32][0], hier[0][0])                        # data help
        self.assertLessEqual(hier[8][0], s["scratch"][0]["random"][8][0] * 1.05)   # prior helps at small k
        self.assertEqual(s["ref"][3]["random"][0][0], hier[0][0])      # k = 0: hier is the reference
        ps.print_loro(out)


if __name__ == "__main__":
    unittest.main()
