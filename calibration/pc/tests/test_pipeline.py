"""End to end on the simulated rig: schedule -> scheduler -> tracker CSV + bot dump -> analyze -> compare to truth.

Truth (sim.default_truth + EllipticalBot2D CoR offsets, lag 60 ms, dead time 30 ms, camera latency 50 ms,
IR decode 5 ms after the frame) is known exactly, so every fitted quantity can be checked.
"""
import json
import shutil
import tempfile
import unittest
from pathlib import Path

import numpy as np

from graspion_calib import schedules as S
from graspion_calib import sim
from graspion_calib.forward_map import PowerLawMap
from graspion_calib.report import analyze

from .simrig import BOT, run_session


class TestPipeline(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.dir = Path(tempfile.mkdtemp(prefix="graspion_pipe_"))
        cls.p = sim.SimParams()
        cls.world, _ = run_session(cls.dir / "grid", S.static_grid(blocks=2, seed=1), params=cls.p, seed=1)
        steps = S.steps([((60, 60), (110, 110)), ((90, 0), (90, 90)), ((0, 90), (70, 100)), ((80, 80), (110, 60))])
        run_session(cls.dir / "steps", steps, params=cls.p, seed=2)
        cls.summary = analyze([cls.dir / "grid", cls.dir / "steps"], cls.dir / "out", bootstrap_n=3)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.dir, ignore_errors=True)

    def test_outputs_written(self):
        for f in ("segments_fit.csv", "forward_map.json", "forward_map_bootstrap.json", "inverse_map.json", "calib.json"):
            self.assertTrue((self.dir / "out" / f).exists(), f)
        self.assertGreater(self.summary["n_segments_fit"], 150)

    def test_forward_map_matches_truth(self):
        fwd = PowerLawMap.load(self.dir / "out" / "forward_map.json")
        plant = self.world.plants[BOT]
        g = np.linspace(60, 120, 7)
        err = []
        for L in g:
            for R in g:
                u1, u2, w = plant.truth_twist_com(L, R)          # reference point = CoM in the simulator
                err.append(fwd.predict([L], [R])[0] - (u1, u2, w))
        err = np.array(err)
        rmse = np.sqrt(np.mean(err ** 2, axis=0))
        self.assertLess(rmse[0], 0.004, f"u1 rmse {rmse[0]}")   # m/s
        self.assertLess(rmse[1], 0.003, f"u2 rmse {rmse[1]}")
        self.assertLess(rmse[2], 0.12, f"omega rmse {rmse[2]}")  # rad/s
        for L, R in ((90, 0), (0, 90)):                          # single-motor pivots are in the grid too
            u1, u2, w = plant.truth_twist_com(L, R)
            self.assertLess(abs(fwd.predict([L], [R])[0][2] - w), 0.25)

    def test_cor_offset(self):
        meta = self.summary["forward_map"]["meta"]
        self.assertEqual(meta["selected"], "cor")                # EllipticalBot2D kinematics: u1 = v + d|w|
        self.assertAlmostEqual(meta["cor_offset_m"], float(np.mean(self.p.d_cor)), delta=0.0015)

    def test_latency_offset(self):
        lat = self.summary["latency"][0]
        expected = 0.004 + self.p.cam_latency                    # decode - ack, + camera latency
        self.assertAlmostEqual(lat["cam_offset"], expected, delta=0.008)
        self.assertGreaterEqual(lat["found"] / lat["sent"], 0.95)

    def test_lag_and_dead_time(self):
        d = self.summary["dynamics"]["joint"]
        self.assertAlmostEqual(d["dead_time"], self.p.dead, delta=0.015)
        self.assertAlmostEqual(d["tau"], self.p.tau, delta=0.025)

    def test_heading_diffusion(self):
        n = self.summary["noise"]["heading_segments"]
        self.assertAlmostEqual(n["D_r"] / self.p.D_r, 1.0, delta=0.35)

    def test_bot_log_merge(self):
        m = self.summary["bot_log_merge"][0]
        self.assertGreater(m["matched"], 0.95 * m["bot_seg_records"])
        self.assertAlmostEqual(m["clock_rate"], 1 / (1 + self.p.clock_err), delta=0.002)

    def test_inverse_map_round_trip(self):
        inv = json.loads((self.dir / "out" / "inverse_map.json").read_text())
        self.assertEqual(len(inv["polytope"]["h"]), 4 if "closed_form" in inv else len(inv["polytope"]["h"]))
        self.assertIn("pivots", inv)

    def test_icr_recorded(self):
        self.assertIn("cw", self.summary["icr"])


if __name__ == "__main__":
    unittest.main()
