"""End to end on the simulated rig: schedule -> scheduler -> tracker CSV -> analyze -> compare to truth.

Truth (sim.default_truth + EllipticalBot2D CoR offsets, camera latency 50 ms, IR decode RECORD_GAP_S + 0.5 ms
= 8.5 ms after the frame ends) is known exactly, so every fitted quantity can be checked.
"""
import dataclasses
import shutil
import tempfile
import unittest
import warnings
from pathlib import Path
from unittest import mock

import numpy as np

from graspion_calib import extract
from graspion_calib import schedules as S
from graspion_calib import sim
from graspion_calib.forward_map import PowerLawMap
from graspion_calib.report import analyze
from graspion_calib.scheduler import ANCHOR_GAP_S, ANCHOR_PINGS

from .simrig import BOT, run_session


class TestPipeline(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.dir = Path(tempfile.mkdtemp(prefix="graspion_pipe_"))
        cls.p = sim.SimParams()
        cls.world, _ = run_session(cls.dir / "grid", S.static_grid(blocks=2, seed=1) + S.static(5.0), params=cls.p,
                                   seed=1, place_home=True)
        cls.summary = analyze([cls.dir / "grid"], cls.dir / "out", bootstrap_n=3)

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
        expected = self.p.decode_delay - 0.002 / 2 + self.p.cam_latency   # decode - ack (SimBridge usb/2), + camera
        self.assertAlmostEqual(lat["cam_offset"], expected, delta=0.008)
        self.assertGreaterEqual(lat["found"] / lat["sent"], 0.95)

    def test_replicate_scatter_and_static_noise(self):
        rs = self.summary["replicate_scatter"]
        self.assertGreater(rs["cells"], 60)
        self.assertLess(rs["sd_median"][0], 0.006)               # replicate SD of u1 near the segment noise
        self.assertLess(rs["sd_median"][2], 0.15)
        st = self.summary["static_noise"]
        self.assertEqual(len(st), 1)
        self.assertLess(st[0]["sd_x"], 0.002)                    # the tracker's noise floor
        self.assertLess(st[0]["sd_theta"], 0.02)

    def test_inverse_map_written(self):
        inv = self.summary["inverse_map"]
        self.assertEqual(len(inv["polytope"]["h"]), 4)           # closed form: an exact parallelogram
        self.assertIn("pivots", inv)

    def test_icr_recorded(self):
        self.assertIn("cw", self.summary["icr"])


class TestFlashSync(unittest.TestCase):
    """Flash onsets and their matching to acknowledgements (synthetic traces), and the ping link test (sim)."""

    @classmethod
    def setUpClass(cls):
        cls.dir = Path(tempfile.mkdtemp(prefix="graspion_ping_"))
        cls.p = {}
        for loss, latency in ((0.0, 0.05), (0.3, 0.05), (0.3, 0.15)):       # 0.15 s: camera offset > the period
            p = dataclasses.replace(sim.SimParams(), ir_loss=loss, cam_latency=latency)
            name = f"ping{int(100 * loss)}_{int(1000 * latency)}"
            run_session(cls.dir / name, S.pings(20.0), params=p, seed=4)
            cls.p[name] = p

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.dir, ignore_errors=True)

    def test_onsets_when_mostly_lit_and_nan(self):
        rng = np.random.default_rng(0)
        t = np.arange(0, 20, 1 / 60)
        starts = np.arange(1.0, 19.0, 0.11)                       # 70 ms flashes every 110 ms (ping test)
        lit = np.zeros(len(t), bool)
        for s in starts:
            lit |= (t >= s) & (t < s + 0.07)
        self.assertGreater(lit.mean(), 0.5)                       # the old median baseline sat on the lit level
        led = 30 + 120 * lit + rng.normal(0, 3, len(t))
        on = extract.flash_onsets(t, led)
        self.assertEqual(len(on), len(starts))
        self.assertLess(np.max(np.abs(on - starts)), 1 / 60 + 1e-9)
        k = int(np.flatnonzero(lit[1:-1] & lit[:-2] & lit[2:])[10]) + 1   # mid-flash frame: one NaN
        led[k] = np.nan                                           # (LED patch outside the image) used to
        np.testing.assert_array_equal(extract.flash_onsets(t, led), on)   # wipe out every flash

    def test_match_is_one_to_one_and_gated(self):
        """A frame the bot never decoded used to inherit the next frame's flash: 60 % loss read as ~5-15 %."""
        rng = np.random.default_rng(1)
        n, P = 300, 0.11
        acks = 1.0 + np.arange(n) * P + rng.uniform(0, 0.002, n)
        for d in (0.06, 0.16):                                    # camera offset below and above the period
            for loss in (0.0, 0.3, 0.6):
                dec = rng.random(n) >= loss
                dec[[0, -1]] = True     # a lost first / last frame makes the offset ambiguous by one period
                onsets = acks[dec] + d + rng.uniform(0, 1 / 60, dec.sum())
                rep = {}
                lags = extract.match_flashes(acks, onsets, report=rep)
                np.testing.assert_array_equal(np.isfinite(lags), dec, err_msg=f"d={d} loss={loss}")
                self.assertAlmostEqual(np.median(lags[dec]), d + 1 / 120, delta=0.006)
                self.assertLess(np.max(lags[dec]), d + 1 / 60 + 1e-9)
                self.assertFalse(rep["ambiguous"])
        # sparse SEG acknowledgements with a stray onset in between
        acks = 5.0 * np.arange(1, 21)
        onsets = np.r_[acks + 0.07, 33.0]
        np.testing.assert_allclose(extract.match_flashes(acks, onsets), 0.07)

    def test_lag_above_the_period(self):
        """Camera offset above the period and the last PING lost: in a plain train the lag and the lag minus one
        period match equally many PINGs, and the smaller one used to win silently, so every PING took the previous
        one's flash. That is now reported; the anchor PINGs that start a ping run (scheduler.run_ping) resolve it."""
        rng = np.random.default_rng(2)
        n, period, d = 200, 0.11, 0.16
        gap = ANCHOR_GAP_S + 0.07                                   # ack to ack: the gap plus one frame
        for anchors in (0, ANCHOR_PINGS):
            with self.subTest(anchors=anchors):
                acks = np.r_[1.0 - gap * np.arange(anchors, 0, -1), 1.0 + np.arange(n) * period]
                acks += rng.uniform(0, 0.002, len(acks))
                dec = rng.random(len(acks)) >= 0.3
                dec[0], dec[-1] = True, False
                onsets = acks[dec] + d + rng.uniform(0, 1 / 60, dec.sum())
                rep = {}
                lags = extract.match_flashes(acks, onsets, report=rep)
                if anchors:
                    self.assertFalse(rep["ambiguous"])
                    np.testing.assert_array_equal(np.isfinite(lags), dec)
                    self.assertAlmostEqual(np.median(lags[dec]), d + 1 / 120, delta=0.006)
                else:
                    self.assertTrue(rep["ambiguous"])
                    self.assertAlmostEqual(np.nanmedian(lags), d - period + 1 / 120, delta=0.006)   # the wrong one
                    self.assertTrue(any(abs(x - d) < 1 / 60 for x in rep["rival_lags"]), rep)

    def test_ping_session_link_stats(self):
        for name, p in self.p.items():
            sess = extract.load_session(self.dir / name)
            st = extract.link_stats(sess)
            self.assertGreater(st["pings"], 150)
            self.assertAlmostEqual(st["loss"], p.ir_loss, delta=0.07, msg=f"{name}: {st}")
            t_ping = sess.frames["t_ack"][sess.frames["op"] == "PING"]
            self.assertTrue(np.all(np.diff(t_ping[:ANCHOR_PINGS + 1]) > ANCHOR_GAP_S))   # the anchors
            self.assertFalse(st["offset_ambiguous"], name)
            offset = p.decode_delay - 0.002 / 2 + p.cam_latency    # flash - ack (SimBridge usb/2), + camera latency
            frame = 1 / p.cam_fps
            # raw lag = offset + the wait for the first exposure showing the flash, in [0, 1 frame). A 110 ms ping
            # train meets the 60 fps camera at only 5 phases, so the median is not offset + half a frame here;
            # the flash may show up to one 2 ms simulator step early
            p5, p50, p95 = st["offset_p5_p50_p95"]
            self.assertGreater(p5, offset - 0.002, name)
            self.assertLess(p95, offset + frame, name)
            self.assertTrue(offset < p50 < offset + frame, (name, p50, offset))
            off = extract.command_offset(sess)                     # median raw lag - half a frame
            self.assertAlmostEqual(off["cam_offset"], offset, delta=frame / 2, msg=name)
            self.assertNotIn("warning", off)


class TestNoFlash(unittest.TestCase):
    """A camera that never shows the sync flash (covered LED, wrong LED patch)."""

    @classmethod
    def setUpClass(cls):
        cls.dir = Path(tempfile.mkdtemp(prefix="graspion_noflash_"))
        cls.p = sim.SimParams()
        cells = [(70, 70), (90, 90), (110, 110), (70, 100), (100, 70), (120, 90), (90, 120), (80, 110), (110, 80),
                 (100, 100)]
        specs = [S.drive(L, R) for L, R in cells]
        run_session(cls.dir / "colour", specs, params=cls.p, seed=5)
        dark = dataclasses.replace(cls.p, led_flash=0.0)
        run_session(cls.dir / "dark", specs, params=dark, seed=6)

    @classmethod
    def tearDownClass(cls):
        shutil.rmtree(cls.dir, ignore_errors=True)

    def test_offset_from_other_sessions(self):
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("always")
            s = analyze([self.dir / "colour", self.dir / "dark"], self.dir / "out_both", bootstrap_n=0)
        self.assertTrue(any("no sync flash" in str(x.message) for x in w))
        lat = s["latency"]
        self.assertEqual(lat[0]["cam_offset_source"], "measured")
        self.assertEqual(lat[1]["cam_offset_source"], "other_sessions")
        self.assertEqual(lat[1]["cam_offset_used"], lat[0]["cam_offset"])
        self.assertIn("warning", lat[1])

    def test_ambiguous_offset_is_not_lent(self):
        """A cam_offset that may be one frame spacing off (extract.match_flashes) is used by its own session only."""
        real = extract.command_offset

        def flagged(sess, **kw):
            out = real(sess, **kw)
            out["ambiguous"] = out["ambiguous"] or sess.path.name == "colour"
            return out

        with mock.patch.object(extract, "command_offset", flagged), warnings.catch_warnings():
            warnings.simplefilter("ignore")
            s = analyze([self.dir / "colour", self.dir / "dark"], self.dir / "out_amb", bootstrap_n=0)
        lat = s["latency"]
        self.assertTrue(lat[0]["ambiguous"])
        self.assertEqual(lat[0]["cam_offset_source"], "measured")
        self.assertEqual(lat[1]["cam_offset_source"], "none")

    def test_no_offset_anywhere_is_reported(self):
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("always")
            s = analyze([self.dir / "dark"], self.dir / "out_dark", bootstrap_n=0)
        self.assertTrue(any("no sync flash" in str(x.message) for x in w))
        self.assertEqual(s["latency"][0]["cam_offset_source"], "none")
        self.assertTrue(any("no sync flash" in n for n in s["warnings"]))
        self.assertGreaterEqual(s["n_segments_fit"], 8)         # the map is still fitted, on acknowledgement time


if __name__ == "__main__":
    unittest.main()
