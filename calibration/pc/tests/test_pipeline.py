"""End to end on the simulated rig: schedule -> scheduler -> tracker CSV + bot dump -> analyze -> compare to truth.

Truth (sim.default_truth + EllipticalBot2D CoR offsets, lag 60 ms, dead time 30 ms, camera latency 50 ms,
IR decode RECORD_GAP_S + 0.5 ms = 8.5 ms after the frame ends) is known exactly, so every fitted quantity can be checked.
"""
import dataclasses
import json
import shutil
import tempfile
import unittest
import warnings
from pathlib import Path
from unittest import mock

import numpy as np

from graspion_calib import botlog, extract, forward_map, noise, report
from graspion_calib import schedules as S
from graspion_calib import sim
from graspion_calib.forward_map import PowerLawMap
from graspion_calib.geometry import wrap
from graspion_calib.inverse_map import InverseMap
from graspion_calib.report import analyze
from graspion_calib.scheduler import ANCHOR_GAP_S, ANCHOR_PINGS

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
        expected = self.p.decode_delay - 0.002 / 2 + self.p.cam_latency   # decode - ack (SimBridge usb/2), + camera
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

    def test_battery_variant_selected(self):
        """'bat' winning the CV used to crash analyze in DynSegment (no calib.json) and InverseMap (validation)."""
        orig = forward_map.select_and_fit

        def force_bat(L, R, Y, V=None, **kw):
            self.assertIsNotNone(V)                              # every usable row has a rest voltage
            return orig(L, R, Y, V=V, variants=("bat",), **kw)

        with mock.patch.object(forward_map, "select_and_fit", force_bat):
            s = analyze([self.dir / "grid", self.dir / "steps"], self.dir / "out_bat", bootstrap_n=2)
        self.assertEqual(s["forward_map"]["features"], "bat")
        self.assertTrue((self.dir / "out_bat" / "calib.json").exists())
        self.assertIn("joint", s["dynamics"])
        self.assertIsNotNone(s["inverse_map"])
        inv = InverseMap(PowerLawMap.load(self.dir / "out_bat" / "forward_map.json"))   # schedule validation
        self.assertAlmostEqual(inv.v_bat, s["forward_map"]["meta"]["v_bat_median"])


class TestFlashSync(unittest.TestCase):
    """Flash onsets and their matching to acknowledgements (synthetic traces), and the ping link test (sim)."""

    @classmethod
    def setUpClass(cls):
        cls.dir = Path(tempfile.mkdtemp(prefix="graspion_ping_"))
        cls.p = {}
        for loss, latency in ((0.0, 0.05), (0.3, 0.05), (0.3, 0.15)):       # 0.15 s: camera offset > the period
            p = dataclasses.replace(sim.SimParams(), ir_loss=loss, cam_latency=latency)
            name = f"ping{int(100 * loss)}_{int(1000 * latency)}"
            run_session(cls.dir / name, S.pings(20.0), params=p, seed=4, write_dump=False)
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


def _bot_session(rng, t0, nblk=1, rate=1.012, cells=None):
    cells = cells or [(a, b) for a in S.GRID for b in S.GRID if a or b]
    sched = []
    for _ in range(nblk):
        sched += [cells[i] for i in rng.permutation(len(cells))]
    acks = t0 + np.cumsum(rng.uniform(6, 9, len(sched)))
    recs = [{"kind": 1, "L": L, "R": R, "t_start_ms": int((a - 990.0) * rate * 1000 + rng.normal(0, 15))}
            for (L, R), a in zip(sched, acks)]
    return sched, acks, recs


class TestBotLog(unittest.TestCase):
    """Alignment of the PC's SEG frames with the bot's own log."""

    def _correct(self, pairs, offset=0, skip_pc=0):
        return sum(1 for i, b in pairs if b - offset == i - skip_pc)

    def test_leftover_records_and_missed_first_seg(self):
        rng = np.random.default_rng(0)
        sched, acks, recs = _bot_session(rng, 1000.0, nblk=2)
        n = len(sched)
        self.assertEqual(self._correct(botlog.match_records(acks, sched, recs)), n)
        for nb in (1, 3, 5):                                    # bench checks 'X <id> 1 80 80' left in the log
            bench = [{"kind": 1, "L": 80, "R": 80, "t_start_ms": 5000 + 3000 * k} for k in range(nb)]
            pairs = botlog.match_records(acks, sched, bench + recs)
            self.assertEqual(self._correct(pairs, offset=nb), n, f"{nb} leftover records")
            self.assertEqual(len(pairs), n)
        for gap in (1.2, 1.7, 3.0):                             # first SEG lost, retried with the same (L, R)
            pairs = botlog.match_records(np.r_[acks[0] - gap, acks], [sched[0]] + sched, recs)
            self.assertEqual(len(pairs), n, f"retry gap {gap}")
            self.assertEqual(self._correct(pairs, skip_pc=1), n)

    def test_shared_dump_and_ambiguity(self):
        rng = np.random.default_rng(1)
        s1, a1, r1 = _bot_session(rng, 1000.0)
        s2, a2, r2 = _bot_session(rng, a1[-1] + 120.0)
        pairs = botlog.match_records(a2, s2, r1 + r2)           # second session on one battery, one dump
        self.assertEqual(self._correct(pairs, offset=len(r1)), len(s2))
        pairs = botlog.match_records(a1, s1, r1 + r2)
        self.assertEqual(self._correct(pairs), len(s1))
        # battery swapped mid-session: the bot reset, so the dump holds only the later records, on a new clock
        half = len(s1) // 2
        reset = [dict(r, t_start_ms=r["t_start_ms"] - r1[half]["t_start_ms"] + 500) for r in r1[half:]]
        pairs = botlog.match_records(a1, s1, reset)
        self.assertEqual(self._correct(pairs, offset=-half), len(s1) - half)
        # the same schedule run twice with identical timing: cannot be told apart, so nothing is matched
        twin = [dict(r, t_start_ms=r["t_start_ms"] + 600000) for r in r1]
        self.assertEqual(botlog.match_records(a1, s1, r1 + twin), [])

    def test_merge_uses_rest_voltage(self):
        rng = np.random.default_rng(2)
        sched, acks, recs = _bot_session(rng, 1000.0, nblk=3, cells=[(80, 80), (100, 60), (60, 100), (90, 90)])
        for k, r in enumerate(recs):
            r.update({"mv_rest": 3900 - k, "mv_load_mean": 3650 - 2 * k, "mv_load_min": 3600, "end_reason": 3,
                      "t_end_ms": r["t_start_ms"] + 3000})
        recs[1]["mv_rest"] = 0                                  # battery read failed
        frames = {"op": np.array(["SEG"] * len(acks), dtype=object), "t_ack": acks,
                  "L": np.array([c[0] for c in sched], float), "R": np.array([c[1] for c in sched], float),
                  "seg": np.arange(len(acks), dtype=float)}
        rows = [{"seg": k, "attempt": 0} for k in range(len(acks))]
        info = botlog.merge(rows, frames, recs)
        self.assertEqual(info["matched"], len(acks))
        self.assertAlmostEqual(rows[0]["V_bat"], 3.9)           # rest, not the drive-dependent load voltage
        self.assertNotIn("V_bat", rows[1])
        self.assertEqual(rows[2]["bot_end"], "stop")
        with self.assertWarns(UserWarning):                     # a poor match is reported, not silent
            botlog.merge([], frames, [dict(r, L=0) for r in recs])


def synthetic_segment(seg, omega, stop_at=None, D=0.01, T=3.0, fps=60.0, u1=0.08, tau=0.06, seed=0):
    """A drive segment whose bot runs at (u1, omega) from t_cmd and stops at t_cmd + stop_at (watchdog trip)
    or at the STOP; first-order lag tau, heading diffusion D while moving, tracker noise."""
    rng = np.random.default_rng(seed)
    t_cmd = 20.0 * seg + 5.0
    t_off = t_cmd + (T if stop_at is None else stop_at)
    t = np.arange(t_cmd - 0.5, t_cmd + T + 1.0, 1 / fps)
    g = np.where(t < t_cmd, 0.0, 1 - np.exp(-(t - t_cmd) / tau))
    g_off = 1 - np.exp(-(t_off - t_cmd) / tau)
    g = np.where(t < t_off, g, g_off * np.exp(-(t - t_off) / tau))
    th = 0.3 + np.cumsum(omega * g / fps + np.sqrt(2 * D / fps) * rng.normal(size=len(t)) * (g > 0.01))
    x = np.cumsum(u1 * g * np.cos(th)) / fps + rng.normal(0, 3e-4, len(t))
    y = np.cumsum(u1 * g * np.sin(th)) / fps + rng.normal(0, 3e-4, len(t))
    levels = [(t_cmd, 80, 80), (t_cmd + T, 0, 0)]
    return extract.SegmentData(seg, 0, "drive", "grid", 0, 80, 80, "ok", True, 0.05, t_cmd, t_cmd + T, levels,
                               t, x, y, wrap(th + rng.normal(0, 0.004, len(t))))


def synthetic_steps(seg, fwd, pairs, hold=1.5, stop_at=None, fps=60.0, tau=0.06, seed=0):
    """A step segment: each (L, R) in pairs held for `hold` s at the twist fwd predicts, reached with first-order lag
    tau; the bot stops at t_cmd + stop_at (watchdog trip). Tracker noise as in synthetic_segment."""
    rng = np.random.default_rng(seed)
    t_cmd = 20.0 * seg + 5.0
    T = hold * len(pairs)
    t = np.arange(t_cmd - 0.5, t_cmd + T + 1.0, 1 / fps)
    levels = [(t_cmd + k * hold, L, R, ()) for k, (L, R) in enumerate(pairs)] + [(t_cmd + T, 0, 0, ())]
    target = np.zeros((len(t), 3))
    for t_k, L, R, _ in levels:
        target[t >= t_k] = fwd.predict([L], [R])[0]
    if stop_at is not None:
        target[t >= t_cmd + stop_at] = 0.0
    tw = np.zeros_like(target)
    a = 1 - np.exp(-1 / (fps * tau))
    for i in range(1, len(t)):
        tw[i] = tw[i - 1] + a * (target[i] - tw[i - 1])
    th = 0.3 + np.cumsum(tw[:, 2]) / fps
    x = np.cumsum(tw[:, 0] * np.cos(th) - tw[:, 1] * np.sin(th)) / fps + rng.normal(0, 3e-4, len(t))
    y = np.cumsum(tw[:, 0] * np.sin(th) + tw[:, 1] * np.cos(th)) / fps + rng.normal(0, 3e-4, len(t))
    return extract.SegmentData(seg, 0, "steps", "step", 0, *pairs[0], "ok", True, 0.05, t_cmd, t_cmd + T, levels,
                               t, x, y, wrap(th + rng.normal(0, 0.004, len(t))))


class TestAbandonedSegments(unittest.TestCase):
    """Segments the bot stopped early (command watchdog) are still logged 'ok' by the PC."""

    def test_step_down_to_a_slow_level_is_not_a_stop(self):
        """A bot whose dead zone ends near 60: the step (100, 100) -> (60, 60) falls below a quarter of the peak
        speed, which the visible-stop test used to take for a stop, dropping valid lag-fit segments. A trip while
        a level that should move the bot is commanded is still one."""
        for p0 in (58.0, 59.5, 60.0):
            fwd = dataclasses.replace(sim.default_truth(), p0=(p0, p0))
            for k, (pairs, stop_at) in enumerate(((((100, 100), (60, 60)), None), (((100, 100), (60, 60)), 0.9),
                                                  (((60, 60), (100, 100)), 2.2), (((80, 0), (100, 100)), None))):
                with self.subTest(p0=p0, pairs=pairs, stop_at=stop_at):
                    sd = synthetic_steps(k, fwd, pairs, stop_at=stop_at, seed=k)
                    if p0 == 60.0 and pairs[1] == (60, 60) and stop_at is None:   # the old rule: against the
                        self.assertIsNotNone(extract.detect_early_stop(sd, sd.t_cmd + 0.5, sd.t_end))   # peak speed
                    self.assertEqual(extract.abandoned(sd, fwd=fwd), stop_at is not None)
                    if stop_at is not None:
                        self.assertAlmostEqual(sd.stop, sd.t_cmd + stop_at, delta=0.1)
                    self.assertFalse(extract.abandoned(sd))             # without a map only the bot log counts
        sd = synthetic_steps(9, fwd, ((100, 100), (60, 60)), stop_at=0.9)
        sd.bot_end = "watchdog"
        self.assertTrue(extract.abandoned(sd))

    def test_heading_diffusion_ignores_stopped_tail(self):
        rng = np.random.default_rng(5)
        segs = [synthetic_segment(k, rng.uniform(-3, 3), stop_at=1.5 if k % 2 else None, seed=k) for k in range(60)]
        heading, _, _ = report.noise_runs(segs)
        d_fixed = noise.heading_diffusion(heading, max_lag_s=0.5)["D_r"]
        naive = [(s.t[m], s.th[m]) for s in segs for m in [(s.t >= s.t_cmd + 0.5) & (s.t <= s.t_end)]]
        d_naive = noise.heading_diffusion(naive, max_lag_s=0.5)["D_r"]
        self.assertAlmostEqual(d_fixed / 0.01, 1.0, delta=0.2)
        self.assertGreater(d_naive / d_fixed, 1.15)             # the full window is visibly biased
        self.assertTrue(all(s.stop is not None for s in segs[1::2]))
        self.assertTrue(all(s.stop is None for s in segs[0::2]))

    def test_lag_pool_skips_abandoned(self):
        segs = [synthetic_segment(k, 1.0, stop_at=1.5 if k < 2 else None, seed=k) for k in range(6)]
        late = synthetic_segment(6, 1.0, stop_at=2.85, seed=6)  # trip too late to see: only the bot log knows
        segs.append(late)
        self.assertFalse(extract.abandoned(late))
        info = {"clock_rate": 1.0}
        rec = {"kind": 1, "end_reason": 4, "t_start_ms": 1000, "t_end_ms": 3850}
        report.mark_bot_ends(segs, (info, {(6, 0): (rec, late.t_cmd - 0.05)}))
        self.assertEqual(late.bot_end, "watchdog")
        self.assertAlmostEqual(late.t_trip, late.t_cmd + 2.85)
        t0, t1, truncated = extract.steady_window(late)
        self.assertTrue(truncated)
        self.assertLessEqual(t1, late.t_trip)
        dyn, skipped = report.dynamics_segments([segs], sim.default_truth())
        self.assertEqual(skipped, 3)
        self.assertEqual(len(dyn), 4)


class TestNoFlash(unittest.TestCase):
    """A camera that never shows the sync flash (mono camera without the luma channel, covered LED)."""

    @classmethod
    def setUpClass(cls):
        cls.dir = Path(tempfile.mkdtemp(prefix="graspion_noflash_"))
        cls.p = sim.SimParams()
        cells = [(70, 70), (90, 90), (110, 110), (70, 100), (100, 70), (120, 90), (90, 120), (80, 110), (110, 80),
                 (100, 100)]
        specs = [S.drive(L, R) for L, R in cells] + S.steps([((60, 60), (110, 110)), ((70, 100), (100, 70)),
                                                              ((80, 80), (110, 60))])
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
        self.assertFalse(s["dead_time_includes_cam_offset"])

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

    def test_no_offset_anywhere_is_reported_as_total_delay(self):
        with warnings.catch_warnings(record=True) as w:
            warnings.simplefilter("always")
            s = analyze([self.dir / "dark"], self.dir / "out_dark", bootstrap_n=0)
        self.assertTrue(any("no sync flash" in str(x.message) for x in w))
        self.assertEqual(s["latency"][0]["cam_offset_source"], "none")
        self.assertTrue(s["dead_time_includes_cam_offset"])
        self.assertTrue(any("total delay" in n for n in s["warnings"]))
        total = self.p.dead + self.p.decode_delay - 0.002 / 2 + self.p.cam_latency   # dead + decode - ack + camera
        self.assertAlmostEqual(s["dynamics"]["joint"]["dead_time"], total, delta=0.02)


if __name__ == "__main__":
    unittest.main()
