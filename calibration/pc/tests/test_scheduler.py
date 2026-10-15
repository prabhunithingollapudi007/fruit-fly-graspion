"""Scheduler and firmware safety paths against the simulated rig (no hardware)."""
import contextlib
import dataclasses
import io
import json
import math
import shutil
import sys
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import numpy as np

from graspion_calib import bridge as B
from graspion_calib import protocol as P
from graspion_calib import schedules as S
from graspion_calib import sim
from graspion_calib.__main__ import main as cli_main
from graspion_calib.clock import SimClock
from graspion_calib.extract import read_csv
from graspion_calib.geometry import Arena
from graspion_calib.poses import Detection, PoseStore, TrackerCSV
from graspion_calib.scheduler import RecentreConfig, SafetyConfig, Session, SessionConfig

from .simrig import BOT, SERIAL, FakeBlasterSerial, FakeBotSerial, abort_on_pause, bot_info_line, run_session


def op_of(raw):
    return P.unpack(raw).op


def wall_clearance(world, tracker_csv, arena):
    """Smallest distance of the true body point to the arena edge over the tracked run [m]."""
    tr = read_csv(tracker_csv)
    bx, by, _ = world.params[BOT].geometry.body_from_marker(tr["x"], tr["y"], tr["theta"])
    return float(np.min(np.minimum.reduce([bx - arena.xmin, arena.xmax - bx, by - arena.ymin, arena.ymax - by])))


class TempDirCase(unittest.TestCase):
    def setUp(self):
        self.dir = Path(tempfile.mkdtemp(prefix="graspion_test_"))
        abort_on_pause.messages.clear()

    def tearDown(self):
        shutil.rmtree(self.dir, ignore_errors=True)


class TestFirmwareSemantics(unittest.TestCase):
    """SimFirmware mirrors calib_bot.ino; these pin down the behaviour both must have.

    They run the Python model only. calib_bot.ino itself is covered just by compiling it (with the static_assert
    protocol vectors), so reverting a fix there leaves these green: change both together, by hand."""

    def setUp(self):
        self.p = sim.SimParams(ir_loss=0.0)
        self.w = sim.SimWorld({BOT: (self.p, (0.5, 0.5, 0.0))}, seed=0)
        self.fw = self.w.fw[BOT]

    def tx(self, raw):
        """Send one frame and run until the bot has handled it, spacing frames like the blaster (no collision)."""
        t0 = self.w.t
        self.w.transmit(raw, t0)
        self.w.advance(max(t0 + B.nec_frame_s(raw) + self.p.decode_delay + 0.002, t0 + B.PERIOD_MIN_MS / 1000))

    def test_command_watchdog(self):
        self.tx(P.pack(BOT, P.OP_SEG, 90, 90))
        self.assertEqual((self.fw.L, self.fw.R), (90, 90))
        t_last = self.fw.last_frame
        self.w.advance(t_last + self.p.watchdog_s - 0.01)
        self.assertEqual((self.fw.L, self.fw.R), (90, 90))
        self.w.advance(t_last + self.p.watchdog_s + 0.01)
        self.assertEqual((self.fw.L, self.fw.R), (0, 0))
        self.assertEqual(self.fw.log[-1]["end_reason"], 4)
        self.w.advance(self.w.t + 0.5)
        self.assertLess(abs(self.w.plants[BOT].v) + abs(self.w.plants[BOT].w), 1e-3)

    def test_filters(self):
        self.tx(P.pack(BOT + 1, P.OP_SEG, 90, 90))                   # other bot
        self.tx(P.pack(BOT, P.OP_SEG, 90, 90) ^ (1 << 20))           # corrupted
        self.tx(P.pack(BOT, P.OP_DRIVE, 90, 90))                     # DRIVE without a SEG
        self.assertEqual((self.fw.L, self.fw.R), (0, 0))
        self.assertEqual(self.fw.stats["other_addr"], 1)
        self.assertEqual(self.fw.stats["crc_err"], 1)
        self.assertEqual(self.fw.stats["drive_no_seg"], 1)
        self.tx(P.pack(BOT, P.OP_SEG, 125, -125))                    # clamped to the drive cap
        self.assertEqual((self.fw.L, self.fw.R), (120, -120))
        self.tx(P.pack(P.ADDR_BROADCAST, P.OP_STOP))
        self.assertEqual((self.fw.L, self.fw.R), (0, 0))

    def test_recentre_closes_segment(self):
        self.tx(P.pack(BOT, P.OP_SEG, 80, 80))
        self.tx(P.pack(BOT, P.OP_RECENTRE, 0, 90))
        self.assertEqual(self.fw.log[-1]["end_reason"], 2)
        self.assertEqual((self.fw.L, self.fw.R), (0, 90))
        self.tx(P.pack(BOT, P.OP_DRIVE, 80, 80))                     # segment closed: not executed, stops
        self.assertEqual((self.fw.L, self.fw.R), (0, 0))
        self.assertEqual(self.fw.stats["drive_no_seg"], 1)


class TestScheduler(TempDirCase):
    def test_grid_session(self):
        specs = S.static_grid(blocks=1)[:10]
        world, counts = run_session(self.dir, specs)
        segs, frames = read_csv(self.dir / "segments.csv"), read_csv(self.dir / "frames.csv")
        self.assertTrue(np.all(segs["status"] == "ok"))
        self.assertEqual(int(np.sum(segs["attempt"] == 0)), 10)
        for raw in frames["raw"]:
            self.assertEqual(P.check(int(raw, 16)), P.OK)
        self.assertEqual(frames["purpose"][-1], "final_broadcast")
        for k in range(len(segs["seg"])):                           # a STOP follows every segment
            after = frames["t_ack"] > segs["t_seg_ack"][k]
            ops = frames["op"][after]
            first_non_drive = next(o for o in ops if o != "DRIVE")
            self.assertEqual(first_non_drive, "STOP")
        self.assertEqual((world.fw[BOT].L, world.fw[BOT].R), (0, 0))
        self.assertEqual(world.fw[BOT].stats["wd_trips"], 0)

    def test_marker_loss_stops_bot(self):
        state = {}

        def bridge(world):
            b = sim.SimBridge(world)
            send = b.send

            def wrapped(raw, check=True):
                rec = send(raw, check)
                if op_of(raw) == P.OP_SEG and "t" not in state:
                    state["t"] = rec.t_ack + 1.0
                    world.schedule(state["t"], lambda t: world.occluded.add(world.params[BOT].geometry.marker_id))
                return rec
            b.send = wrapped
            return b

        spec = [S.drive(80, 90, duration_s=4.0), S.drive(80, 80)]    # the 2nd waits for the marker -> pause
        world, counts = run_session(self.dir, spec, bridge_factory=bridge)
        segs, frames = read_csv(self.dir / "segments.csv"), read_csv(self.dir / "frames.csv")
        self.assertEqual(segs["status"][0], "marker_loss")
        t_stop = frames["t_ack"][(frames["op"] == "STOP") & (frames["t_ack"] > state["t"])][0]
        self.assertLess(t_stop - state["t"], 0.25 + world.cam_latency + 0.25)   # loss timeout + latency + 1-2 frames
        self.assertTrue(any("not visible" in m for m in abort_on_pause.messages))
        self.assertEqual((world.fw[BOT].L, world.fw[BOT].R), (0, 0))

    def test_geofence(self):
        arena = Arena(0.0, 0.45, 0.0, 0.45, 0.08)
        world, counts = run_session(self.dir, [S.drive(100, 100, duration_s=6.0)], arena=arena,
                                    start=(0.22, 0.22, 0.0))
        segs = read_csv(self.dir / "segments.csv")
        self.assertEqual(segs["status"][0], "geofence")
        tr = read_csv(self.dir / "tracker.csv")
        g = world.params[BOT].geometry
        bx, by, _ = g.body_from_marker(tr["x"], tr["y"], tr["theta"])
        self.assertTrue(np.all((bx > arena.xmin) & (bx < arena.xmax) & (by > arena.ymin) & (by < arena.ymax)),
                        "bot left the arena")

    def test_low_battery(self):
        p = sim.SimParams(v_bat0=3.45, v_sag=0.12)
        world, counts = run_session(self.dir, [S.drive(110, 110)], params=p)
        segs = read_csv(self.dir / "segments.csv")
        self.assertIn("lowbat", list(segs["status"]))
        self.assertTrue(world.fw[BOT].lowbat)
        self.assertEqual((world.fw[BOT].L, world.fw[BOT].R), (0, 0))
        self.assertTrue(any("low battery" in m for m in abort_on_pause.messages))

    def test_missed_seg_is_retried(self):
        dropped = []

        def bridge(world):
            b = sim.SimBridge(world)
            tx = world.transmit

            def lossy(raw, t_start):
                if op_of(raw) == P.OP_SEG and not dropped:
                    dropped.append(raw)                              # the first SEG never reaches the bot
                    return
                tx(raw, t_start)
            world.transmit = lossy
            return b

        world, counts = run_session(self.dir, [S.drive(80, 80)], bridge_factory=bridge)
        segs = read_csv(self.dir / "segments.csv")
        self.assertEqual(list(segs["status"]), ["missed", "ok"])
        self.assertEqual(list(segs["attempt"]), [0, 1])

    def test_recentre_from_corner(self):
        world, counts = run_session(self.dir, [S.drive(70, 70, duration_s=1.0)], start=(0.15, 0.85, -2.5))
        ev = read_csv(self.dir / "events.csv")
        self.assertIn("recentred", list(ev["event"]))
        segs = read_csv(self.dir / "segments.csv")
        hx, hy = 0.5, 0.5
        self.assertLess(math.hypot(segs["x0"][0] - hx, segs["y0"][0] - hy), 0.08)

    def test_geofence_lookahead_follows_period(self):
        """A longer blaster period means a longer stop latency; the lookahead must grow with it (README B.4)."""
        arena = Arena(0.0, 1.0, 0.0, 1.0, 0.08)
        for th in (0.0, 2.4):
            with self.subTest(th=th):
                out = self.dir / f"th{th}"
                p = sim.SimParams(ir_loss=0.0, watchdog_s=1.0)          # 2.5 periods
                world, _ = run_session(out, [S.drive(85, 120, duration_s=8.0)], params=p, arena=arena,
                                       start=(0.5, 0.5, th), bridge_factory=lambda w: sim.SimBridge(w, period_ms=400))
                self.assertEqual(read_csv(out / "segments.csv")["status"][0], "geofence")
                meta = json.loads((out / "session.json").read_text())
                self.assertAlmostEqual(meta["geofence_lookahead_s"], 0.05 + 2 * 0.4 + 0.1)
                # with the old fixed 0.3 s lookahead the body point got to 3-5 cm from the wall here
                self.assertGreater(wall_clearance(world, out / "tracker.csv", arena), arena.margin - 0.01)


class TestRecentreSafety(TempDirCase):
    """Re-centring with wrong guesses (README B.7) must stop the bot instead of driving it into the wall."""

    def test_marker_yaw_off_by_pi(self):
        p = sim.SimParams()
        g_pc = dataclasses.replace(p.geometry, marker_yaw=p.geometry.marker_yaw + math.pi)   # marker glued backwards
        arena = Arena(0.0, 1.0, 0.0, 1.0, 0.08)
        world, _ = run_session(self.dir, [S.drive(80, 80, duration_s=1.0)], params=p, geometry=g_pc, arena=arena,
                               start=(0.2, 0.2, math.pi + math.pi / 4 + 0.3))
        ev = list(read_csv(self.dir / "events.csv")["event"])
        self.assertTrue({"recentre_diverging", "recentre_geofence", "recentre_no_progress"} & set(ev), ev)
        self.assertTrue(any("re-centring failed" in m for m in abort_on_pause.messages))
        self.assertLess(world.t, 6.0)                                    # stopped in seconds, not after 25 s
        self.assertGreater(wall_clearance(world, self.dir / "tracker.csv", arena), 0.04)
        self.assertEqual((world.fw[BOT].L, world.fw[BOT].R), (0, 0))

    def test_swapped_pivot_pairs(self):
        arena = Arena(0.0, 1.0, 0.0, 1.0, 0.08)
        rc = RecentreConfig(pivot_ccw=(90, 0), pivot_cw=(0, 90))         # e.g. a board that pivots the other way
        world, _ = run_session(self.dir, [S.drive(80, 80, duration_s=1.0)], recentre=rc, arena=arena,
                               start=(0.2, 0.2, 0.0))
        ev = list(read_csv(self.dir / "events.csv")["event"])
        self.assertTrue({"recentre_geofence", "recentre_no_progress"} & set(ev), ev)
        self.assertLess(world.t, 8.0)
        self.assertGreater(wall_clearance(world, self.dir / "tracker.csv", arena), 0.015)

    def test_low_battery_during_recentre(self):
        """The latched bot answers RECENTRE with a red flash and no motion: pause for the battery at once."""
        p = sim.SimParams(v_bat0=3.42, v_sag=0.12)                       # trips once the motors load it
        world, _ = run_session(self.dir, [S.drive(80, 80)], params=p, start=(0.2, 0.2, 0.0))
        ev = list(read_csv(self.dir / "events.csv")["event"])
        self.assertIn("recentre_lowbat", ev)
        self.assertTrue(world.fw[BOT].lowbat)
        self.assertTrue(any("low battery" in m for m in abort_on_pause.messages))
        self.assertFalse(any("re-centring failed" in m for m in abort_on_pause.messages))
        self.assertLess(world.t, 8.0)


class MonoPoseStore(PoseStore):
    """What the scheduler gets from the tracker's luma mode (mono camera): led_b = patch brightness, which either
    flash colour raises, and led_r = NaN."""

    def put(self, det):
        super().put(dataclasses.replace(det, led_b=det.led_b + det.led_r, led_r=float("nan")))


def run_mono_session(out, specs, params, start, led_mode="auto"):
    """simrig.run_session with a mono camera; returns (world, printed text)."""
    out.mkdir(parents=True, exist_ok=True)
    world = sim.SimWorld({BOT: (params, start)}, seed=0, pose_store=MonoPoseStore(),
                         tracker_csv=TrackerCSV(out / "tracker.csv"))
    cfg = SessionConfig(BOT, params.geometry, Arena(0.0, 1.0, 0.0, 1.0, 0.08), str(out), led_mode=led_mode)
    printed = io.StringIO()
    try:
        with contextlib.redirect_stdout(printed):
            Session(cfg, sim.SimBridge(world), world.poses, SimClock(world), on_pause=abort_on_pause).run(specs)
    finally:
        world.csv.close()
    return world, printed.getvalue()


class ChromaPoseStore(PoseStore):
    """The tracker's chroma channels on a colour camera, where each flash colour pulls the other channel down by
    half its rise (led_r = R - (G+B)/2: a blue flash lowers it). While `hidden`, the LED patch is outside the
    image: both LED values NaN (the sim's own camera has neither effect)."""

    def __init__(self, base):
        super().__init__()
        self.base, self.hidden = base, True

    def put(self, det):
        nan = float("nan")
        if self.hidden:
            det = dataclasses.replace(det, led_b=nan, led_r=nan)
        else:
            rise_b, rise_r = det.led_b - self.base[0], det.led_r - self.base[1]
            det = dataclasses.replace(det, led_b=det.led_b - 0.5 * rise_r, led_r=det.led_r - 0.5 * rise_b)
        super().put(det)


class TestLedSignal(TempDirCase):
    """LED checks with NaN samples (LED patch outside the image) and with a mono camera (no red channel)."""

    def test_sync_flash_with_no_red_baseline(self):
        """No LED sample in the second before the SEG: the blue sync flash's dip in led_r, measured from the
        window's lowest sample, used to read as a red low-battery flash. A latched bot's red flash still counts."""
        for v_bat0, expected in ((4.05, "ok"), (3.45, "lowbat")):
            with self.subTest(v_bat0=v_bat0):
                out = self.dir / f"v{v_bat0}"
                out.mkdir()
                p = sim.SimParams(ir_loss=0.0, v_bat0=v_bat0, v_sag=0.12)
                store = ChromaPoseStore(p.led_base)
                world = sim.SimWorld({BOT: (p, (0.45, 0.5, 0.3))}, seed=0, pose_store=store,
                                     tracker_csv=TrackerCSV(out / "tracker.csv"))
                bridge = sim.SimBridge(world)
                send = bridge.send

                def show_led_from_seg(raw, check=True, send=send, store=store):
                    if op_of(raw) == P.OP_SEG:
                        store.hidden = False                     # the LED patch comes into view with the SEG
                    return send(raw, check)
                bridge.send = show_led_from_seg
                cfg = SessionConfig(BOT, p.geometry, Arena(0.0, 1.0, 0.0, 1.0, 0.08), str(out), led_mode="chroma")
                try:
                    Session(cfg, bridge, world.poses, SimClock(world), on_pause=abort_on_pause).run(
                        [dataclasses.replace(S.drive(110, 110), recentre=False)])
                finally:
                    world.csv.close()
                segs = read_csv(out / "segments.csv")
                self.assertEqual(segs["status"][0], expected)
                if expected == "ok":
                    self.assertEqual(int(segs["flash_live"][0]), 1)
                    self.assertFalse(abort_on_pause.messages)
                else:
                    self.assertTrue(world.fw[BOT].lowbat)

    def _session(self, **cfg_kw):
        world = sim.SimWorld({BOT: (sim.SimParams(ir_loss=0.0), (0.5, 0.5, 0.0))}, seed=0)
        cfg = SessionConfig(BOT, sim.SimParams().geometry, Arena(), str(self.dir), **cfg_kw)
        return Session(cfg, sim.SimBridge(world), PoseStore(), SimClock(world), on_pause=abort_on_pause)

    def _put(self, sess, samples):
        mid = sess.cfg.geometry.marker_id
        for k, (t, b, r) in enumerate(samples):
            sess.poses.put(Detection(k, t, mid, 0.5, 0.5, 0.0, b, r, 0.02, 4))

    def test_nan_samples_do_not_hide_a_flash(self):
        nan = float("nan")
        sess = self._session(led_mode="chroma")
        self._put(sess, [(0.0, nan, nan), (0.1, 30.0, nan), (0.2, 32.0, nan), (0.3, nan, nan),
                         (1.0, nan, nan), (1.05, 160.0, nan), (1.1, 31.0, nan)])
        base_b, base_r = sess._led_baseline(1.0)
        self.assertAlmostEqual(base_b, 31.0)                     # median of the finite samples only
        self.assertTrue(math.isnan(base_r))                      # nothing finite: NaN, not 0
        self.assertTrue(sess._flash_seen(1.0, 1.2, base_b, "b"))  # max() over [nan, 160, ...] would be nan
        self.assertTrue(sess._flash_seen(1.0, 1.2, nan, "b"))     # no baseline: rise over the window's minimum
        self.assertFalse(sess._flash_seen(1.0, 1.2, base_r, "r"))  # no finite red sample: no flash, no error
        self.assertFalse(sess._flash_seen(1.1, 1.2, base_b, "b"))
        sess.run([])                                             # closes the session files

    def test_mono_warns_once_at_session_start(self):
        sess = self._session(led_mode="luma")
        printed = io.StringIO()
        with contextlib.redirect_stdout(printed):
            sess.run([])
            self.assertTrue(sess._mono())                        # no second warning
        self.assertEqual(printed.getvalue().count("warning: mono camera"), 1)
        ev = read_csv(self.dir / "events.csv")
        self.assertEqual(ev["event"][0], "warning")
        meta = json.loads((self.dir / "session.json").read_text())
        self.assertEqual(meta["led_mode"], "luma")
        self.assertEqual(len(meta["warnings"]), 1)

    def test_mono_low_battery_falls_back_to_the_bot_latch(self):
        """As TestRecentreSafety.test_low_battery_during_recentre, on a mono camera: the red flash is not seen, the
        bot stops itself, and the re-centring progress check pauses with a battery hint."""
        p = sim.SimParams(v_bat0=3.42, v_sag=0.12)
        world, printed = run_mono_session(self.dir, [S.drive(80, 80)], p, (0.2, 0.2, 0.0))
        ev = list(read_csv(self.dir / "events.csv")["event"])
        self.assertNotIn("recentre_lowbat", ev)
        self.assertIn("recentre_no_progress", ev)
        self.assertEqual(ev.count("warning"), 1)
        self.assertEqual(printed.count("warning: mono camera"), 1)
        self.assertTrue(world.fw[BOT].lowbat)                    # the bot's own latch stopped it
        self.assertEqual((world.fw[BOT].L, world.fw[BOT].R), (0, 0))
        self.assertTrue(any("re-centring failed (no_progress)" in m and "mono camera" in m
                            for m in abort_on_pause.messages), abort_on_pause.messages)
        self.assertLess(world.t, 12.0)
        self.assertEqual(json.loads((self.dir / "session.json").read_text())["led_mode"], "luma")   # from 'auto'

    def test_mono_latched_segment_is_stalled(self):
        """Mono camera, latch tripped mid-segment: the next strong segment flashes without moving -> 'stalled' and
        the low-battery pause (instead of an 'ok' segment with zero twist)."""
        p = sim.SimParams(v_bat0=3.45, v_sag=0.12)
        specs = [dataclasses.replace(S.drive(110, 110), recentre=False) for _ in range(2)]
        world, printed = run_mono_session(self.dir, specs, p, (0.45, 0.5, 0.3), led_mode="luma")
        segs = read_csv(self.dir / "segments.csv")
        self.assertEqual(list(segs["status"]), ["ok", "stalled"])     # the 1st moved before the latch stopped it
        self.assertEqual(int(segs["flash_live"][1]), 1)
        self.assertTrue(world.fw[BOT].lowbat)
        self.assertTrue(any("flashed but did not move" in m and "low-battery latch" in m
                            for m in abort_on_pause.messages), abort_on_pause.messages)

    def test_mono_sync_flash_still_detected(self):
        """Luma mode keeps the sync-flash check (no 'missed' retries), with the red channel all NaN; a spin in
        place is not 'stalled'."""
        specs = [S.drive(80, 80, duration_s=1.0), S.drive(-90, 90, duration_s=1.0)]
        world, printed = run_mono_session(self.dir, specs, sim.SimParams(ir_loss=0.0), (0.5, 0.5, 0.0),
                                          led_mode="luma")
        segs = read_csv(self.dir / "segments.csv")
        self.assertEqual(list(segs["status"]), ["ok", "ok"])
        self.assertEqual(list(segs["flash_live"]), [1, 1])


class TestExitPath(TempDirCase):
    """On any exit the bot gets STOP (stop_repeats times) and a broadcast STOP, even if one of them fails."""

    def _serial_bridge(self, holder, **fake_kw):
        def factory(world):
            holder["fake"] = fake = FakeBlasterSerial(world, **fake_kw)
            holder["world"] = world
            return B.SerialBridge(None, reset_wait_s=0.0, ser=fake, now=lambda: world.t, sleep=SimClock(world).sleep)
        return factory

    def test_ctrl_c_mid_reply(self):
        # consume 0: Ctrl-C before the reply arrived (it stays queued); 3: in the middle of "K <hex8> ..."
        for consume in (0, 3):
            with self.subTest(consume=consume):
                out, h = self.dir / f"c{consume}", {}
                with self.assertRaises(KeyboardInterrupt):                # not masked by a BridgeError
                    run_session(out, [S.drive(90, 90, duration_s=3.0)], start=(0.5, 0.5, 0.0), write_dump=False,
                                bridge_factory=self._serial_bridge(h, interrupt_at=20, consume=consume))
                world, fake = h["world"], h["fake"]
                world.csv = None                                          # closed by run_session
                world.advance(world.t + 0.05)                             # let the last frame decode
                last = [P.unpack(r) for r in fake.sent[-3:]]
                self.assertEqual([(f.op, f.bot_id) for f in last],
                                 [(P.OP_STOP, BOT), (P.OP_STOP, BOT), (P.OP_STOP, P.ADDR_BROADCAST)])
                self.assertEqual(op_of(fake.sent[-4]), P.OP_DRIVE)        # the interrupted keep-alive
                frames, ev = read_csv(out / "frames.csv"), read_csv(out / "events.csv")
                self.assertEqual(list(frames["purpose"][-2:]), ["final", "final_broadcast"])
                self.assertIn("final_stop_error", list(ev["event"]))
                self.assertIn("abort", list(ev["event"]))
                self.assertEqual((world.fw[BOT].L, world.fw[BOT].R), (0, 0))
                self.assertIn("finished_wall", json.loads((out / "session.json").read_text()))

    @staticmethod
    def _failing_bridge(fails):
        """SimBridge whose n-th STOP raises BridgeError when fails(n). One drive segment from home sends STOPs
        1-2 pre_rest, 3-4 end, 5-6 final, 7 final broadcast."""
        def factory(world):
            b = sim.SimBridge(world)
            send, n = b.send, [0]

            def wrapped(raw, check=True):
                if op_of(raw) == P.OP_STOP:
                    n[0] += 1
                    if fails(n[0]):
                        raise B.BridgeError(f"no reply to STOP {n[0]} (simulated)")
                return send(raw, check)
            b.send = wrapped
            return b
        return factory

    def test_failed_final_stop_does_not_skip_the_rest(self):
        world, _ = run_session(self.dir, [S.drive(80, 80, duration_s=1.0)], start=(0.5, 0.5, 0.0),
                               bridge_factory=self._failing_bridge(lambda n: n == 5))
        frames, ev = read_csv(self.dir / "frames.csv"), read_csv(self.dir / "events.csv")
        self.assertEqual(list(frames["purpose"][-2:]), ["final", "final_broadcast"])
        self.assertEqual(list(ev["event"]).count("final_stop_error"), 1)

    def test_no_final_stop_acknowledged_raises(self):
        with self.assertRaises(B.BridgeError):
            run_session(self.dir, [S.drive(80, 80, duration_s=1.0)], start=(0.5, 0.5, 0.0),
                        bridge_factory=self._failing_bridge(lambda n: n >= 5))
        self.assertEqual(list(read_csv(self.dir / "events.csv")["event"]).count("final_stop_error"), 3)
        self.assertIn("finished_wall", json.loads((self.dir / "session.json").read_text()))


class TestSerialBridge(unittest.TestCase):
    """SerialBridge against FakeBlasterSerial, which mirrors ir_blaster.ino's line protocol.

    The blaster side here is the Python model in simrig: these pin the PC side and the intended blaster behaviour.
    ir_blaster.ino itself is covered just by compiling it (with the static_assert protocol vectors), so a change
    to its line handling must be mirrored in FakeBlasterSerial by hand."""

    def setUp(self):
        self.world = sim.SimWorld({BOT: (sim.SimParams(ir_loss=0.0), (0.5, 0.5, 0.0))}, seed=0)
        self.fake = FakeBlasterSerial(self.world)

    def bridge(self, **kw):
        return B.SerialBridge(None, reset_wait_s=0.0, ser=self.fake, now=lambda: self.world.t,
                              sleep=SimClock(self.world).sleep, **kw)

    def test_open_resyncs_a_stale_blaster(self):
        seg = P.pack(BOT, P.OP_SEG, 90, 90)
        self.fake._line = f"F {seg:08X}".encode()      # a whole request but its newline, left by a killed PC process
        self.fake.rx = b"35A 4880 4948\r\n"            # and the tail of an old reply on the PC side
        br = self.bridge()
        self.assertTrue(br.info.startswith("I "))
        self.assertNotIn(seg, self.fake.sent)          # a bare newline used to complete the line and send the SEG
        self.world.advance(self.world.t + 0.1)
        self.assertEqual(self.world.fw[BOT].log, [])   # the bot never started a segment
        rec = br.send(P.pack(BOT, P.OP_STOP))
        self.assertEqual(rec.raw, P.pack(BOT, P.OP_STOP))
        br.resync()                                    # an idle resync (the exit path) sends nothing either
        self.assertEqual(self.fake.sent, [P.pack(BOT, P.OP_STOP)])

    def test_blaster_on_another_protocol_is_refused(self):
        self.fake = FakeBlasterSerial(self.world, proto=1)
        with self.assertRaisesRegex(B.BridgeError, r"protocol 1\b.*Reflash ir_blaster"):
            self.bridge()
        self.assertTrue(self.fake.closed)
        self.assertEqual(self.fake.sent, [])

    def test_period_limits_and_read_timeout(self):
        br = self.bridge()
        with self.assertRaises(B.BridgeError):
            br.set_period(70)                          # long frames would merge at the receiver
        br.set_period(1500)
        self.assertGreaterEqual(self.fake.timeout, 1.5 + B.NEC_FRAME_MAX_S)
        br.set_period(110)
        self.assertEqual(self.fake.timeout, 1.0)

    def test_overlong_line_is_dropped_whole(self):
        """Exercises FakeBlasterSerial's copy of ir_blaster.ino's lineDrop rule (see the class docstring)."""
        br = self.bridge()
        n_sent = len(self.fake.sent)
        with self.assertRaisesRegex(B.BridgeError, "E long"):
            br.request("X" * 40 + " f " + f"{P.pack(BOT, P.OP_SEG, 80, 80):08X}")
        self.assertEqual(len(self.fake.sent), n_sent)  # the tail did not run as a command

    def test_nec_frame_length(self):
        self.assertAlmostEqual(B.nec_frame_s(0x0000FFFF), 0.06776)        # 16 ones: classic NEC, ~68 ms
        self.assertAlmostEqual(B.NEC_FRAME_MAX_S, 0.08568)
        self.assertGreaterEqual(B.PERIOD_MIN_MS / 1000, B.NEC_FRAME_MAX_S + B.RECORD_GAP_S)
        lens = [B.nec_frame_s(P.pack(BOT, op, L, R)) for op in (P.OP_STOP, P.OP_SEG, P.OP_DRIVE)
                for L in range(-120, 121, 10) for R in range(-120, 121, 10)]
        self.assertGreater(max(lens) - min(lens), 0.01)                   # payload-dependent, not a constant


class TestProtocolVersion(TempDirCase):
    """A bot built with another IR protocol version rejects every frame as a CRC error: refuse it up front (the
    blaster side is TestSerialBridge.test_blaster_on_another_protocol_is_refused)."""

    def register(self, info_line):
        fake = FakeBotSerial(info_line)
        with mock.patch.dict(sys.modules, {"serial": fake.serial_module()}), \
                contextlib.redirect_stdout(io.StringIO()):
            cli_main(["register", "--port", "COM_TEST", "--bots-dir", str(self.dir / "bots")])

    def test_register_checks_the_bot_protocol(self):
        with self.assertRaisesRegex(SystemExit, r"bot on COM_TEST reports IR protocol 1\b.*Reflash calib_bot"):
            self.register(bot_info_line(proto=1))
        self.assertFalse((self.dir / "bots").exists())          # nothing registered
        self.register(bot_info_line())
        info = json.loads((self.dir / "bots" / f"{P.bot_id_from_usb_serial(SERIAL)}.json").read_text())
        self.assertEqual(info["proto"], str(P.PROTO_VERSION))

    def test_run_checks_the_registered_protocol(self):
        from graspion_calib.__main__ import _check_proto
        _check_proto(BOT, P.info_fields(bot_info_line()), "bots/90.json")
        _check_proto(BOT, {"watchdog_ms": "300"}, "bots/90.json")   # hand-written, no proto: nothing to compare
        with self.assertRaisesRegex(SystemExit, r"bots/90\.json.*protocol 1\b.*register again"):
            _check_proto(BOT, P.info_fields(bot_info_line(proto=1)), "bots/90.json")


class TestScheduleSeed(TempDirCase):
    def cli(self, *args):
        with contextlib.redirect_stdout(io.StringIO()):
            cli_main(list(args))

    def test_fresh_seed_per_schedule_and_recorded(self):
        a, b, c = (self.dir / f"{k}.json" for k in "abc")
        self.cli("schedule", "grid", "--blocks", "1", "--out", str(a))
        self.cli("schedule", "grid", "--blocks", "1", "--out", str(b))
        (sa, ma), (sb, mb) = S.load_with_meta(a), S.load_with_meta(b)
        self.assertNotEqual(ma["seed"], mb["seed"])
        self.assertNotEqual([(s.L, s.R) for s in sa], [(s.L, s.R) for s in sb])
        self.cli("schedule", "grid", "--blocks", "1", "--seed", str(ma["seed"]), "--out", str(c))
        self.assertEqual([(s.L, s.R) for s in S.load(c)], [(s.L, s.R) for s in sa])
        self.assertEqual(ma["kind"], "grid")

    def test_watchdog_vs_period_check(self):
        from graspion_calib.__main__ import _check_watchdog
        info = {"watchdog_ms": "300"}                                      # as read from the bot's 'i' line
        _check_watchdog(BOT, info, 110, [S.drive(80, 80)])                 # 300 >= 2.5 * 110
        with self.assertRaises(SystemExit):
            _check_watchdog(BOT, info, 130, [S.drive(80, 80)])
        _check_watchdog(BOT, info, 400, S.pings(5.0))                     # link test at a long period: no motion

    def test_old_list_files_still_load(self):
        p = self.dir / "old.json"
        specs = S.static_grid(blocks=1, seed=3)
        p.write_text(json.dumps([dataclasses.asdict(s) for s in specs]))
        loaded, meta = S.load_with_meta(p)
        self.assertEqual(meta, {})
        self.assertEqual([(s.L, s.R) for s in loaded], [(s.L, s.R) for s in specs])

    def test_validation_p_min(self):
        fwd = self.dir / "forward_map.json"
        sim.default_truth().save(fwd)
        a, b = self.dir / "a.json", self.dir / "b.json"
        self.cli("schedule", "validation", "--forward-map", str(fwd), "--seed", "1", "--out", str(a))
        self.cli("schedule", "validation", "--forward-map", str(fwd), "--seed", "1", "--p-min", "80", "--out", str(b))
        sa, (sb, mb) = S.load(a), S.load_with_meta(b)
        self.assertTrue(sa)
        self.assertTrue(all(min(s.L, s.R) >= 60 for s in sa))
        self.assertTrue(all(min(s.L, s.R) >= 80 for s in sb))
        self.assertLess(len(sb), len(sa))                                 # a smaller region: fewer feasible targets
        self.assertEqual(mb["args"]["p_min"], [80.0])
        with self.assertRaises(SystemExit):                               # p_min inside the dead zone
            self.cli("schedule", "validation", "--forward-map", str(fwd), "--p-min", "40", "--out", str(b))


# references and a bot marker for the tracker CLI, rendered through a 1 m -> 640 px overhead view
REFS = {0: (0.05, 0.05), 1: (0.95, 0.05), 2: (0.95, 0.95), 3: (0.05, 0.95)}
H_PX = np.array([[640.0, 0, 320], [0, -640.0, 680], [0, 0, 1]])
SIZE = 0.04


class TestTrackerCli(TempDirCase):
    """run / track build the tracker from the rig config: LED height, camera height, references."""

    def rig(self, camera_height_m=None):
        camera = {"reference_size_m": SIZE}
        if camera_height_m is not None:
            camera["camera_height_m"] = camera_height_m
        d = {"arena": {"xmin": 0.0, "xmax": 1.0, "ymin": 0.0, "ymax": 1.0, "margin": 0.08},
             "reference_markers": {str(k): list(v) for k, v in REFS.items()}, "camera": camera,
             "bots": {str(BOT): {"marker_id": 10, "marker_size_m": SIZE, "led_offset": [0.03, 0.0],
                                 "led_height": -0.01}}}
        p = self.dir / f"rig_{camera_height_m}.json"
        p.write_text(json.dumps(d))
        return p

    def test_make_tracker_reads_led_and_camera_height(self):
        from graspion_calib.__main__ import _make_tracker, _rig
        rig = _rig(self.rig(1.2))
        tr = _make_tracker(rig, rig.bots.values())
        self.assertAlmostEqual(tr.bots[10].led_height, -0.01)
        self.assertAlmostEqual(tr.bots[10].led_offset[0], 0.03)
        self.assertEqual(tr.camera_height_m, 1.2)
        self.assertEqual(tr.ref_size_m, SIZE)

    def test_track_video_prints_reference_summary_and_warnings(self):
        import cv2
        from graspion_calib.tracker import render_scene
        img = render_scene(H_PX, [(i, x, y, 0.0, SIZE) for i, (x, y) in REFS.items()] + [(10, 0.5, 0.5, 0.3, SIZE)],
                           img_size=(1280, 720))
        video = self.dir / "scene.avi"
        vw = cv2.VideoWriter(str(video), cv2.VideoWriter_fourcc(*"MJPG"), 30.0, (1280, 720))
        if not vw.isOpened():
            self.skipTest("no MJPG video writer in this OpenCV build")
        for _ in range(40):
            vw.write(img)
        vw.release()
        for hc in (1.0, None):
            with self.subTest(camera_height_m=hc):
                out, text = self.dir / f"track_{hc}.csv", io.StringIO()
                with contextlib.redirect_stdout(text):
                    cli_main(["track", "--rig", str(self.rig(hc)), "--out", str(out), "--video", str(video)])
                text = text.getvalue()
                self.assertIn("homography: exact 4-point fit", text)
                self.assertIn("reference marker shapes", text)
                self.assertIn("LED signal:", text)
                self.assertNotIn("reprojection RMS nan", text)
                self.assertGreater(int(np.sum(read_csv(out)["id"] == 10)), 0)
                # led_height is applied with camera_height_m (no intrinsics here), else the tracker says so
                self.assertEqual("led_height ignored" in text, hc is None, text)


if __name__ == "__main__":
    unittest.main()
