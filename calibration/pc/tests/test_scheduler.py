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
from graspion_calib.poses import Detection, PoseStore
from graspion_calib.scheduler import Session, SessionConfig

from .simrig import (BOT, SERIAL, FakeBlasterSerial, FakeBotSerial, ScriptedKeys, abort_on_pause, bot_info_line,
                     enter_key, run_session)


def op_of(raw):
    return P.unpack(raw).op


def wall_clearance(world, tracker_csv, arena):
    """Smallest distance of the true body point to the arena edge over the tracked run [m]."""
    tr = read_csv(tracker_csv)
    bx, by, _ = world.params[BOT].geometry.body_from_marker(tr["x"], tr["y"], tr["theta"])
    return float(np.min(np.minimum.reduce([bx - arena.xmin, arena.xmax - bx, by - arena.ymin, arena.ymax - by])))


def events(out):
    """Event names of a session (read_csv gives {} for a session with no event at all)."""
    return list(read_csv(Path(out) / "events.csv").get("event", []))


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
        self.assertEqual(self.fw.stats["wd_trips"], 1)
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
        self.assertEqual((self.fw.L, self.fw.R), (0, 90))
        self.tx(P.pack(BOT, P.OP_DRIVE, 80, 80))                     # segment closed: not executed, stops
        self.assertEqual((self.fw.L, self.fw.R), (0, 0))
        self.assertEqual(self.fw.stats["drive_no_seg"], 1)


class TestScheduler(TempDirCase):
    """Segments with an operator who presses Enter at once (simrig.enter_key)."""

    def test_grid_session(self):
        specs = S.static_grid(blocks=1)[:10]
        world, counts = run_session(self.dir, specs, start=(0.5, 0.5, 0.3))
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

    def test_geofence_lookahead_follows_period(self):
        """A longer blaster period means a longer stop latency; the lookahead must grow with it."""
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

    def test_flash_check_skips_nan_samples(self):
        world = sim.SimWorld({BOT: (sim.SimParams(ir_loss=0.0), (0.5, 0.5, 0.0))}, seed=0)
        cfg = SessionConfig(BOT, sim.SimParams().geometry, Arena(), str(self.dir))
        sess = Session(cfg, sim.SimBridge(world), PoseStore(), SimClock(world), enter_key, on_pause=abort_on_pause)
        nan = float("nan")
        mid = cfg.geometry.marker_id
        for k, (t, b) in enumerate([(0.0, nan), (0.1, 30.0), (0.2, 32.0), (0.3, nan), (1.0, nan), (1.05, 160.0),
                                    (1.1, 31.0)]):
            sess.poses.put(Detection(k, t, mid, 0.5, 0.5, 0.0, b, nan, 0.02, 4))
        self.assertAlmostEqual(sess._led_baseline(1.0), 31.0)      # median of the finite samples only
        self.assertTrue(sess._flash_seen(1.0, 1.2, 31.0))          # max() over [nan, 160, ...] would be nan
        self.assertTrue(sess._flash_seen(1.0, 1.2, nan))           # no baseline: rise over the window's minimum
        self.assertFalse(sess._flash_seen(1.1, 1.2, 31.0))
        sess.run([])                                               # closes the session files


class TestOperatorPause(TempDirCase):
    """The pause before every segment: the operator places or nudges the bot and starts it (README 5 E)."""

    def run_keys(self, specs, keys, start=(0.3, 0.3, 0.5), **kw):
        world_ref, printed = {}, io.StringIO()

        def bridge(world):
            world_ref["w"] = world
            return sim.SimBridge(world)
        with contextlib.redirect_stdout(printed):
            world, counts = run_session(self.dir, specs, start=start, bridge_factory=bridge, quiet=False,
                                        getkey=ScriptedKeys((lambda: keys(world_ref)) if callable(keys) else keys),
                                        **kw)
        return world, counts, printed.getvalue()

    def test_enter_runs_from_where_the_bot_stands(self):
        world, counts, text = self.run_keys([S.drive(80, 80)], [None, None, "\r"])
        segs, frames = (read_csv(self.dir / f) for f in ("segments.csv", "frames.csv"))
        self.assertEqual(list(segs["status"]), ["ok"])
        self.assertNotIn("RECENTRE", list(frames["op"]))                 # nothing drove the bot home
        self.assertLess(math.hypot(segs["x0"][0] - 0.3, segs["y0"][0] - 0.3), 0.03)   # started where it stood
        self.assertIn("cm from home", text)
        self.assertEqual(frames["purpose"][0], "pause")                  # motors off while the operator handles it
        self.assertFalse(abort_on_pause.messages)

    def test_nudges_then_hand_placement(self):
        after_nudge = []

        def keys(ref):
            def record():
                pl = ref["w"].plants[BOT]
                after_nudge.append((pl.x, pl.y))

            def place():                                                 # the operator sets the bot at home
                pl = ref["w"].plants[BOT]
                pl.x, pl.y, pl.th = 0.5, 0.5, 0.0
            return ["w", record, "a", "d", place, None, None, "\r"]
        world, counts, text = self.run_keys([S.drive(80, 80)], keys)
        segs, frames = (read_csv(self.dir / f) for f in ("segments.csv", "frames.csv"))
        self.assertGreater(math.hypot(after_nudge[0][0] - 0.3, after_nudge[0][1] - 0.3), 0.015)  # the nudge moved it
        rc = [(int(frames["L"][k]), int(frames["R"][k])) for k in range(len(frames["op"])) if frames["op"][k] == "RECENTRE"]
        self.assertEqual(sorted(set(rc)), [(0, 90), (85, 85), (90, 0)])  # the nudge drive pairs, nothing else
        self.assertGreaterEqual(rc.count((85, 85)), 4)                   # 0.6 s at the 110 ms period
        purposes = list(frames["purpose"])
        self.assertIn("nudge_w", purposes)
        self.assertEqual(purposes[purposes.index("nudge_w") + rc.count((85, 85))], "nudge_stop")  # STOP ends it
        self.assertEqual(list(segs["status"]), ["ok"])
        self.assertLess(math.hypot(segs["x0"][0] - 0.5, segs["y0"][0] - 0.5), 0.03)   # ran from the placed pose
        self.assertFalse({e for e in events(self.dir) if e.startswith("nudge_")})    # no nudge was cut short
        self.assertEqual(world.fw[BOT].stats["wd_trips"], 0)
        self.assertEqual(world.fw[BOT].stats["drive_no_seg"], 0)

    def test_skip_and_quit(self):
        specs = [S.drive(80, 80, duration_s=1.0) for _ in range(3)]
        world, counts, text = self.run_keys(specs, ["x", "\r", "q"], start=(0.5, 0.5, 0.0))
        segs, frames = (read_csv(self.dir / f) for f in ("segments.csv", "frames.csv"))
        self.assertEqual(list(segs["seg"]), [1])                        # only the second segment ran
        self.assertEqual(list(segs["status"]), ["ok"])
        ev = events(self.dir)
        self.assertEqual(ev.count("skipped"), 1)
        self.assertIn("abort", ev)
        self.assertEqual(counts["skipped"], 1)
        self.assertEqual(list(frames["purpose"][-2:]), ["final", "final_broadcast"])
        self.assertEqual((world.fw[BOT].L, world.fw[BOT].R), (0, 0))

    def test_nudge_stops_at_the_fence(self):
        """Six forward nudges from 10 cm before the wall, facing it: the fence stops them, not the wall."""
        arena = Arena(0.0, 1.0, 0.0, 1.0, 0.08)
        world, counts, text = self.run_keys([S.drive(80, 80)], ["w"] * 6 + ["x"], start=(0.9, 0.5, 0.0),
                                            arena=arena)
        self.assertIn("nudge_geofence", events(self.dir))
        self.assertIn("nudge stopped: geofence", text)
        self.assertGreater(wall_clearance(world, self.dir / "tracker.csv", arena), 0.02)
        self.assertEqual((world.fw[BOT].L, world.fw[BOT].R), (0, 0))

    def test_hidden_marker_during_the_pause_does_not_prompt(self):
        """A hand over the bot hides the marker while the operator handles it: the status line says so, no
        'marker not visible' pause fires, and a nudge is refused; the segment waits for the marker as usual."""
        def keys(ref):
            mid = ref["w"].params[BOT].geometry.marker_id
            hide = lambda: ref["w"].occluded.add(mid)
            show = lambda: ref["w"].occluded.discard(mid)
            return [hide] + [None] * 12 + ["w", show, None, None, None, "\r"]     # 12 polls = 0.6 s > marker_loss_s
        world, counts, text = self.run_keys([S.drive(80, 80)], keys, start=(0.5, 0.5, 0.0))
        self.assertIn("marker not visible", text)
        self.assertFalse(abort_on_pause.messages)
        self.assertIn("nudge_marker_loss", events(self.dir))             # the blind nudge sent nothing
        self.assertNotIn("RECENTRE", list(read_csv(self.dir / "frames.csv")["op"]))
        self.assertEqual(list(read_csv(self.dir / "segments.csv")["status"]), ["ok"])


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
                    run_session(out, [S.drive(90, 90, duration_s=3.0)], start=(0.5, 0.5, 0.0),
                                bridge_factory=self._serial_bridge(h, interrupt_at=20, consume=consume))
                world, fake = h["world"], h["fake"]
                world.csv = None                                          # closed by run_session
                world.advance(world.t + 0.05)                             # let the last frame decode
                last = [P.unpack(r) for r in fake.sent[-3:]]
                self.assertEqual([(f.op, f.bot_id) for f in last],
                                 [(P.OP_STOP, BOT), (P.OP_STOP, BOT), (P.OP_STOP, P.ADDR_BROADCAST)])
                self.assertEqual(op_of(fake.sent[-4]), P.OP_DRIVE)        # the interrupted keep-alive
                frames = read_csv(out / "frames.csv")
                self.assertEqual(list(frames["purpose"][-2:]), ["final", "final_broadcast"])
                self.assertIn("final_stop_error", events(out))
                self.assertIn("abort", events(out))
                self.assertEqual((world.fw[BOT].L, world.fw[BOT].R), (0, 0))
                self.assertIn("finished_wall", json.loads((out / "session.json").read_text()))

    @staticmethod
    def _failing_bridge(fails):
        """SimBridge whose n-th STOP raises BridgeError when fails(n). One drive segment sends STOPs 1-2 pause,
        3-4 pre_rest, 5-6 end, 7-8 final, 9 final broadcast."""
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
                               bridge_factory=self._failing_bridge(lambda n: n == 7))
        frames = read_csv(self.dir / "frames.csv")
        self.assertEqual(list(frames["purpose"][-2:]), ["final", "final_broadcast"])
        self.assertEqual(events(self.dir).count("final_stop_error"), 1)

    def test_no_final_stop_acknowledged_raises(self):
        with self.assertRaises(B.BridgeError):
            run_session(self.dir, [S.drive(80, 80, duration_s=1.0)], start=(0.5, 0.5, 0.0),
                        bridge_factory=self._failing_bridge(lambda n: n >= 7))
        self.assertEqual(events(self.dir).count("final_stop_error"), 3)
        self.assertIn("finished_wall", json.loads((self.dir / "session.json").read_text()))


class TestSerialBridge(unittest.TestCase):
    """SerialBridge against FakeBlasterSerial, which mirrors the blaster sketch's line protocol.

    The blaster side here is the Python model in simrig: these pin the PC side and the intended blaster behaviour.
    bot_blaster.ino itself is covered by its hardware test (HARDWARE.md 4.5), so a change to its line handling
    must be mirrored in FakeBlasterSerial by hand."""

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
        self.assertFalse(self.world.fw[BOT].seg_active)   # the bot never started a segment
        rec = br.send(P.pack(BOT, P.OP_STOP))
        self.assertEqual(rec.raw, P.pack(BOT, P.OP_STOP))
        br.resync()                                    # an idle resync (the exit path) sends nothing either
        self.assertEqual(self.fake.sent, [P.pack(BOT, P.OP_STOP)])

    def test_blaster_on_another_protocol_is_refused(self):
        self.fake = FakeBlasterSerial(self.world, proto=1)
        with self.assertRaisesRegex(B.BridgeError, r"protocol 1\b"):
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
        """Exercises FakeBlasterSerial's copy of the sketch's lineDrop rule (see the class docstring)."""
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
        self.assertEqual(info["watchdog_ms"], "300")

    def test_run_checks_the_registered_protocol(self):
        from graspion_calib.__main__ import _check_proto
        _check_proto(BOT, P.info_fields(bot_info_line()), "bots/90.json")
        _check_proto(BOT, {"watchdog_ms": "300"}, "bots/90.json")   # hand-written, no proto: nothing to compare
        with self.assertRaisesRegex(SystemExit, r"bots/90\.json.*protocol 1\b.*register again"):
            _check_proto(BOT, P.info_fields(bot_info_line(proto=1)), "bots/90.json")


class TestScheduleCli(TempDirCase):
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
        self.assertEqual(len(sa), 88)                                      # 80 cells + 8 reverse checks

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

    def test_cells_schedule(self):
        out = self.dir / "cells.json"
        self.cli("schedule", "cells", "--cells", "[[80, 80], [100, 60]]", "--out", str(out))
        specs, meta = S.load_with_meta(out)
        self.assertEqual([(s.L, s.R, s.block, s.tag) for s in specs], [(80, 80, 0, "cells"), (100, 60, 0, "cells")])
        self.assertEqual(meta["kind"], "cells")
        self.cli("schedule", "cells", "--cells", "[[80, 80], [100, 60]]", "--blocks", "2", "--out", str(out))
        self.assertEqual([(s.L, s.R, s.block) for s in S.load(out)],
                         [(80, 80, 0), (100, 60, 0), (80, 80, 1), (100, 60, 1)])      # given order, every block
        for bad in (["--cells", "[[130, 0]]"], ["--cells", "[[80]]"], ["--cells", "[]"], []):
            with self.assertRaises(SystemExit):
                self.cli("schedule", "cells", *bad, "--out", str(out))

    def test_parse_selection(self):
        self.assertEqual(S.parse_selection("0-2,5", 8), [0, 1, 2, 5])
        self.assertEqual(S.parse_selection("6-", 8), [6, 7])
        self.assertEqual(S.parse_selection("3, 1,3", 8), [3, 1])                     # given order, each once
        for bad in ("8", "2-1", "", "a", "-1"):
            with self.assertRaises(ValueError):
                S.parse_selection(bad, 8)

    def test_run_only_in_the_simulator(self):
        sched, out = self.dir / "cells.json", self.dir / "session"
        self.cli("schedule", "cells", "--cells", "[[70, 70], [0, 80], [80, 0], [60, 60]]", "--out", str(sched))
        self.cli("run", "--sim", "--bot", str(BOT), "--schedule", str(sched), "--out", str(out), "--only", "1-2,0")
        self.assertEqual([(s.L, s.R) for s in S.load(out / "schedule.json")], [(0, 80), (80, 0), (70, 70)])
        self.assertEqual(list(read_csv(out / "segments.csv")["status"]), ["ok"] * 3)
        meta = json.loads((out / "session.json").read_text())
        self.assertEqual(meta["only"], {"selection": "1-2,0", "indices": [1, 2, 0]})
        self.assertNotIn("RECENTRE", list(read_csv(out / "frames.csv")["op"]))     # no operator, no nudges
        with self.assertRaises(SystemExit):
            self.cli("run", "--sim", "--bot", str(BOT), "--schedule", str(sched), "--out", str(out), "--only", "4")


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
