"""Scheduler and firmware safety paths against the simulated rig (no hardware)."""
import math
import shutil
import tempfile
import unittest
from pathlib import Path

import numpy as np

from graspion_calib import protocol as P
from graspion_calib import schedules as S
from graspion_calib import sim
from graspion_calib.extract import read_csv
from graspion_calib.geometry import Arena

from .simrig import BOT, abort_on_pause, run_session


class TempDirCase(unittest.TestCase):
    def setUp(self):
        self.dir = Path(tempfile.mkdtemp(prefix="graspion_test_"))
        abort_on_pause.messages.clear()

    def tearDown(self):
        shutil.rmtree(self.dir, ignore_errors=True)


class TestFirmwareSemantics(unittest.TestCase):
    """SimFirmware mirrors calib_bot.ino; these pin down the behaviour both must have."""

    def setUp(self):
        self.p = sim.SimParams(ir_loss=0.0)
        self.w = sim.SimWorld({BOT: (self.p, (0.5, 0.5, 0.0))}, seed=0)
        self.fw = self.w.fw[BOT]

    def tx(self, raw):
        self.w.transmit(raw, self.w.t)
        self.w.advance(self.w.t + 0.08)

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
        self.tx(P.pack(BOT, P.OP_DRIVE, 80, 80))                     # segment closed: ignored
        self.assertEqual((self.fw.L, self.fw.R), (0, 90))


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
                if (raw >> 13) & 7 == P.OP_SEG and "t" not in state:
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
                if (raw >> 13) & 7 == P.OP_SEG and not dropped:
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


if __name__ == "__main__":
    unittest.main()
