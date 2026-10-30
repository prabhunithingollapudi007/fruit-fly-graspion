"""SimFirmware against calib_bot.ino's frame handling, and the simulated IR channel and blaster (no hardware).

The firmware rules pinned down here (calib_bot.ino handleFrame / commandWatchdog):
  * only frames that command the drive (STOP, SEG, DRIVE inside a segment, RECENTRE) set lastFrameMs; the
    watchdog stops the motors WATCHDOG_MS after the last one and counts wd_trips;
  * PING flashes but never feeds the watchdog;
  * DRIVE with no open segment stops the motors (drive_no_seg++) and does not feed the watchdog;
  * RECENTRE closes the segment and drives.
Every frame goes through SimBridge at the shortest blaster period, so frame lengths and decode times are the
payload-dependent ones of the real link.

These tests exercise the Python model sim.SimFirmware, not the C code: calib_bot.ino itself is covered only by
compiling it and by the static_assert protocol vectors. Keep the two in step by hand.
"""
import unittest

from graspion_calib import bridge as B
from graspion_calib import protocol as P
from graspion_calib import sim

from .simrig import BOT


class FirmwareCase(unittest.TestCase):
    def setUp(self):
        self.p = sim.SimParams(ir_loss=0.0)
        self.w = sim.SimWorld({BOT: (self.p, (0.5, 0.5, 0.0))}, seed=0)
        self.fw = self.w.fw[BOT]
        self.br = sim.SimBridge(self.w, period_ms=B.PERIOD_MIN_MS)

    def send(self, op, L=0, R=0):
        """One frame on the blaster's period grid; returns once the bot has handled it (still inside the period,
        so the next send starts one period after this one)."""
        rec = self.br.send(P.pack(BOT, op, L, R))
        self.w.advance(self.w.t + self.p.decode_delay)
        return rec

    def motors(self):
        return self.fw.L, self.fw.R


class TestWatchdogFeeds(FirmwareCase):
    def test_pings_do_not_feed_the_watchdog(self):
        """SEG(90, 90) then only PINGs every period: the motors stop ~300 ms after the SEG."""
        self.send(P.OP_SEG, 90, 90)
        t_seg = self.fw.last_frame
        self.assertEqual(self.motors(), (90, 90))
        for _ in range(8):                                           # 0.8 s of PINGs, well past the watchdog
            self.send(P.OP_PING)
        self.assertEqual(self.motors(), (0, 0))
        self.assertEqual(self.fw.stats["wd_trips"], 1)
        self.assertEqual(self.fw.last_frame, t_seg)                  # no PING touched the watchdog stamp
        self.w.advance(self.w.t + 0.5)
        plant = self.w.plants[BOT]
        self.assertLess(abs(plant.v) + abs(plant.w), 1e-3)

    def test_recentre_feeds_the_watchdog(self):
        """RECENTRE keeps the drive alive for longer than the watchdog; it trips ~300 ms after the last one."""
        self.send(P.OP_SEG, 80, 80)
        for _ in range(10):                                          # 1 s of nudging, one frame per period
            self.send(P.OP_RECENTRE, 70, 90)
            self.assertEqual(self.motors(), (70, 90))
        self.assertEqual(self.fw.stats["wd_trips"], 0)
        t_last = self.fw.last_frame
        self.w.advance(t_last + self.p.watchdog_s - 0.005)
        self.assertEqual(self.motors(), (70, 90))
        self.assertEqual(self.fw.stats["wd_trips"], 0)
        self.w.advance(t_last + self.p.watchdog_s + 0.005)
        self.assertEqual(self.motors(), (0, 0))
        self.assertEqual(self.fw.stats["wd_trips"], 1)

    def test_drive_in_segment_feeds_the_watchdog(self):
        self.send(P.OP_SEG, 80, 80)
        for k in range(6):                                           # 0.6 s of keep-alives and level changes
            self.send(P.OP_DRIVE, 80 + 5 * (k % 2), 80)
        self.assertEqual(self.motors(), (85, 80))
        self.assertEqual(self.fw.stats["wd_trips"], 0)
        self.w.advance(self.fw.last_frame + self.p.watchdog_s + 0.005)
        self.assertEqual(self.motors(), (0, 0))
        self.assertEqual(self.fw.stats["wd_trips"], 1)

    def test_drive_without_segment_stops(self):
        """RECENTRE then DRIVE without a segment stops at once and counts drive_no_seg."""
        self.send(P.OP_RECENTRE, 80, 80)
        self.assertEqual(self.motors(), (80, 80))
        t_feed = self.fw.last_frame
        self.send(P.OP_DRIVE, 90, 90)
        self.assertEqual(self.motors(), (0, 0))                     # stopped, not driven at (90, 90)
        self.assertEqual(self.fw.stats["drive_no_seg"], 1)
        self.assertEqual(self.fw.last_frame, t_feed)                 # no watchdog feed
        # a keep-alive after the watchdog ended the segment does not restart it either
        self.send(P.OP_SEG, 90, 90)
        self.w.advance(self.fw.last_frame + self.p.watchdog_s + 0.005)
        self.assertEqual(self.fw.stats["wd_trips"], 1)
        self.send(P.OP_DRIVE, 90, 90)
        self.assertEqual(self.motors(), (0, 0))
        self.assertEqual(self.fw.stats["drive_no_seg"], 2)
        self.w.advance(self.w.t + 0.5)
        plant = self.w.plants[BOT]
        self.assertLess(abs(plant.v) + abs(plant.w), 1e-3)

    def test_ping_flashes_without_motion(self):
        for _ in range(5):
            self.send(P.OP_PING)
        self.assertEqual(self.motors(), (0, 0))
        self.assertTrue(self.fw.led(self.fw.flash_until - 0.01))
        self.assertEqual(self.fw.stats["frames_ok"], 5)


class TestSimChannel(unittest.TestCase):
    """The simulated IR channel and blaster: per-frame NEC length, end-of-frame gap, period limits."""

    def setUp(self):
        self.p = sim.SimParams(ir_loss=0.0)
        self.w = sim.SimWorld({BOT: (self.p, (0.5, 0.5, 0.0))}, seed=0)
        self.fw = self.w.fw[BOT]

    def test_info_and_period_limits(self):
        br = sim.SimBridge(self.w)
        self.assertIn(f"proto={P.PROTO_VERSION}", br.info.split())
        with self.assertRaises(B.BridgeError):
            br.set_period(B.PERIOD_MIN_MS - 1)                       # frames would merge at the receiver
        with self.assertRaises(B.BridgeError):
            br.set_period(B.PERIOD_MAX_MS + 1)
        with self.assertRaises(B.BridgeError):
            sim.SimBridge(self.w, period_ms=70)
        br.set_period(B.PERIOD_MIN_MS)
        self.assertEqual(br.period_s, B.PERIOD_MIN_MS / 1000)

    def test_frame_length_and_decode_time(self):
        self.assertGreaterEqual(self.p.decode_delay, B.RECORD_GAP_S)
        br = sim.SimBridge(self.w)
        frames = [P.pack(BOT, P.OP_STOP), P.pack(BOT, P.OP_SEG, 90, 90), P.pack(BOT, P.OP_RECENTRE, 0, 90),
                  P.pack(BOT, P.OP_SEG, -120, 120)]
        lens = {B.nec_frame_s(raw) for raw in frames}
        self.assertGreater(len(lens), 2)                             # the payload sets the length
        for raw in frames:
            t_start = self.w.t + br.usb / 2                          # idle blaster: starts at once
            rec = br.send(raw)
            t_end = t_start + B.nec_frame_s(raw)
            self.assertGreaterEqual(rec.t_ack, t_end)                # the reply follows the frame
            self.assertAlmostEqual(rec.blaster_t1_ms - rec.blaster_t0_ms, B.nec_frame_s(raw) * 1000, delta=1.0)
            self.w.advance(self.w.t + 0.2)
            self.assertAlmostEqual(self.fw.last_frame, t_end + self.p.decode_delay, places=9)

    def test_back_to_back_frames_at_min_period_do_not_collide(self):
        br = sim.SimBridge(self.w, period_ms=B.PERIOD_MIN_MS)
        for _ in range(3):
            br.send(0xFFFFFFFF, check=False)                         # the longest frame (bad CRC: counted, ignored)
        self.w.advance(self.w.t + 0.1)
        self.assertEqual(self.w.collisions, 0)
        self.assertEqual(self.fw.stats["crc_err"], 3)

    def test_colliding_frames_are_both_lost(self):
        seg, ping = P.pack(BOT, P.OP_SEG, 90, 90), P.pack(BOT, P.OP_PING)
        t1 = B.nec_frame_s(seg)
        self.w.transmit(seg, 0.0)
        self.w.transmit(ping, t1 + B.RECORD_GAP_S - 0.001)          # inside the receiver's end-of-frame gap
        self.w.advance(0.5)
        self.assertEqual(self.w.collisions, 2)
        self.assertEqual(self.fw.stats["frames_ok"], 0)
        self.assertEqual((self.fw.L, self.fw.R), (0, 0))
        t0 = self.w.t
        self.w.transmit(seg, t0)
        self.w.transmit(ping, t0 + t1 + B.RECORD_GAP_S + 0.001)     # a real gap: both decode
        self.w.advance(t0 + 0.2)
        self.assertEqual(self.w.collisions, 2)
        self.assertEqual(self.fw.stats["frames_ok"], 2)


if __name__ == "__main__":
    unittest.main()
