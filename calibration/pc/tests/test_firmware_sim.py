"""SimFirmware against calib_bot.ino's frame handling, and the simulated IR channel and blaster (no hardware).

The firmware rules pinned down here (calib_bot.ino handleFrame / commandWatchdog / logPing):
  * only frames that command the drive (STOP, SEG, DRIVE inside a segment, RECENTRE) set lastFrameMs; the
    watchdog stops the motors WATCHDOG_MS after the last one, closes an open record (END_WATCHDOG), wd_trips++;
  * PING flashes and logs but never feeds the watchdog; inside an open record it counts as one of its frames,
    otherwise a run of PINGs shares one REC_PING record (n_frames, t_start_ms = first, t_end_ms = last);
  * DRIVE with no open segment stops the motors (drive_no_seg++), does not feed the watchdog and logs nothing;
  * RECENTRE closes the record (END_RECENTRE) and drives, unlogged.
Every frame goes through SimBridge at the shortest blaster period, so frame lengths and decode times are the
payload-dependent ones of the real link.

These tests exercise the Python model sim.SimFirmware, not the C code: calib_bot.ino itself is covered only by
compiling it and by the static_assert protocol vectors. Keep the two in step by hand.
"""
import unittest

from graspion_calib import bridge as B
from graspion_calib import protocol as P
from graspion_calib import sim
from graspion_calib.botlog import END_REASONS, parse_dump

from .simrig import BOT

END = {name: code for code, name in END_REASONS.items()}
REC_SEG, REC_PING = 1, 2


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

    def bot_s(self, rec, a="t_start_ms", b="t_end_ms"):
        """Host seconds between two bot-clock stamps of a log record (the simulated bot clock runs fast)."""
        return (rec[b] - rec[a]) / (1 + self.p.clock_err) / 1000

    def motors(self):
        return self.fw.L, self.fw.R


class TestWatchdogFeeds(FirmwareCase):
    def test_pings_do_not_feed_the_watchdog(self):
        """README B.2: SEG(90, 90) then only PINGs every period: the motors stop ~300 ms after the SEG."""
        self.send(P.OP_SEG, 90, 90)
        t_seg = self.fw.last_frame
        self.assertEqual(self.motors(), (90, 90))
        n_pings = 8                                                  # 0.8 s of PINGs, well past the watchdog
        for _ in range(n_pings):
            self.send(P.OP_PING)
        self.assertEqual(self.motors(), (0, 0))
        self.assertEqual(self.fw.stats["wd_trips"], 1)
        self.assertEqual(self.fw.last_frame, t_seg)                  # no PING touched the watchdog stamp
        self.assertEqual(len(self.fw.log), 2)
        seg, ping = self.fw.log
        self.assertEqual((seg["kind"], seg["end_reason"]), (REC_SEG, END["watchdog"]))
        self.assertAlmostEqual(self.bot_s(seg), self.p.watchdog_s, delta=0.005)   # after the SEG, not the PINGs
        # PINGs before the trip are frames of the open segment; the rest form one PING record after it
        self.assertEqual((ping["kind"], ping["end_reason"]), (REC_PING, END["ping"]))
        self.assertGreaterEqual(seg["n_frames"], 2)
        self.assertGreaterEqual(ping["n_frames"], 2)
        self.assertEqual(seg["n_frames"] + ping["n_frames"], 1 + n_pings)
        self.w.advance(self.w.t + 0.5)
        plant = self.w.plants[BOT]
        self.assertLess(abs(plant.v) + abs(plant.w), 1e-3)

    def test_recentre_feeds_the_watchdog(self):
        """RECENTRE keeps the drive alive for longer than the watchdog; it trips ~300 ms after the last one."""
        self.send(P.OP_SEG, 80, 80)
        n = 10                                                       # 1 s of re-centring, one frame per period
        for _ in range(n):
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
        # the first RECENTRE closed the segment; RECENTRE itself is not logged, so the trip leaves the log alone
        self.assertEqual(len(self.fw.log), 1)
        self.assertEqual((self.fw.log[0]["end_reason"], self.fw.log[0]["n_frames"]), (END["recentre"], 1))

    def test_drive_in_segment_feeds_the_watchdog(self):
        self.send(P.OP_SEG, 80, 80)
        for k in range(6):                                           # 0.6 s of keep-alives and level changes
            self.send(P.OP_DRIVE, 80 + 5 * (k % 2), 80)
        self.assertEqual(self.motors(), (85, 80))
        self.assertEqual(self.fw.stats["wd_trips"], 0)
        self.w.advance(self.fw.last_frame + self.p.watchdog_s + 0.005)
        self.assertEqual(self.motors(), (0, 0))
        rec = self.fw.log[-1]
        self.assertEqual((rec["end_reason"], rec["n_frames"]), (END["watchdog"], 7))

    def test_drive_without_segment_stops(self):
        """README B.2: RECENTRE then DRIVE without a segment stops at once and counts drive_no_seg."""
        self.send(P.OP_RECENTRE, 80, 80)
        self.assertEqual(self.motors(), (80, 80))
        t_feed = self.fw.last_frame
        self.send(P.OP_DRIVE, 90, 90)
        self.assertEqual(self.motors(), (0, 0))                     # stopped, not driven at (90, 90)
        self.assertEqual(self.fw.stats["drive_no_seg"], 1)
        self.assertEqual(self.fw.last_frame, t_feed)                 # no watchdog feed
        self.assertEqual(self.fw.log, [])                            # and no record
        # a keep-alive after the watchdog ended the segment does not restart it either
        self.send(P.OP_SEG, 90, 90)
        self.w.advance(self.fw.last_frame + self.p.watchdog_s + 0.005)
        self.assertEqual(self.fw.stats["wd_trips"], 1)
        self.send(P.OP_DRIVE, 90, 90)
        self.assertEqual(self.motors(), (0, 0))
        self.assertEqual(self.fw.stats["drive_no_seg"], 2)
        self.assertEqual([r["end_reason"] for r in self.fw.log], [END["watchdog"]])
        self.w.advance(self.w.t + 0.5)
        plant = self.w.plants[BOT]
        self.assertLess(abs(plant.v) + abs(plant.w), 1e-3)


class TestPingRecords(FirmwareCase):
    def test_ping_run_shares_one_record(self):
        for _ in range(5):
            self.send(P.OP_PING)
        self.assertEqual(self.motors(), (0, 0))                     # PINGs never move the bot
        self.assertEqual(len(self.fw.log), 1)
        r = self.fw.log[0]
        self.assertEqual((r["kind"], r["n_frames"], r["end_reason"]), (REC_PING, 5, END["ping"]))
        self.assertGreater(r["t_end_ms"], r["t_start_ms"])
        self.assertAlmostEqual(self.bot_s(r), 4 * self.br.period_s, delta=0.003)  # first to last PING

    def test_seg_starts_a_new_ping_run(self):
        for _ in range(3):
            self.send(P.OP_PING)
        self.send(P.OP_SEG, 80, 80)
        self.send(P.OP_PING)                                         # inside the segment: one of its frames
        self.send(P.OP_STOP)
        for _ in range(2):
            self.send(P.OP_PING)
        self.assertEqual([r["kind"] for r in self.fw.log], [REC_PING, REC_SEG, REC_PING])
        self.assertEqual([r["n_frames"] for r in self.fw.log], [3, 2, 2])
        self.assertEqual([r["end_reason"] for r in self.fw.log], [END["ping"], END["stop"], END["ping"]])
        self.assertEqual(self.fw.stats["wd_trips"], 0)
        # the dump round trip keeps the runs apart, as botlog.py reads them
        info, stats, recs = parse_dump(self.fw.dump_text())
        self.assertEqual([(r["kind"], r["n_frames"]) for r in recs], [(2, 3), (1, 2), (2, 2)])
        self.assertEqual(info["proto"], str(P.PROTO_VERSION))
        self.assertEqual(int(info["watchdog_ms"]), round(self.p.watchdog_s * 1000))
        self.assertEqual(int(info["nrec"]), 3)


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
        self.assertEqual(self.fw.stats["rx_ok"], 0)
        self.assertEqual(self.fw.log, [])
        self.assertEqual(self.motors(), (0, 0))
        t0 = self.w.t
        self.w.transmit(seg, t0)
        self.w.transmit(ping, t0 + t1 + B.RECORD_GAP_S + 0.001)     # a real gap: both decode
        self.w.advance(t0 + 0.2)
        self.assertEqual(self.w.collisions, 2)
        self.assertEqual(self.fw.stats["rx_ok"], 2)
        self.assertEqual(self.fw.log[0]["n_frames"], 2)              # the SEG and the PING inside it

    def motors(self):
        return self.fw.L, self.fw.R


if __name__ == "__main__":
    unittest.main()
