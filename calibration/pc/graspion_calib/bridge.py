"""PC side of the IR blaster (bot_blaster/bot_blaster.ino): one request line, one reply line.

All timestamps are host time.perf_counter() seconds (the same clock as the tracker). t_ack is taken when the
blaster's "K ..." reply arrives, i.e. just after the frame has left the IR LED. The bot's IRremote receiver ends a
frame only after RECORD_GAP_S of silence, so it decodes the frame >= 8 ms after the last mark.
A frame takes 50-86 ms: without the NEC inverse bytes its length depends on the number of 1 bits (nec_frame_s).
The blaster enforces the minimum start-to-start period, so send() blocks for up to about one period.
"""
from __future__ import annotations

import csv
import time
from typing import NamedTuple

from .protocol import OP_NAMES, proto_error, unpack

RECORD_GAP_S = 0.008             # IRremote 4.7.1 RECORD_GAP_MICROS: silence that ends a received frame
PERIOD_MIN_MS = 100              # longest frame (85.7 ms) + RECORD_GAP_S + margin; the blaster rejects less
PERIOD_MAX_MS = 2000
RESYNC = b"\x18\n"               # CAN: the blaster drops the partial line unrun; the empty line gets no reply


def nec_frame_s(raw: int) -> float:
    """On-air length of one sendNECRaw frame (IRremote 4.7.1 NEC timing, 560 us unit): 9 ms + 4.5 ms header,
    32 x (560 us mark + 560 or 1680 us space), 560 us stop mark = 49.84 ms + 1.12 ms per 1 bit."""
    return 0.04984 + 0.00112 * bin(raw & 0xFFFFFFFF).count("1")


NEC_FRAME_MAX_S = nec_frame_s(0xFFFFFFFF)   # 85.68 ms


class BridgeError(RuntimeError):
    pass


class SendRecord(NamedTuple):
    t_req: float
    t_ack: float
    raw: int
    blaster_t0_ms: int
    blaster_t1_ms: int


class SerialBridge:
    """ser: an already open serial-like object, otherwise pyserial opens port. now / sleep: the host clock
    (time.perf_counter / time.sleep); tests pass simulated ones together with a fake blaster as ser.
    Opening asks the blaster for 'V' and raises BridgeError unless it reports this code's protocol.PROTO_VERSION
    (frames of another protocol version fail the CRC on the bot)."""

    def __init__(self, port, baud=115200, period_ms=110, timeout_s=1.0, reset_wait_s=2.0, ser=None, now=None,
                 sleep=None):
        if ser is None:
            try:
                import serial  # pyserial
            except ImportError as e:
                raise BridgeError("pyserial is needed for the real blaster: pip install pyserial") from e
            ser = serial.Serial(port, baud, timeout=timeout_s)
        self._ser = ser
        self._now = now or time.perf_counter
        self._sleep = sleep or time.sleep
        self._timeout_min = timeout_s
        self.period_s = 0.11                      # the blaster's default until set_period
        self._sleep(reset_wait_s)                 # a Nano resets when the port opens
        try:
            self.resync(0.3)                      # a native-USB blaster does not, and may hold a partial line
            self.info = self.request("V", expect="I ")
            err = proto_error(self.info, "the IR blaster", "ir_blaster")
            if err:
                raise BridgeError(err)
            self.set_period(period_ms)
        except BaseException:
            self._ser.close()
            raise

    def request(self, line: str, expect: str | None = None) -> str:
        self._ser.write((line + "\n").encode("ascii"))
        reply = self._ser.readline().decode("ascii", "replace").strip()
        if not reply:
            raise BridgeError(f"no reply to {line!r} (blaster unplugged or busy?)")
        if reply.startswith("E"):
            raise BridgeError(f"blaster rejected {line!r}: {reply}")
        if expect is not None and not reply.startswith(expect):
            raise BridgeError(f"unexpected reply {reply!r} to {line!r} (out of step with the blaster?)")
        return reply

    def resync(self, wait_s: float | None = None):
        """Drop what an interrupted exchange left behind: a reply still on its way (the blaster may be waiting for
        its period slot and then sending, up to one period plus one frame) or half read, and a partial request line
        in the blaster. RESYNC's CAN byte makes ir_blaster drop that line unrun (a bare newline would complete it,
        and a stale 'F <SEG>' would go out); the empty line after it gets no reply. (An ir_blaster/2 build keeps
        CAN in the line, so a stale frame request fails to parse there; that reply is flushed with the rest.)"""
        self._ser.write(RESYNC)
        self._sleep(self.period_s + NEC_FRAME_MAX_S + 0.05 if wait_s is None else wait_s)
        self._ser.reset_input_buffer()

    def set_period(self, period_ms: int):
        period_ms = int(period_ms)
        if not PERIOD_MIN_MS <= period_ms <= PERIOD_MAX_MS:
            raise BridgeError(f"blaster period {period_ms} ms outside {PERIOD_MIN_MS}..{PERIOD_MAX_MS} ms "
                              f"(a frame takes up to {NEC_FRAME_MAX_S * 1000:.1f} ms, and the receiver needs "
                              f"{RECORD_GAP_S * 1000:.0f} ms of silence after it)")
        self.request(f"G {period_ms}", expect="K G")
        self.period_s = period_ms / 1000.0
        # a frame's reply comes after the blaster's period slot and the frame itself: wait longer than that
        self._ser.timeout = max(self._timeout_min, self.period_s + NEC_FRAME_MAX_S + 0.5)

    def send(self, raw: int, check=True) -> SendRecord:
        t_req = self._now()
        reply = self.request(f"{'F' if check else 'f'} {raw:08X}")
        t_ack = self._now()
        parts = reply.split()
        if len(parts) != 4 or parts[0] != "K" or parts[1].upper() != f"{raw:08X}":
            raise BridgeError(f"unexpected reply {reply!r} to frame {raw:08X}")
        return SendRecord(t_req, t_ack, raw, int(parts[2]), int(parts[3]))

    def close(self):
        self._ser.close()


class FrameLog:
    """frames.csv: every frame the session sent, with host times. The analysis rebuilds commands from it."""
    COLUMNS = ["t_req", "t_ack", "raw", "op", "bot", "L", "R", "seg", "purpose"]

    def __init__(self, path):
        self._f = open(path, "w", newline="")
        self._w = csv.writer(self._f)
        self._w.writerow(self.COLUMNS)

    def write(self, rec: SendRecord, seg: int, purpose: str):
        try:
            fr = unpack(rec.raw)
            op, bot, L, R = OP_NAMES.get(fr.op, fr.op), fr.bot_id, fr.L, fr.R
        except ValueError:
            op, bot, L, R = "INVALID", rec.raw & 0xFF, "", ""
        self._w.writerow([f"{rec.t_req:.6f}", f"{rec.t_ack:.6f}", f"{rec.raw:08X}", op, bot, L, R, seg, purpose])
        self._f.flush()

    def close(self):
        self._f.close()
