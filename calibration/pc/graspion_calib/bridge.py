"""PC side of the IR blaster (blaster/ir_blaster): one request line, one reply line.

All timestamps are host time.perf_counter() seconds (the same clock as the tracker). t_ack is taken when the
blaster's "K ..." reply arrives, i.e. just after the frame has left the IR LED; the bot decodes it ~5 ms after
the last mark. The blaster enforces the minimum start-to-start period, so send() blocks ~68-110 ms.
"""
from __future__ import annotations

import csv
import time
from typing import NamedTuple

from .protocol import OP_NAMES, unpack


class BridgeError(RuntimeError):
    pass


class SendRecord(NamedTuple):
    t_req: float
    t_ack: float
    raw: int
    blaster_t0_ms: int
    blaster_t1_ms: int


class SerialBridge:
    def __init__(self, port, baud=115200, period_ms=110, timeout_s=1.0, reset_wait_s=2.0):
        try:
            import serial  # pyserial
        except ImportError as e:
            raise BridgeError("pyserial is needed for the real blaster: pip install pyserial") from e
        self._ser = serial.Serial(port, baud, timeout=timeout_s)
        time.sleep(reset_wait_s)                  # a Nano resets when the port opens
        self._ser.reset_input_buffer()
        self.info = self.request("V")
        self.set_period(period_ms)

    def request(self, line: str) -> str:
        self._ser.write((line + "\n").encode("ascii"))
        reply = self._ser.readline().decode("ascii", "replace").strip()
        if not reply:
            raise BridgeError(f"no reply to {line!r} (blaster unplugged or busy?)")
        if reply.startswith("E"):
            raise BridgeError(f"blaster rejected {line!r}: {reply}")
        return reply

    def set_period(self, period_ms: int):
        self.request(f"G {int(period_ms)}")
        self.period_s = period_ms / 1000.0

    def send(self, raw: int, check=True) -> SendRecord:
        t_req = time.perf_counter()
        reply = self.request(f"{'F' if check else 'f'} {raw:08X}")
        t_ack = time.perf_counter()
        parts = reply.split()
        if len(parts) != 4 or parts[0] != "K" or int(parts[1], 16) != raw:
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
