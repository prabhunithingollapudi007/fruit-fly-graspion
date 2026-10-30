"""IR frame format, mirrored from firmware/calib_bot/calib_protocol.h (keep both in sync).

One NEC-timed 32-bit frame, LSB first (IRremote ``sendNECRaw`` / ``decodedRawData``)::

    byte 0  bot ID 1..254 (0xFF = broadcast, STOP only)
    byte 1  op (3 high bits) | CRC-5 check bits (5 low bits)
    byte 2  L code = 128 + drive_L   (drive -127..127, code 128 = stop, code 0 invalid)
    byte 3  R code = 128 + drive_R

The CRC-5 runs over the whole frame in on-air order (raw bit 0 first), so its guarantees hold for physical
errors: every single-bit error, every burst of up to 5 consecutive on-air bits, and every 2-bit error except
raw bits 0 + 31 (first and last on air) is detected. See ``crc5_air`` and calib_protocol.h.

``python -m graspion_calib.protocol --write-vectors`` regenerates ``calib_protocol_vectors.h`` in both sketch
folders; the sketches ``static_assert`` those vectors, so a successful firmware compile proves that the C
and Python implementations agree.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass
from pathlib import Path

PROTO_VERSION = 2          # 2: CRC in on-air bit order; builds of version 1 and 2 reject each other's frames
ADDR_BROADCAST = 0xFF

OP_STOP = 0
OP_SEG = 1
OP_DRIVE = 2
OP_RECENTRE = 3
OP_PING = 4
OP_NAMES = {OP_STOP: "STOP", OP_SEG: "SEG", OP_DRIVE: "DRIVE", OP_RECENTRE: "RECENTRE", OP_PING: "PING"}

OK = 0
ERR_CODE = -1
ERR_CRC = -2

DRIVE_LIMIT = 127


def clamp_drive(d: int) -> int:
    return max(-DRIVE_LIMIT, min(DRIVE_LIMIT, int(d)))


def code(d: int) -> int:
    return 128 + clamp_drive(d)


def _crc5_bit(crc: int, b: int) -> int:
    fb = ((crc >> 4) ^ b) & 1
    return ((crc << 1) ^ 0x05) & 0x1F if fb else (crc << 1) & 0x1F


def crc5_air(raw: int, crc: int = 0x1F) -> int:
    """CRC-5 register (poly 0x05, init 0x1F, MSB-first register, no reflection, no final XOR) after the 32 frame
    bits in on-air order, raw bit 0 first. 0 for a valid frame: read in that order, the frame is a code word."""
    for i in range(32):
        crc = _crc5_bit(crc, (raw >> i) & 1)
    return crc


def _rev5(v: int) -> int:
    return ((v & 1) << 4) | ((v & 2) << 2) | (v & 4) | ((v & 8) >> 2) | ((v & 16) >> 4)


def crc_field(data: int) -> int:
    """Check bits for raw bits 8..12 of ``data`` (a frame with those bits 0): the one value that makes
    ``crc5_air`` of the whole frame 0. The register over ``data`` is the remainder to cancel. A check value c
    enters the register as c * x^24 (raw bit 12 is x^24), so c = remainder * x^-24 = remainder * x^7 (mod the
    polynomial, period 31): 7 zero steps. Raw bit 8 goes out first, so it takes c's top bit (bit-reversed)."""
    crc = crc5_air(data)
    for _ in range(7):
        crc = _crc5_bit(crc, 0)
    return _rev5(crc)


def pack(bot_id: int, op: int, L: int = 0, R: int = 0) -> int:
    if not 0 <= bot_id <= 0xFF:
        raise ValueError(f"bot id {bot_id} out of range")
    if not 0 <= op <= 7:
        raise ValueError(f"op {op} out of range")
    data = (bot_id & 0xFF) | ((op & 7) << 13) | (code(L) << 16) | (code(R) << 24)
    return data | (crc_field(data) << 8)


@dataclass(frozen=True)
class Frame:
    bot_id: int
    op: int
    L: int
    R: int

    @property
    def raw(self) -> int:
        return pack(self.bot_id, self.op, self.L, self.R)

    def __str__(self) -> str:
        return f"{OP_NAMES.get(self.op, self.op)}(id={self.bot_id}, L={self.L}, R={self.R})"


def check(raw: int) -> int:
    lc, rc = (raw >> 16) & 0xFF, (raw >> 24) & 0xFF
    if lc == 0 or rc == 0:
        return ERR_CODE
    return OK if crc5_air(raw) == 0 else ERR_CRC


def unpack(raw: int) -> Frame:
    r = check(raw)
    if r != OK:
        raise ValueError(f"invalid frame 0x{raw:08X} ({'code' if r == ERR_CODE else 'crc'})")
    return Frame(raw & 0xFF, (raw >> 13) & 7, ((raw >> 16) & 0xFF) - 128, ((raw >> 24) & 0xFF) - 128)


# ----------------------------------------------------------------------------- bot identity
def _fnv_byte(h: int, b: int) -> int:
    return ((h ^ (b & 0xFF)) * 16777619) & 0xFFFFFFFF


def serial_hash(words) -> int:
    h = 2166136261
    for w in words:
        for shift in (24, 16, 8, 0):
            h = _fnv_byte(h, w >> shift)
    return h


def bot_id_from_words(words) -> int:
    h = serial_hash(words)
    fold = (h ^ (h >> 8) ^ (h >> 16) ^ (h >> 24)) & 0xFF
    return 1 + fold % 254


def words_from_usb_serial(serial: str):
    """USB iSerial string (32 hex chars, Adafruit TinyUSB SAMD21 port) -> the four serial words."""
    s = serial.strip()
    if len(s) != 32:
        raise ValueError(f"expected 32 hex chars, got {len(s)}: {serial!r}")
    return [int(s[i:i + 8], 16) for i in range(0, 32, 8)]


def bot_id_from_usb_serial(serial: str) -> int:
    return bot_id_from_words(words_from_usb_serial(serial))


# ----------------------------------------------------------------------------- device info
def info_fields(line: str) -> dict:
    """key=value fields of an info line: the bot's 'i' reply or the blaster's 'V' reply ("I ... proto=2 ...")."""
    return dict(kv.split("=", 1) for kv in line.split() if "=" in kv)


def proto_error(info, device: str, sketch: str) -> str | None:
    """None if info (an info line, or its fields as a dict) reports PROTO_VERSION; else why `device` must be
    reflashed with `sketch`. Builds of different protocol versions reject each other's frames as CRC errors."""
    proto = (info_fields(info) if isinstance(info, str) else info).get("proto")
    if proto == str(PROTO_VERSION):
        return None
    return (f"{device} reports IR protocol {proto if proto is not None else '(none)'}, this PC code protocol "
            f"{PROTO_VERSION}: each side would reject the other's frames as CRC errors. Reflash {sketch} from this "
            f"repository (calib_protocol.h with CALIB_PROTO_VERSION {PROTO_VERSION})")


# ----------------------------------------------------------------------------- C test vectors
VECTOR_FRAMES = [
    (1, OP_STOP, 0, 0), (0x5A, OP_SEG, 60, 80), (0x5A, OP_DRIVE, -90, 90), (254, OP_RECENTRE, 0, 100),
    (17, OP_PING, 0, 0), (ADDR_BROADCAST, OP_STOP, 0, 0), (200, OP_SEG, 127, -127), (3, OP_SEG, 300, -300),
    (128, OP_DRIVE, 55, 120), (77, OP_RECENTRE, 120, 0),
]
# On-air error patterns XORed into DRIVE(0x5A, 80, 80): the 2..5-bit bursts that protocol 1 missed, then raw
# bits 0 + 31, the one 2-bit error the CRC cannot see. Pins the on-air bit order of calib_check.
VECTOR_ERRORS = [0x180, 0x3800, 0x1600000, 0x2C00000, 0x3A00000, 0x80000001]
_C_RESULT = {OK: "CALIB_OK", ERR_CODE: "CALIB_ERR_CODE", ERR_CRC: "CALIB_ERR_CRC"}
VECTOR_SERIALS = [
    (0x00000000, 0x00000000, 0x00000000, 0x00000000),
    (0x12345678, 0x9ABCDEF0, 0x0F1E2D3C, 0x4B5A6978),
    (0xDEADBEEF, 0x01234567, 0x89ABCDEF, 0xFEDCBA98),
    (0x5A4B3C2D, 0x1E0F0011, 0x22334455, 0x66778899),
]


def vectors_header() -> str:
    lines = [
        "// calib_protocol_vectors.h -- GENERATED by `python -m graspion_calib.protocol --write-vectors`.",
        "// Do not edit. Compile-time cross-check of calib_protocol.h against pc/graspion_calib/protocol.py.",
        "#pragma once",
        '#include "calib_protocol.h"',
        f"static_assert(CALIB_PROTO_VERSION == {PROTO_VERSION}, \"protocol version mismatch\");",
    ]
    for bot_id, op, L, R in VECTOR_FRAMES:
        raw = pack(bot_id, op, L, R)
        lines.append(f"static_assert(calib_pack({bot_id}, {op}, {L}, {R}) == 0x{raw:08X}UL, \"pack\");")
        lines.append(f"static_assert(calib_check(0x{raw:08X}UL) == CALIB_OK, \"check ok\");")
        lines.append(f"static_assert(calib_check(0x{raw ^ (1 << 20):08X}UL) != CALIB_OK, \"check flip\");")
    lines.append(f"static_assert(calib_check(0x{pack(9, OP_SEG, 10, 10) & 0xFF00FFFF:08X}UL) == CALIB_ERR_CODE, \"code 0\");")
    base = pack(0x5A, OP_DRIVE, 80, 80)
    for e in VECTOR_ERRORS:
        r = check(base ^ e)
        lines.append(f"static_assert(calib_check(0x{base ^ e:08X}UL) == {_C_RESULT[r]}, "
                     f"\"{'on-air blind spot' if r == OK else 'on-air burst'}\");")
    for w in VECTOR_SERIALS:
        args = ", ".join(f"0x{x:08X}UL" for x in w)
        lines.append(f"static_assert(calib_serial_hash({args}) == 0x{serial_hash(w):08X}UL, \"fnv\");")
        lines.append(f"static_assert(calib_bot_id({args}) == {bot_id_from_words(w)}, \"bot id\");")
    return "\n".join(lines) + "\n"


SKETCH_DIRS = ("firmware/calib_bot",)


def calibration_root() -> Path:
    return Path(__file__).resolve().parents[2]


def write_vectors(root: Path | None = None) -> list[Path]:
    root = root or calibration_root()
    out = []
    text = vectors_header()
    for d in SKETCH_DIRS:
        p = root / d / "calib_protocol_vectors.h"
        p.write_text(text, encoding="ascii", newline="\n")
        out.append(p)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--write-vectors", action="store_true", help="regenerate calib_protocol_vectors.h")
    ap.add_argument("--pack", nargs=4, metavar=("ID", "OP", "L", "R"), type=int, help="print a raw frame")
    ap.add_argument("--id-from-serial", metavar="HEX32", help="bot ID from a USB iSerial string")
    a = ap.parse_args(argv)
    if a.write_vectors:
        for p in write_vectors():
            print("wrote", p)
    if a.pack:
        print(f"0x{pack(*a.pack):08X}")
    if a.id_from_serial:
        print(bot_id_from_usb_serial(a.id_from_serial))


if __name__ == "__main__":
    main()
