import unittest
from pathlib import Path

import numpy as np

from graspion_calib import protocol as P

ROOT = Path(__file__).resolve().parents[2]


class TestProtocol(unittest.TestCase):
    def test_roundtrip(self):
        rng = np.random.default_rng(0)
        for _ in range(2000):
            bid, op = int(rng.integers(0, 256)), int(rng.integers(0, 8))
            L, R = int(rng.integers(-127, 128)), int(rng.integers(-127, 128))
            f = P.unpack(P.pack(bid, op, L, R))
            self.assertEqual((f.bot_id, f.op, f.L, f.R), (bid, op, L, R))

    def test_stop_is_code_128(self):
        raw = P.pack(5, P.OP_SEG, 0, 0)
        self.assertEqual((raw >> 16) & 0xFF, 128)
        self.assertEqual(raw >> 24, 128)

    def test_clamp(self):
        f = P.unpack(P.pack(3, P.OP_SEG, 500, -500))
        self.assertEqual((f.L, f.R), (127, -127))

    def test_every_single_bit_error_detected(self):
        rng = np.random.default_rng(1)
        for _ in range(300):
            raw = P.pack(int(rng.integers(1, 255)), int(rng.integers(0, 5)), int(rng.integers(-120, 121)),
                         int(rng.integers(-120, 121)))
            for b in range(32):
                self.assertNotEqual(P.check(raw ^ (1 << b)), P.OK, f"flip {b} of {raw:08X} undetected")

    def test_layout_and_codeword(self):
        raw = P.pack(0x5A, P.OP_DRIVE, -90, 90)
        self.assertEqual((raw & 0xFF, (raw >> 13) & 7, (raw >> 16) & 0xFF, raw >> 24), (0x5A, P.OP_DRIVE, 38, 218))
        self.assertEqual(P.crc5_air(raw), 0)             # the whole frame, in on-air order, is a code word
        field = (raw >> 8) & 0x1F
        others = [v for v in range(32) if P.crc5_air((raw & ~0x1F00) | (v << 8)) == 0]
        self.assertEqual(others, [field])                # exactly one check value per data word

    # On-air order: IRremote sendNECRaw sends raw bit 0 first, so a physical burst of n flipped bits is n adjacent
    # raw bit indices. The code is linear: raw ^ e passes the CRC iff crc5_air(e, 0) == 0, whatever the frame.
    @staticmethod
    def _bursts(max_len):
        for n in range(1, max_len + 1):
            for start in range(33 - n):
                for inner in range(1 << max(n - 2, 0)):
                    b = 1 if n == 1 else 1 | (inner << 1) | (1 << (n - 1))
                    yield n, b << start

    def test_every_on_air_burst_up_to_5_bits_detected(self):
        bursts = list(self._bursts(5))
        self.assertEqual(len(bursts), 32 + 31 + 30 * 2 + 29 * 4 + 28 * 8)
        for n, e in bursts:                              # exhaustive, frame-independent
            self.assertNotEqual(P.crc5_air(e, 0), 0, f"{n}-bit burst {e:08X} undetected")
        rng = np.random.default_rng(4)
        for _ in range(100):                             # and through check() on real frames
            raw = P.pack(int(rng.integers(0, 256)), int(rng.integers(0, 8)), int(rng.integers(-127, 128)),
                         int(rng.integers(-127, 128)))
            for n, e in bursts:
                self.assertNotEqual(P.check(raw ^ e), P.OK, f"{n}-bit burst {e:08X} on {raw:08X} undetected")

    def test_burst_bound_is_tight(self):
        # x^5 + x^2 + 1 laid out in on-air order is a 6-bit burst at raw bits p, p+3, p+5: undetected anywhere.
        for p in range(27):
            self.assertEqual(P.crc5_air(0x29 << p, 0), 0)
        sixes = [e for n, e in self._bursts(6) if n == 6]
        self.assertEqual(sum(P.crc5_air(e, 0) == 0 for e in sixes) * 16, len(sixes))   # 1 in 16 escape

    def test_protocol1_escapes_now_detected(self):
        # Short on-air bursts the protocol-1 CRC (computed in another bit order than the air) let through.
        raw = P.pack(0x5A, P.OP_DRIVE, 80, 80)
        for e in (0x180, 0x3800, 0x1600000, 0x2C00000, 0x3A00000):
            self.assertEqual(P.check(raw ^ e), P.ERR_CRC, f"{e:08X}")

    def test_double_bit_errors(self):
        # All 496 pairs: the only undetected one is raw bits 0 + 31, the first and last bit on air.
        missed = [(a, b) for a in range(32) for b in range(a + 1, 32) if P.crc5_air((1 << a) | (1 << b), 0) == 0]
        self.assertEqual(missed, [(0, 31)])
        rng = np.random.default_rng(2)
        for _ in range(100):
            raw = P.pack(int(rng.integers(1, 255)), int(rng.integers(0, 5)), int(rng.integers(-120, 121)),
                         int(rng.integers(-120, 121)))
            for a in range(32):
                for b in range(a + 1, 32):
                    if (a, b) != (0, 31):
                        self.assertNotEqual(P.check(raw ^ (1 << a) ^ (1 << b)), P.OK,
                                            f"pair {a},{b} of {raw:08X}")

    def test_bot_id_range_and_usb_serial(self):
        rng = np.random.default_rng(3)
        ids = []
        for _ in range(500):
            words = [int(w) for w in rng.integers(0, 2 ** 32, 4, dtype=np.uint64)]
            s = "".join(f"{w:08X}" for w in words)
            i = P.bot_id_from_usb_serial(s)
            self.assertEqual(i, P.bot_id_from_words(words))
            self.assertTrue(1 <= i <= 254)
            ids.append(i)
        self.assertGreater(len(set(ids)), 200)          # spread over the range

    def test_fnv_reference_value(self):
        # FNV-1a 32 of 16 zero bytes, computed independently
        h = 2166136261
        for _ in range(16):
            h = (h * 16777619) & 0xFFFFFFFF
        self.assertEqual(P.serial_hash([0, 0, 0, 0]), h)

    def test_vectors_up_to_date(self):
        text = P.vectors_header()
        for d in P.SKETCH_DIRS:
            self.assertEqual((ROOT / d / "calib_protocol_vectors.h").read_text(), text,
                             f"{d}/calib_protocol_vectors.h is stale: run python -m graspion_calib.protocol --write-vectors")

    def test_c_constants_match(self):
        h = (ROOT / "firmware/calib_bot/calib_protocol.h").read_text()
        for name, val in (("CALIB_OP_STOP", P.OP_STOP), ("CALIB_OP_SEG", P.OP_SEG), ("CALIB_OP_DRIVE", P.OP_DRIVE),
                          ("CALIB_OP_RECENTRE", P.OP_RECENTRE), ("CALIB_OP_PING", P.OP_PING),
                          ("CALIB_ADDR_BROADCAST", P.ADDR_BROADCAST), ("CALIB_PROTO_VERSION", P.PROTO_VERSION)):
            line = next(ln for ln in h.splitlines() if ln.startswith(f"#define {name} "))
            self.assertEqual(int(line.split()[2], 0), val, name)


if __name__ == "__main__":
    unittest.main()
