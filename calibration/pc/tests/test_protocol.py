import re
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

    def test_double_bit_errors_mostly_detected(self):
        rng = np.random.default_rng(2)
        missed = total = 0
        for _ in range(200):
            raw = P.pack(int(rng.integers(1, 255)), int(rng.integers(0, 5)), int(rng.integers(-120, 121)),
                         int(rng.integers(-120, 121)))
            for a in range(32):
                for b in range(a + 1, 32):
                    total += 1
                    missed += P.check(raw ^ (1 << a) ^ (1 << b)) == P.OK
        self.assertLess(missed / total, 1.5 / 496)       # at most 1 of the 496 pairs per frame

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

    def test_header_copies_identical(self):
        a = (ROOT / "firmware/calib_bot/calib_protocol.h").read_bytes()
        b = (ROOT / "blaster/ir_blaster/calib_protocol.h").read_bytes()
        self.assertEqual(a, b, "calib_protocol.h copies differ")

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

    def test_dump_columns_match_firmware(self):
        from graspion_calib.sim import SimFirmware
        ino = (ROOT / "firmware/calib_bot/calib_bot.ino").read_text()
        cols = SimFirmware.DUMP_COLUMNS
        flat = re.sub(r'"\s*\n\s*"', "", ino)          # join adjacent C string literals
        self.assertIn(cols, flat, "dump header in calib_bot.ino differs from sim/botlog")


if __name__ == "__main__":
    unittest.main()
