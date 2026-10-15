// calib_protocol.h -- IR frame format shared by calib_bot (receiver) and ir_blaster (sender).
// Mirrored in pc/graspion_calib/protocol.py. Keep the copies in firmware/calib_bot/ and blaster/ir_blaster/
// byte-identical (pc/tests/test_protocol.py checks this). C++11 constexpr so calib_protocol_vectors.h can
// static_assert the Python-generated test vectors at compile time on both targets.
//
// One frame = one NEC-timed 32-bit frame (IRremote sendNECRaw / decodedRawData, LSB first on air):
//
//   byte 0 (bits  0..7)  bot ID: 1..254 (0xFF = broadcast, STOP only; 0x00 left to consumer remotes)
//   byte 1 (bits  8..15) op (3 bits, high) | CRC-5 check bits (5 bits, low)
//   byte 2 (bits 16..23) L code = 128 + drive_L   (drive -127..127; code 128 = stop; code 0 is invalid)
//   byte 3 (bits 24..31) R code = 128 + drive_R
//
// "drive" is the signed scale of HARDWARE.md 4.4: analogWrite(pin, 128 + drive), i.e. DRV2603
// VOUT = drive / 128 * VFS. The NEC address/command inverse checks are NOT used (IRremote then reports the
// frame as ONKYO or extended NEC); the CRC-5 replaces them. IRremote's pulse-distance decoder does not reject
// marginal timings, so a corrupted frame shows up as flipped bits: always check the CRC.
//
// CRC-5: poly x^5 + x^2 + 1 (0x05), init 0x1F, MSB-first register, no reflection, no final XOR, run over all
// 32 frame bits in ON-AIR order (raw bit 0 first). The sender picks the check bits (raw bits 8..12) so that
// the register ends at 0; the receiver accepts a frame only if it does. Read in on-air order the frame is a
// CRC-5 code word, so the guarantees hold for physical errors (adjacent on air = adjacent in the code word):
//   - every single-bit error and every burst of up to 5 consecutive on-air bits is detected;
//   - every 2-bit error except raw bits 0 + 31 together (first and last bit on air) is detected (the code
//     word is 32 bits, one more than the polynomial's period of 31);
//   - otherwise the CRC is blind to 1/16 of 6-bit bursts, 1/32 of longer bursts and ~1/29 of 3- or 4-bit errors.
// Protocol 1 ran the CRC in another bit order than the air, so some 2..5-bit on-air bursts passed unseen.
//
// UNTESTED on hardware: the encoding was checked only against the Python mirror (compile-time vectors).
#pragma once
#include <stdint.h>

#define CALIB_PROTO_VERSION 2

#define CALIB_ADDR_BROADCAST 0xFF

// Operations (3 bits).
#define CALIB_OP_STOP     0  // motors off (drive 0, EN LOW); closes the open segment
#define CALIB_OP_SEG      1  // start a calibration segment at (L, R): sync flash + new log record
#define CALIB_OP_DRIVE    2  // keep-alive or level change inside the open segment (no flash); outside one it stops the motors (drive_no_seg)
#define CALIB_OP_RECENTRE 3  // drive (L, R) for re-centring between segments; closes the open segment
#define CALIB_OP_PING     4  // no change to the motors; sync flash; does not feed the command watchdog; a run of PINGs shares one ping record

#define CALIB_OK        0
#define CALIB_ERR_CODE -1  // a drive code of 0 (outside -127..127)
#define CALIB_ERR_CRC  -2

struct CalibFrame {
  uint8_t id;
  uint8_t op;
  int8_t L;
  int8_t R;
};

constexpr int calib_clamp_drive(int d) { return d > 127 ? 127 : (d < -127 ? -127 : d); }
constexpr uint8_t calib_code(int d) { return (uint8_t)(128 + calib_clamp_drive(d)); }

// One register step: shift in bit b.
constexpr uint8_t calib_crc5_bit(uint8_t crc, uint32_t b) {
  return (((crc >> 4) ^ b) & 1u) ? (uint8_t)(((crc << 1) ^ 0x05u) & 0x1Fu) : (uint8_t)((crc << 1) & 0x1Fu);
}
// Register after raw bits i..31 in on-air order (raw bit 0 first); calib_crc5_air(0x1F, raw, 0) == 0 if valid.
constexpr uint8_t calib_crc5_air(uint8_t crc, uint32_t raw, int i) {
  return i > 31 ? crc : calib_crc5_air(calib_crc5_bit(crc, (raw >> i) & 1u), raw, i + 1);
}
// Register after n zero bits: multiplies the remainder by x^n (mod the polynomial).
constexpr uint8_t calib_crc5_zeros(uint8_t crc, int n) {
  return n <= 0 ? crc : calib_crc5_zeros(calib_crc5_bit(crc, 0u), n - 1);
}
constexpr uint8_t calib_rev5(uint8_t v) {
  return (uint8_t)(((v & 1u) << 4) | ((v & 2u) << 2) | (v & 4u) | ((v & 8u) >> 2) | ((v & 16u) >> 4));
}
// Check bits for raw bits 8..12 of `data` (those bits 0): the one value that zeroes the register over the
// frame. A check value c enters the register as c * x^24 (raw bit 12 is x^24), so c = remainder * x^-24
// = remainder * x^7 (the polynomial's period is 31): 7 zero steps. Raw bit 8 goes out first: c's top bit.
constexpr uint8_t calib_crc_field(uint32_t data) {
  return calib_rev5(calib_crc5_zeros(calib_crc5_air(0x1F, data, 0), 7));
}

constexpr uint32_t calib_data(uint8_t id, uint8_t op, int L, int R) {
  return (uint32_t)id
       | ((uint32_t)(op & 7u) << 13)
       | ((uint32_t)calib_code(L) << 16)
       | ((uint32_t)calib_code(R) << 24);
}

constexpr uint32_t calib_pack(uint8_t id, uint8_t op, int L, int R) {
  return calib_data(id, op, L, R) | ((uint32_t)calib_crc_field(calib_data(id, op, L, R)) << 8);
}

constexpr int calib_check(uint32_t raw) {
  return (((raw >> 16) & 0xFFu) == 0u || ((raw >> 24) & 0xFFu) == 0u) ? CALIB_ERR_CODE
       : (calib_crc5_air(0x1F, raw, 0) == 0u) ? CALIB_OK : CALIB_ERR_CRC;
}

static inline int calib_unpack(uint32_t raw, CalibFrame *f) {
  int r = calib_check(raw);
  if (r != CALIB_OK) return r;
  f->id = (uint8_t)(raw & 0xFFu);
  f->op = (uint8_t)((raw >> 13) & 7u);
  f->L = (int8_t)((int)((raw >> 16) & 0xFFu) - 128);
  f->R = (int8_t)((int)((raw >> 24) & 0xFFu) - 128);
  return CALIB_OK;
}

// Bot ID from the SAMD21 128-bit serial (words at 0x0080A00C, 0x0080A040, 0x0080A044, 0x0080A048).
// FNV-1a 32 over the 16 bytes in the order of the USB iSerial string (each word big-endian), folded to
// 8 bits and mapped to 1..254. 254 values: expect collisions in a fleet (P ~ 16 % for 10 bots), so the
// PC side checks the fleet for duplicates and the sketch has BOT_ID_OVERRIDE.
constexpr uint32_t calib_fnv_byte(uint32_t h, uint32_t b) { return (h ^ (b & 0xFFu)) * 16777619u; }
constexpr uint32_t calib_fnv_word(uint32_t h, uint32_t w) {
  return calib_fnv_byte(calib_fnv_byte(calib_fnv_byte(calib_fnv_byte(h, w >> 24), w >> 16), w >> 8), w);
}
constexpr uint32_t calib_serial_hash(uint32_t w0, uint32_t w1, uint32_t w2, uint32_t w3) {
  return calib_fnv_word(calib_fnv_word(calib_fnv_word(calib_fnv_word(2166136261u, w0), w1), w2), w3);
}
constexpr uint8_t calib_fold8(uint32_t h) { return (uint8_t)((h ^ (h >> 8) ^ (h >> 16) ^ (h >> 24)) & 0xFFu); }
constexpr uint8_t calib_bot_id(uint32_t w0, uint32_t w1, uint32_t w2, uint32_t w3) {
  return (uint8_t)(1u + calib_fold8(calib_serial_hash(w0, w1, w2, w3)) % 254u);
}
