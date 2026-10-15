// calib_protocol.h -- IR frame format shared by calib_bot (receiver) and ir_blaster (sender).
// Mirrored in pc/graspion_calib/protocol.py. Keep the copies in firmware/calib_bot/ and blaster/ir_blaster/
// byte-identical (pc/tests/test_protocol.py checks this). C++11 constexpr so calib_protocol_vectors.h can
// static_assert the Python-generated test vectors at compile time on both targets.
//
// One frame = one NEC-timed 32-bit frame (IRremote sendNECRaw / decodedRawData, LSB first on air):
//
//   byte 0 (bits  0..7)  bot ID: 1..254 (0xFF = broadcast, STOP only; 0x00 left to consumer remotes)
//   byte 1 (bits  8..15) op (3 bits, high) | CRC-5 (5 bits, low)
//   byte 2 (bits 16..23) L code = 128 + drive_L   (drive -127..127; code 128 = stop; code 0 is invalid)
//   byte 3 (bits 24..31) R code = 128 + drive_R
//
// "drive" is the signed scale of HARDWARE.md 4.4: analogWrite(pin, 128 + drive), i.e. DRV2603
// VOUT = drive / 128 * VFS. The NEC address/command inverse checks are NOT used (IRremote then reports the
// frame as ONKYO or extended NEC); the CRC-5 replaces them. IRremote's pulse-distance decoder does not reject
// marginal timings, so a corrupted frame shows up as flipped bits: always check the CRC.
//
// CRC-5: poly x^5 + x^2 + 1 (0x05), init 0x1F, no reflection, no final XOR, MSB first over the 27 data bits
// (id << 19) | (op << 16) | (Lcode << 8) | Rcode. It detects every single-bit error, every burst <= 5 bits and
// all but 1 of the 496 possible 2-bit errors (the code word is 32 bits, one more than the polynomial's period).
//
// UNTESTED on hardware: the encoding was checked only against the Python mirror (compile-time vectors).
#pragma once
#include <stdint.h>

#define CALIB_PROTO_VERSION 1

#define CALIB_ADDR_BROADCAST 0xFF

// Operations (3 bits).
#define CALIB_OP_STOP     0  // motors off (drive 0, EN LOW); closes the open segment
#define CALIB_OP_SEG      1  // start a calibration segment at (L, R): sync flash + new log record
#define CALIB_OP_DRIVE    2  // keep-alive or level change inside the open segment (no flash); ignored otherwise
#define CALIB_OP_RECENTRE 3  // drive (L, R) for re-centring between segments; closes the open segment
#define CALIB_OP_PING     4  // no change to the motors; sync flash + ping record (link and latency tests)

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

constexpr uint32_t calib_crc_msg(uint8_t id, uint8_t op, uint8_t lcode, uint8_t rcode) {
  return ((uint32_t)id << 19) | ((uint32_t)(op & 7u) << 16) | ((uint32_t)lcode << 8) | (uint32_t)rcode;
}

constexpr uint8_t calib_crc5_step(uint8_t crc, uint32_t msg, int bit) {
  return bit < 0 ? crc
                 : calib_crc5_step((((crc >> 4) ^ (uint8_t)((msg >> bit) & 1u)) & 1u)
                                       ? (uint8_t)(((crc << 1) ^ 0x05u) & 0x1Fu)
                                       : (uint8_t)((crc << 1) & 0x1Fu),
                                   msg, bit - 1);
}
constexpr uint8_t calib_crc5(uint32_t msg) { return calib_crc5_step(0x1F, msg, 26); }

constexpr uint32_t calib_pack(uint8_t id, uint8_t op, int L, int R) {
  return (uint32_t)id
       | ((uint32_t)(((op & 7u) << 5) | calib_crc5(calib_crc_msg(id, op, calib_code(L), calib_code(R)))) << 8)
       | ((uint32_t)calib_code(L) << 16)
       | ((uint32_t)calib_code(R) << 24);
}

constexpr int calib_check(uint32_t raw) {
  return (((raw >> 16) & 0xFFu) == 0u || ((raw >> 24) & 0xFFu) == 0u) ? CALIB_ERR_CODE
       : (calib_crc5(calib_crc_msg((uint8_t)(raw & 0xFFu), (uint8_t)((raw >> 13) & 7u),
                                   (uint8_t)((raw >> 16) & 0xFFu), (uint8_t)((raw >> 24) & 0xFFu)))
          == (uint8_t)((raw >> 8) & 0x1Fu)) ? CALIB_OK : CALIB_ERR_CRC;
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
