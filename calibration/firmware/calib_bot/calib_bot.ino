// calib_bot.ino -- Phase 0 "learning to walk" calibration firmware for the GRASPion body (SAMD21E18A).
//
// UNTESTED: compiled with arduino-cli only; never run on a bot. Bench-test every safety path (watchdog,
// low-battery abort, address filtering) before the first calibration run (calibration/README.md, step B).
//
// Build: FQBN graspion:samd:graspionBody:opt=fastest,usbstack=tinyusb (NOT the QT Py M0 board: it drives
// D12 = motor enable HIGH at boot). Libraries: IRremote@4.7.1 and Adafruit NeoPixel only. The MLX90393 is
// driven directly over Wire: the Adafruit MLX90393 library pulls in Adafruit BusIO, whose SPI code does not
// assemble at opt=fastest on this core (delayMicroseconds' "+r" inline asm gets a high register on the M0+).
// See calibration/README.md for the full arduino-cli command.
//
// What it does (FEASIBILITY.md 5.2, step 2):
//  * Receives addressed 32-bit NEC-timed frames from the overhead IR blaster on the TSOP (D7). The frame
//    format is in calib_protocol.h: bot ID (from the chip serial), op, CRC-5, L and R drive codes.
//    Frames for other IDs, bad CRCs and NEC repeat codes are ignored.
//  * SEG starts a calibration segment at (L, R): one ~15 % blue flash on the top NeoPixel (camera time sync and
//    link-latency measurement) and a new log record. DRIVE keeps the segment alive or changes its level.
//    RECENTRE drives between segments (not logged). STOP stops. PING flashes without moving.
//  * Command watchdog: motors stop (drive 0 = analogWrite 128, EN LOW) WATCHDOG_MS after the last valid frame.
//    Optional SAMD21 hardware watchdog (USE_HW_WDT) resets the MCU if loop() hangs (e.g. a stuck I2C bus).
//  * Logs mVbat from the AVR (I2C 0x36) at rest and under load; latches a low-battery stop below MVBAT_ABORT.
//    After the latch, every frame for this bot is answered with a red flash and the motors stay off.
//  * Logs the MLX90393 (0x10; gain 1x, OSR 0, filter 2, 18-bit: 1.84 ms non-blocking conversions, about
//    +-20 mT range so the ERM magnets should not clip it) at rest and under load, which gives the magnetometer
//    offset map b(L, R) needed for Phase 2.
//  * D6 (IR TX -> AVR -> IR LED) is driven HIGH and Serial1 is never started, so the bot's IR LED stays off.
//  * USB serial (115200, TinyUSB): 'i' info, 'd' dump log as CSV, 'c' clear log, 's' stats, 'r' clear the
//    low-battery latch. All output is non-blocking; loop() always returns (TinyUSB needs it).

#if defined(USE_TINYUSB)
#include <Adafruit_TinyUSB.h>      // arduino-cli links the TinyUSB stack only when a sketch includes it
#endif
#define DECODE_NEC                 // NEC family only (NEC, NEC2, Onkyo, Apple); the raw 32 bits are used
#define EXCLUDE_EXOTIC_PROTOCOLS
#define EXCLUDE_UNIVERSAL_PROTOCOLS
#define NO_LED_FEEDBACK_CODE       // LED_BUILTIN (D13) is not connected on the body
#define RAW_BUFFER_LENGTH 100      // NEC needs 68 entries
#include <IRremote.hpp>

#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include "calib_protocol.h"
#include "calib_protocol_vectors.h"  // compile-time check against the Python mirror

// ------------------------------------------------------------------------------------------------ settings
// PWM mode: decide once per fleet BEFORE calibrating; a map measured in one mode is invalid in the other.
// 0 = core analogWrite, 732 Hz (all upstream examples; 14x below the DRV2603 10 kHz minimum).
// 1 = 40 kHz on TCC0 (HARDWARE.md 4.4 snippet; in spec, UNTESTED).
// Override without editing: arduino-cli compile --build-property "compiler.cpp.extra_flags=-DPWM_40KHZ=1" ...
#ifndef PWM_40KHZ
#define PWM_40KHZ 0
#endif
#ifndef BOT_ID_OVERRIDE
#define BOT_ID_OVERRIDE 0          // 1..254 to replace the serial-derived ID (use on ID collisions)
#endif
#ifndef USE_HW_WDT
#define USE_HW_WDT 1               // SAMD21 WDT, ~1 s timeout, fed every loop()
#endif

#define FW_VERSION "calib_bot/1"

static const int16_t  DRIVE_MAX = 120;          // |drive| cap; 120 ~ 3.07 V on the motor (2.7 V rated, 3.3 V max)
static const uint16_t WATCHDOG_MS = 300;        // ~2-3 IR frame periods (FEASIBILITY.md 5.2); raise if the
                                                // bench link test shows < 9 frames/s
static const uint16_t MVBAT_ABORT = 3400;       // under load; motor speed saturates below V+ ~3.4 V (INFERRED)
static const uint8_t  MVBAT_ABORT_COUNT = 3;    // consecutive low samples before latching (rejects start dips)
static const uint16_t MVBAT_PLAUSIBLE = 2000;   // readings below this are treated as read errors
static const uint16_t MVUSB_PRESENT = 4500;     // AVR bytes 0-1 >= 4500 mV: USB or charger present (examples)
static const uint16_t PWR_PERIOD_LOAD_MS = 200;
static const uint16_t PWR_PERIOD_REST_MS = 1000;
static const uint16_t SETTLE_MS = 300;          // skip battery / magnetometer samples after a drive change
static const uint16_t FLASH_MS = 70;            // >= 2 frames at 30 fps
static const uint8_t  FLASH_LEVEL = 38;         // ~15 % of one channel (400 mA overcurrent trip, HARDWARE.md 3.2)
static const uint16_t MAG_PERIOD_MS = 20;       // 50 Hz
static const uint16_t MAG_CONV_MS = 3;          // OSR 0, filter 2: 1.84 ms for XYZ (Adafruit tconv table)
static const uint16_t MAG_RETRY_MS = 2;
static const uint16_t MAG_GIVEUP_MS = 50;
static const uint16_t MAX_RECS = 400;           // 36 B each; dump + clear between batteries
static const int16_t  MAG_NONE = 0x7FFF;

// ------------------------------------------------------------------------------------------------ state
enum EndReason : uint8_t { END_OPEN = 0, END_NEXT_SEG, END_RECENTRE, END_STOP, END_WATCHDOG, END_LOWBAT, END_PING };
enum RecKind : uint8_t { REC_SEG = 1, REC_PING = 2 };
enum RecFlags : uint8_t {
  F_MOTORS_WERE_ON = 1,  // motors were running when the record opened (rest values are not at rest)
  F_USB_PRESENT = 2,     // USB / charger present: battery readings are charging readings
  F_MAG_SAT = 4,         // a magnetometer sample hit the int16 clamp
  F_MAG_STALE = 8,       // no magnetometer sample within 100 ms before the start
};

struct LogRec {          // 36 bytes
  uint32_t t_start_ms;   // bot millis() when the frame was decoded (crystalless clock, +-2 % on battery)
  uint32_t t_end_ms;
  uint8_t  kind;
  int8_t   L, R;         // applied drive (after the DRIVE_MAX clamp); for PING the drive at that time
  uint8_t  end_reason;
  uint16_t n_frames;     // valid frames for this bot while the record was open (incl. the SEG frame)
  uint8_t  n_changes;    // level changes via DRIVE
  uint8_t  flags;
  uint16_t mv_rest;      // mVbat read just before the drive was applied (0 = read failed)
  uint16_t mv_load_min;  // under load, after SETTLE_MS (0 = no sample)
  uint16_t mv_load_mean;
  uint16_t n_mag;        // magnetometer samples averaged into mag_load
  int16_t  mag_rest[3];  // uT, last sample before the start (MAG_NONE = none)
  int16_t  mag_load[3];  // uT, mean under load after SETTLE_MS (MAG_NONE = none)
};

struct Stats {
  uint32_t rx_ok, crc_err, other_addr, ir_other, bad_op, drive_no_seg, bcast_stop;
  uint32_t wd_trips, lowbat_trips, pwr_err, mag_err, rec_overflow;
};

static Adafruit_NeoPixel px(NUM_NEOPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
static Stats st;
static LogRec recs[MAX_RECS];
static uint16_t nRecs = 0;
static bool recOpen = false;     // recs[nRecs - 1] is open
static bool segActive = false;   // a SEG started and has not ended (even if the log overflowed)

static uint8_t botId = 0;
static uint32_t serialWords[4];
static uint32_t resetCause = 0;

static int8_t curL = 0, curR = 0;
static bool motorsOn = false;
static uint32_t lastFrameMs = 0, driveChangeMs = 0;

static uint16_t mvSys = 0, mvBat = 0;
static bool usbPresent = false, lowBat = false;
static uint8_t lowCount = 0;
static uint32_t pwrLastMs = 0;
static uint32_t accMvSum = 0;
static uint16_t accMvN = 0, accMvMin = 0xFFFF;

static bool magOk = false, magPending = false;
static uint32_t magStartMs = 0, magTryMs = 0, magLastMs = 0;
static int16_t magLast[3] = {MAG_NONE, MAG_NONE, MAG_NONE};
static int32_t accMag[3];
static uint16_t accMagN = 0;
static bool accMagSat = false;

static bool flashOn = false;
static uint32_t flashUntil = 0;

static int32_t dumpIdx = -1;     // -1 idle; 0..nRecs-1 next record to print; nRecs = footer

// ------------------------------------------------------------------------------------------------ motors
#if PWM_40KHZ
static const uint16_t TCC_PER = 1199;           // 48 MHz / 1200 = 40 kHz
static void motorsPwmInit() {                   // HARDWARE.md 4.4; call with EN LOW
  analogWrite(BUZ_RIGHT, 128); analogWrite(BUZ_LEFT, 128);   // core sets the pin mux, GCLK0 -> TCC0, NPWM
  TCC0->CTRLA.bit.ENABLE = 0; while (TCC0->SYNCBUSY.bit.ENABLE);
  TCC0->COUNT.reg = 0;        while (TCC0->SYNCBUSY.bit.COUNT);
  TCC0->PER.reg = TCC_PER;    while (TCC0->SYNCBUSY.bit.PER);
  TCC0->CC[0].reg  = TCC0->CC[1].reg  = (TCC_PER + 1) / 2;
  TCC0->CCB[0].reg = TCC0->CCB[1].reg = (TCC_PER + 1) / 2;   // overwrite values analogWrite left pending
  while (TCC0->SYNCBUSY.reg);
  TCC0->CTRLA.bit.ENABLE = 1; while (TCC0->SYNCBUSY.bit.ENABLE);
}
static void motorSet(uint8_t ch /*0 = RIGHT D2, 1 = LEFT D3*/, int16_t drive) {
  // duty = (128 + drive) / 256, the same VOUT = drive / 128 * VFS as the analogWrite mode
  TCC0->CCB[ch].reg = (uint32_t)(((int32_t)(128 + drive) * (TCC_PER + 1) + 128) / 256);
  while (TCC0->SYNCBUSY.reg & (TCC_SYNCBUSY_CCB0 << ch));
}
#endif

static void setDrive(int16_t L, int16_t R, uint32_t now) {
  L = constrain(L, -DRIVE_MAX, DRIVE_MAX);
  R = constrain(R, -DRIVE_MAX, DRIVE_MAX);
  const bool on = (L != 0 || R != 0);
  if (!on) digitalWrite(BUZ_ENABLE, LOW);        // shared enable: LOW whenever both motors stop
#if PWM_40KHZ
  motorSet(1, L);
  motorSet(0, R);
#else
  analogWrite(BUZ_LEFT, 128 + L);                // 128 is the true zero (the examples' 127 is -0.8 %)
  analogWrite(BUZ_RIGHT, 128 + R);
#endif
  if (on) digitalWrite(BUZ_ENABLE, HIGH);
  if (L != curL || R != curR || on != motorsOn) driveChangeMs = now;
  curL = (int8_t)L; curR = (int8_t)R; motorsOn = on;
}

// ------------------------------------------------------------------------------------------------ hw watchdog
#if USE_HW_WDT
static void hwWdtBegin() {                       // same clocking as Adafruit SleepyDog: GCLK2 = OSCULP32K / 32
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(2) | GCLK_GENDIV_DIV(4);
  while (GCLK->STATUS.bit.SYNCBUSY);
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(2) | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_OSCULP32K | GCLK_GENCTRL_DIVSEL;
  while (GCLK->STATUS.bit.SYNCBUSY);
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_WDT | GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK2;
  while (GCLK->STATUS.bit.SYNCBUSY);
  WDT->CTRL.reg = 0;                    while (WDT->STATUS.bit.SYNCBUSY);
  WDT->CONFIG.reg = WDT_CONFIG_PER_1K;  while (WDT->STATUS.bit.SYNCBUSY);   // 1024 cycles at 1.024 kHz ~ 1 s
  WDT->CTRL.reg = WDT_CTRL_ENABLE;      while (WDT->STATUS.bit.SYNCBUSY);
}
static void hwWdtFeed() {
  if (!WDT->STATUS.bit.SYNCBUSY) WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;   // never block on the sync
}
#else
static void hwWdtBegin() {}
static void hwWdtFeed() {}
#endif

// ------------------------------------------------------------------------------------------------ LEDs
static void ledSet(uint8_t r, uint8_t g, uint8_t b) {   // top pixel only; show() blocks IRQs ~60 us
  px.setPixelColor(NEOPIXEL_TOP, px.Color(r, g, b));
  px.setPixelColor(NEOPIXEL_BOTTOM, 0);
  px.show();
}
static void flash(bool red, uint32_t now) {
  if (red) ledSet(FLASH_LEVEL, 0, 0); else ledSet(0, 0, FLASH_LEVEL);
  flashOn = true;
  flashUntil = now + FLASH_MS;
}
static void updateFlash(uint32_t now) {
  if (flashOn && (int32_t)(now - flashUntil) >= 0) { ledSet(0, 0, 0); flashOn = false; }
}

// ------------------------------------------------------------------------------------------------ power
// AVR "BIOS" at 0x36: plain 5-byte read, no register pointer. Bytes 0-1 = SYS / USB mV, 2-3 = mVbat,
// big-endian (INFERRED, HARDWARE.md 3.3: verify against a multimeter before trusting MVBAT_ABORT).
static bool readPower(uint16_t &s, uint16_t &b) {
  uint8_t buf[5];
  if (Wire.requestFrom((uint8_t)0x36, (uint8_t)5) != 5) return false;
  for (uint8_t i = 0; i < 5; i++) buf[i] = (uint8_t)Wire.read();
  s = (uint16_t)((buf[0] << 8) | buf[1]);
  b = (uint16_t)((buf[2] << 8) | buf[3]);
  return true;
}
static uint16_t readBatNow() {
  uint16_t s, b;
  if (!readPower(s, b)) { st.pwr_err++; return 0; }
  mvSys = s; mvBat = b; usbPresent = (s >= MVUSB_PRESENT);
  return b;
}

// ------------------------------------------------------------------------------------------------ log
static void closeRec(uint8_t reason, uint32_t now) {
  segActive = false;
  if (!recOpen) return;
  LogRec &r = recs[nRecs - 1];
  r.t_end_ms = now;
  r.end_reason = reason;
  r.mv_load_min = accMvN ? accMvMin : 0;
  r.mv_load_mean = accMvN ? (uint16_t)(accMvSum / accMvN) : 0;
  r.n_mag = accMagN;
  for (uint8_t k = 0; k < 3; k++) r.mag_load[k] = accMagN ? (int16_t)(accMag[k] / (int32_t)accMagN) : MAG_NONE;
  if (accMagSat) r.flags |= F_MAG_SAT;
  recOpen = false;
}

static LogRec *newRec(uint8_t kind, uint32_t now) {
  if (nRecs >= MAX_RECS) { st.rec_overflow++; return nullptr; }
  LogRec &r = recs[nRecs++];
  memset(&r, 0, sizeof(r));
  r.kind = kind;
  r.t_start_ms = now;
  r.t_end_ms = now;
  r.L = curL; r.R = curR;
  if (motorsOn) r.flags |= F_MOTORS_WERE_ON;
  r.mv_rest = readBatNow();
  if (usbPresent) r.flags |= F_USB_PRESENT;
  const bool fresh = magOk && magLast[0] != MAG_NONE && (now - magLastMs) < 100;
  if (!fresh) r.flags |= F_MAG_STALE;
  for (uint8_t k = 0; k < 3; k++) { r.mag_rest[k] = fresh ? magLast[k] : MAG_NONE; r.mag_load[k] = MAG_NONE; }
  return &r;
}

static void openSeg(int8_t L, int8_t R, uint32_t now) {
  LogRec *r = newRec(REC_SEG, now);
  setDrive(L, R, now);
  segActive = true;
  accMvSum = 0; accMvN = 0; accMvMin = 0xFFFF;
  accMag[0] = accMag[1] = accMag[2] = 0; accMagN = 0; accMagSat = false;
  if (r) { r->L = curL; r->R = curR; r->n_frames = 1; recOpen = true; }
}

static void logPing(uint32_t now) {
  if (recOpen) recs[nRecs - 1].n_frames++;      // a PING inside a segment still counts as a frame
  LogRec *r = newRec(REC_PING, now);
  if (!r) return;
  r->end_reason = END_PING;
  r->n_frames = 1;
  if (recOpen) {                                 // keep the open segment as the last record
    LogRec tmp = recs[nRecs - 1];
    recs[nRecs - 1] = recs[nRecs - 2];
    recs[nRecs - 2] = tmp;
  }
}

// ------------------------------------------------------------------------------------------------ IR frames
static void lowBatteryStop(uint32_t now) {
  closeRec(END_LOWBAT, now);
  setDrive(0, 0, now);
  if (!lowBat) st.lowbat_trips++;
  lowBat = true;
  flash(true, now);
}

static void handleFrame(uint32_t raw, uint32_t now) {
  CalibFrame f;
  if (calib_unpack(raw, &f) != CALIB_OK) { st.crc_err++; return; }
  if (f.id == CALIB_ADDR_BROADCAST) {
    if (f.op == CALIB_OP_STOP) { st.bcast_stop++; closeRec(END_STOP, now); setDrive(0, 0, now); }
    return;
  }
  if (f.id != botId) { st.other_addr++; return; }
  st.rx_ok++;
  lastFrameMs = now;
  if (lowBat) {                                  // never drive again until 'r' over USB or a reset
    setDrive(0, 0, now);
    if (f.op != CALIB_OP_STOP && f.op != CALIB_OP_DRIVE) flash(true, now);
    return;
  }
  switch (f.op) {
    case CALIB_OP_STOP:
      closeRec(END_STOP, now);
      setDrive(0, 0, now);
      break;
    case CALIB_OP_SEG:
      closeRec(END_NEXT_SEG, now);
      openSeg(f.L, f.R, now);
      flash(false, now);
      break;
    case CALIB_OP_DRIVE:
      if (!segActive) { st.drive_no_seg++; break; }   // SEG missed: stay still, the PC sees no flash
      if (recOpen) {
        LogRec &r = recs[nRecs - 1];
        r.n_frames++;
        if ((f.L != curL || f.R != curR) && r.n_changes < 255) r.n_changes++;
      }
      setDrive(f.L, f.R, now);
      break;
    case CALIB_OP_RECENTRE:
      closeRec(END_RECENTRE, now);
      setDrive(f.L, f.R, now);
      break;
    case CALIB_OP_PING:
      logPing(now);
      flash(false, now);
      break;
    default:
      st.bad_op++;
  }
}

static void pollIR(uint32_t now) {
  if (!IrReceiver.decode()) return;
  const IRData &d = IrReceiver.decodedIRData;
  const bool full = d.numberOfBits == 32 && !(d.flags & IRDATA_FLAGS_WAS_OVERFLOW) &&
                    (d.protocol == NEC || d.protocol == NEC2 || d.protocol == ONKYO || d.protocol == APPLE);
  const uint32_t raw = (uint32_t)d.decodedRawData;
  IrReceiver.resume();
  if (full) handleFrame(raw, now); else st.ir_other++;   // NEC repeat codes carry no address: ignored
}

static void commandWatchdog(uint32_t now) {
  if (motorsOn && (now - lastFrameMs) > WATCHDOG_MS) {
    closeRec(END_WATCHDOG, now);
    setDrive(0, 0, now);
    st.wd_trips++;
  }
}

// ------------------------------------------------------------------------------------------------ sensors
static void pollPower(uint32_t now) {
  if (now - pwrLastMs < (motorsOn ? PWR_PERIOD_LOAD_MS : PWR_PERIOD_REST_MS)) return;
  pwrLastMs = now;
  const uint16_t b = readBatNow();
  if (b == 0) return;
  if (b < MVBAT_PLAUSIBLE) { st.pwr_err++; return; }
  const bool settled = (now - driveChangeMs) >= SETTLE_MS;
  if (motorsOn && settled && recOpen) {
    accMvSum += b; accMvN++;
    if (b < accMvMin) accMvMin = b;
  }
  if (usbPresent) { lowCount = 0; return; }      // on USB the abort is meaningless
  if (motorsOn && !settled) return;              // skip start-current dips; do not reset the count (PRBS)
  if (b < MVBAT_ABORT) {
    if (++lowCount >= MVBAT_ABORT_COUNT && !lowBat) lowBatteryStop(now);
  } else {
    lowCount = 0;
  }
}

// MLX90393 over Wire (datasheet command set; register layout as in the Adafruit library). Every command
// returns a status byte: bit 5 SM_MODE (conversion still running), bit 4 ERROR, bit 2 RS (reset done).
static const uint8_t MLX_ADDR = MLX90393_I2C_ADDRESS;   // 0x10 (variant.h); the Adafruit default 0x0C is wrong
static const uint8_t MLX_SM_XYZ = 0x3E, MLX_RM_XYZ = 0x4E, MLX_EX = 0x80, MLX_RT = 0xF0, MLX_RR = 0x50, MLX_WR = 0x60;
static const uint8_t MLX_ST_SM = 0x20, MLX_ST_ERR = 0x10, MLX_ST_RS = 0x04, MLX_ST_FAIL = 0xFF;
static const uint16_t MLX_CONF1 = 0x0C | (7u << 4);   // HALLCONF 0xC, GAIN_SEL 7 ("1x" in the Adafruit naming)
static const uint16_t MLX_CONF3_MASK = 0x07FF;        // RES_Z, RES_Y, RES_X, DIG_FILT, OSR (OSR2 kept)
static const uint16_t MLX_CONF3 = (2u << 9) | (2u << 7) | (2u << 5) | (2u << 2) | 0u;   // 18 bit, filter 2, OSR 0
static const float MLX_LSB_XY = 0.601f, MLX_LSB_Z = 0.968f;   // uT/LSB for HALLCONF 0xC, GAIN_SEL 7, RES 18

static uint8_t mlxCmd(const uint8_t *tx, uint8_t ntx, uint8_t *rx, uint8_t nrx) {
  Wire.beginTransmission(MLX_ADDR);
  Wire.write(tx, ntx);
  if (Wire.endTransmission() != 0) return MLX_ST_FAIL;
  if (Wire.requestFrom(MLX_ADDR, (uint8_t)(nrx + 1)) != nrx + 1) return MLX_ST_FAIL;
  const uint8_t status = (uint8_t)Wire.read();
  for (uint8_t i = 0; i < nrx; i++) rx[i] = (uint8_t)Wire.read();
  return status;
}
static bool mlxRead(uint8_t reg, uint16_t &v) {
  const uint8_t tx[2] = {MLX_RR, (uint8_t)(reg << 2)};
  uint8_t rx[2];
  const uint8_t s = mlxCmd(tx, 2, rx, 2);
  if (s == MLX_ST_FAIL || (s & MLX_ST_ERR)) return false;
  v = (uint16_t)((rx[0] << 8) | rx[1]);
  return true;
}
static bool mlxWrite(uint8_t reg, uint16_t v) {
  const uint8_t tx[4] = {MLX_WR, (uint8_t)(v >> 8), (uint8_t)(v & 0xFF), (uint8_t)(reg << 2)};
  const uint8_t s = mlxCmd(tx, 4, nullptr, 0);
  return s != MLX_ST_FAIL && !(s & MLX_ST_ERR);
}
static bool mlxBegin() {                         // blocking (~10 ms); only called from setup()
  uint8_t c = MLX_EX;
  mlxCmd(&c, 1, nullptr, 0);
  c = MLX_RT;
  Wire.beginTransmission(MLX_ADDR); Wire.write(c);
  if (Wire.endTransmission() != 0) return false;
  delay(5);
  if (Wire.requestFrom(MLX_ADDR, (uint8_t)1) != 1 || !(Wire.read() & MLX_ST_RS)) return false;
  uint16_t r0, r2;
  if (!mlxRead(0, r0) || !mlxWrite(0, (uint16_t)((r0 & ~0x007Fu) | MLX_CONF1))) return false;
  if (!mlxRead(2, r2) || !mlxWrite(2, (uint16_t)((r2 & ~MLX_CONF3_MASK) | MLX_CONF3))) return false;
  return mlxRead(0, r0) && (r0 & 0x7F) == MLX_CONF1 && mlxRead(2, r2) && (r2 & MLX_CONF3_MASK) == MLX_CONF3;
}

static int16_t magUT(uint8_t hi, uint8_t lo, float lsb) {   // RES 18: unsigned with a 0x8000 offset
  const float v = (float)((int32_t)((hi << 8) | lo) - 32768) * lsb;
  if (v > 32766.f) { accMagSat = true; return 32766; }
  if (v < -32767.f) { accMagSat = true; return -32767; }
  return (int16_t)lroundf(v);
}

static void pollMag(uint32_t now) {
  if (!magOk) return;
  if (!magPending) {
    if (now - magStartMs < MAG_PERIOD_MS) return;
    magStartMs = now;
    const uint8_t s = mlxCmd(&MLX_SM_XYZ, 1, nullptr, 0);
    if (s != MLX_ST_FAIL && !(s & MLX_ST_ERR)) { magPending = true; magTryMs = now; } else st.mag_err++;
    return;
  }
  if (now - magStartMs < MAG_CONV_MS || now - magTryMs < MAG_RETRY_MS) return;
  magTryMs = now;
  uint8_t rx[6];
  const uint8_t s = mlxCmd(&MLX_RM_XYZ, 1, rx, 6);
  if (s != MLX_ST_FAIL && !(s & (MLX_ST_ERR | MLX_ST_SM))) {   // SM_MODE set: conversion still running
    magPending = false;
    magLast[0] = magUT(rx[0], rx[1], MLX_LSB_XY);
    magLast[1] = magUT(rx[2], rx[3], MLX_LSB_XY);
    magLast[2] = magUT(rx[4], rx[5], MLX_LSB_Z);
    magLastMs = now;
    if (recOpen && motorsOn && (now - driveChangeMs) >= SETTLE_MS) {
      for (uint8_t k = 0; k < 3; k++) accMag[k] += magLast[k];
      accMagN++;
    }
  } else if (now - magStartMs > MAG_GIVEUP_MS) {
    magPending = false;
    st.mag_err++;
  }
}

// ------------------------------------------------------------------------------------------------ USB serial
static void printInfo() {
  char ser[33];
  snprintf(ser, sizeof(ser), "%08lX%08lX%08lX%08lX", (unsigned long)serialWords[0], (unsigned long)serialWords[1],
           (unsigned long)serialWords[2], (unsigned long)serialWords[3]);
  Serial.print(F("I fw=" FW_VERSION " proto=")); Serial.print(CALIB_PROTO_VERSION);
  Serial.print(F(" id=")); Serial.print(botId);
  Serial.print(F(" id_override=")); Serial.print(BOT_ID_OVERRIDE);
  Serial.print(F(" serial=")); Serial.print(ser);
  Serial.print(F(" pwm_hz=")); Serial.print(PWM_40KHZ ? 40000 : 732);
  Serial.print(F(" drive_max=")); Serial.print(DRIVE_MAX);
  Serial.print(F(" watchdog_ms=")); Serial.print(WATCHDOG_MS);
  Serial.print(F(" mvbat_abort=")); Serial.print(MVBAT_ABORT);
  Serial.print(F(" hw_wdt=")); Serial.print(USE_HW_WDT);
  Serial.print(F(" rcause=0x")); Serial.print(resetCause, HEX);
  Serial.print(F(" mag=")); Serial.print(magOk ? 1 : 0);
  Serial.print(F(" mvsys=")); Serial.print(mvSys);
  Serial.print(F(" mvbat=")); Serial.print(mvBat);
  Serial.print(F(" lowbat=")); Serial.print(lowBat ? 1 : 0);
  Serial.print(F(" nrec=")); Serial.print(nRecs);
  Serial.print(F(" millis=")); Serial.println(millis());
}

static void printStats() {
  Serial.print(F("S rx_ok=")); Serial.print(st.rx_ok);
  Serial.print(F(" crc_err=")); Serial.print(st.crc_err);
  Serial.print(F(" other_addr=")); Serial.print(st.other_addr);
  Serial.print(F(" ir_other=")); Serial.print(st.ir_other);
  Serial.print(F(" bad_op=")); Serial.print(st.bad_op);
  Serial.print(F(" drive_no_seg=")); Serial.print(st.drive_no_seg);
  Serial.print(F(" bcast_stop=")); Serial.print(st.bcast_stop);
  Serial.print(F(" wd_trips=")); Serial.print(st.wd_trips);
  Serial.print(F(" lowbat_trips=")); Serial.print(st.lowbat_trips);
  Serial.print(F(" pwr_err=")); Serial.print(st.pwr_err);
  Serial.print(F(" mag_err=")); Serial.print(st.mag_err);
  Serial.print(F(" rec_overflow=")); Serial.println(st.rec_overflow);
}

static void printRec(const LogRec &r) {
  char line[160];
  snprintf(line, sizeof(line), "%u,%lu,%lu,%d,%d,%u,%u,%u,%u,%u,%u,%u,%u,%d,%d,%d,%d,%d,%d",
           r.kind, (unsigned long)r.t_start_ms, (unsigned long)r.t_end_ms, r.L, r.R, r.end_reason, r.n_frames,
           r.n_changes, r.flags, r.mv_rest, r.mv_load_min, r.mv_load_mean, r.n_mag, r.mag_rest[0], r.mag_rest[1],
           r.mag_rest[2], r.mag_load[0], r.mag_load[1], r.mag_load[2]);
  Serial.println(line);
}

static void pollDump() {                         // non-blocking: only writes when the CDC FIFO has room
  if (dumpIdx < 0) return;
  if (!Serial) { dumpIdx = -1; return; }         // host closed the port: abandon (the log is kept)
  while (dumpIdx <= (int32_t)nRecs && Serial.availableForWrite() >= 160) {
    if (dumpIdx == (int32_t)nRecs) {
      Serial.print(F("# end n=")); Serial.println(nRecs);
      dumpIdx = -1;
      return;
    }
    printRec(recs[dumpIdx++]);
  }
}

static void pollUsb() {
  while (Serial.available() > 0 && dumpIdx < 0) {
    switch (Serial.read()) {
      case 'i': printInfo(); break;
      case 's': printStats(); break;
      case 'd':
        Serial.print(F("# ")); printInfo();
        Serial.print(F("# ")); printStats();
        Serial.println(F("kind,t_start_ms,t_end_ms,L,R,end_reason,n_frames,n_changes,flags,mv_rest,mv_load_min,"
                         "mv_load_mean,n_mag,mag_rest_x,mag_rest_y,mag_rest_z,mag_load_x,mag_load_y,mag_load_z"));
        dumpIdx = 0;
        break;
      case 'c': nRecs = 0; recOpen = false; memset(&st, 0, sizeof(st)); Serial.println(F("K cleared")); break;
      case 'r': lowBat = false; lowCount = 0; Serial.println(F("K lowbat cleared")); break;
      default: break;
    }
  }
}

// ------------------------------------------------------------------------------------------------ setup / loop
void setup() {
  pinMode(BUZ_ENABLE, OUTPUT); digitalWrite(BUZ_ENABLE, LOW);   // FIRST: D12 boots as a floating input
  pinMode(IR_TX, OUTPUT); digitalWrite(IR_TX, HIGH);            // IR LED off; Serial1 is never started
  analogWriteResolution(8);
#if PWM_40KHZ
  motorsPwmInit();
#endif
  setDrive(0, 0, 0);
  resetCause = PM->RCAUSE.reg;

  serialWords[0] = *(volatile uint32_t *)0x0080A00C;
  serialWords[1] = *(volatile uint32_t *)0x0080A040;
  serialWords[2] = *(volatile uint32_t *)0x0080A044;
  serialWords[3] = *(volatile uint32_t *)0x0080A048;
  botId = BOT_ID_OVERRIDE ? (uint8_t)BOT_ID_OVERRIDE
                          : calib_bot_id(serialWords[0], serialWords[1], serialWords[2], serialWords[3]);

  Serial.begin(115200);                          // never while(!Serial): it blocks forever on battery
  Wire.begin();                                  // 100 kHz: the AVR's I2C speed limit is UNKNOWN
  px.begin();
  ledSet(0, FLASH_LEVEL, 0);                     // short green boot blink (cleared by updateFlash)
  flashOn = true; flashUntil = millis() + 200;

  magOk = mlxBegin();
  readBatNow();

  IrReceiver.begin(IR_RX, DISABLE_LED_FEEDBACK); // D7 becomes GPIO: Serial1 RX is dead from here on
  hwWdtBegin();
}

void loop() {
  const uint32_t now = millis();
  pollIR(now);
  commandWatchdog(now);
  updateFlash(now);
  pollPower(now);
  pollMag(now);
  pollUsb();
  pollDump();
  hwWdtFeed();
}                                                // returning lets TinyUSB run (yield)
