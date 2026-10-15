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
//    link-latency measurement) and a new log record. DRIVE keeps the segment alive or changes its level; a DRIVE
//    with no open segment stops the motors (counted as drive_no_seg). RECENTRE drives between segments (not
//    logged). STOP stops. PING flashes without moving (a run of PINGs shares one log record).
//  * Command watchdog: motors stop (drive 0 = analogWrite 128, EN LOW) WATCHDOG_MS after the last frame that
//    commands the drive (STOP, SEG, DRIVE inside a segment, RECENTRE; PING and ignored frames do not count).
//    A SysTick deadman pulls EN LOW DEADMAN_MARGIN_MS later even when loop() is stuck. The optional SAMD21
//    hardware watchdog (USE_HW_WDT) cuts EN and resets the MCU at its early warning (~0.5 s without a loop()
//    pass); its time-out at ~1 s is the fallback when the early warning cannot run.
//  * Logs mVbat from the AVR (I2C 0x36) at rest and under load; latches a low-battery stop below MVBAT_ABORT.
//    After the latch, every frame for this bot is answered with a red flash and the motors stay off.
//  * Logs the MLX90393 (0x10; gain 1x, OSR 0, filter 2, 18-bit: ~2.0 ms (2.3 ms max) non-blocking conversions;
//    the RES 2 range is +-22000 LSB, about +-13 mT on X/Y and +-21 mT on Z, and samples near it set F_MAG_SAT)
//    at rest and under load, which gives the magnetometer offset map b(L, R) needed for Phase 2.
//  * D6 (IR TX -> AVR -> IR LED) is driven HIGH and Serial1 is never started, so the bot's IR LED stays off.
//  * USB serial (115200, TinyUSB): 'i' info, 'd' dump log as CSV, 'c' clear log, 's' stats, 'r' clear the
//    low-battery latch. Output is written only as far as the CDC FIFO has room, so a host that stops reading
//    cannot stall loop(). I2C transfers end after the SERCOM's 25-35 ms SCL low time-out instead of hanging.
//  * An I2C line held LOW at boot leaves I2C off (no battery reads, so no low-battery abort): loop() retries the
//    bus recovery about once per second, and until it succeeds every SEG is answered with a red flash and no
//    segment ('i' shows i2c=0).

#if defined(USE_TINYUSB)
#include <Adafruit_TinyUSB.h>      // the TinyUSB stack is linked only when the sketch or a library header it includes pulls it in
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
#define USE_HW_WDT 1               // SAMD21 WDT, fed every loop(): EN cut and reset at ~0.5 s (early warning;
                                   // the ~1 s time-out is the fallback)
#endif
#ifndef BENCH_HANG
#define BENCH_HANG 0               // 1: USB 'H' drives both motors at 80 and hangs loop(), to scope EN through the
#endif                             // deadman, the WDT early warning, the reset and the bootloader (bot on a stand)

#define FW_VERSION "calib_bot/2"

static const int16_t  DRIVE_MAX = 120;          // |drive| cap; 120 ~ 3.07 V on the motor (2.7 V rated, 3.3 V max)
static const uint16_t WATCHDOG_MS = 300;        // ~2-3 IR frame periods (FEASIBILITY.md 5.2); raise if the
                                                // bench link test shows < 9 frames/s
static const uint16_t DEADMAN_MARGIN_MS = 50;   // SysTick deadman: EN LOW at WATCHDOG_MS + this if loop() is stuck
static const uint16_t MVBAT_ABORT = 3400;       // under load; motor speed saturates below V+ ~3.4 V (INFERRED)
static const uint8_t  MVBAT_ABORT_COUNT = 3;    // consecutive low samples before latching (rejects start dips)
static const uint16_t MVBAT_PLAUSIBLE = 2000;   // battery readings outside [this, MVBAT_MAX] are read errors
static const uint16_t MVBAT_MAX = 4500;         // a 1-cell LiPo never reads this high, even while charging
static const uint16_t MVSYS_MAX = 6000;         // USB is 5 V +-10 %; above this (e.g. 0xFFFF) the read failed
static const uint16_t MVUSB_PRESENT = 4500;     // AVR bytes 0-1 >= 4500 mV: USB or charger present (examples)
static const uint16_t PWR_PERIOD_LOAD_MS = 200;
static const uint16_t PWR_PERIOD_REST_MS = 1000;
static const uint16_t SETTLE_MS = 300;          // skip battery / magnetometer samples after a drive change
static const uint16_t FLASH_MS = 70;            // >= 2 frames at 30 fps
static const uint8_t  FLASH_LEVEL = 38;         // ~15 % of one channel (400 mA overcurrent trip, HARDWARE.md 3.2)
static const uint16_t MAG_PERIOD_MS = 20;       // 50 Hz
static const uint16_t MAG_CONV_MS = 4;          // >= 3 ms after SM: magStartMs is stamped once the SM is out and
                                                // millis() ticks every 1 ms. OSR 0, filter 2, XYZ: TSTBY + TACTIVE
                                                // + 3 TCONVM + TCONV_END = 0.22 + 0.36 + 3 * 0.451 + 0.10 ~ 2.0 ms,
                                                // 2.3 ms max (MLX90393 Table 8), then 132 us TINT_RDY
static const uint16_t MAG_RETRY_MS = 2;
static const uint16_t MAG_GIVEUP_MS = 50;
static const uint16_t I2C_RETRY_MS = 1000;      // bus recovery retry while I2C is down (motors off only)
static const uint16_t MAX_RECS = 400;          // 36 B each; dump + clear between batteries
static const int16_t  MAG_NONE = 0x7FFF;

// ------------------------------------------------------------------------------------------------ state
enum EndReason : uint8_t { END_OPEN = 0, END_NEXT_SEG, END_RECENTRE, END_STOP, END_WATCHDOG, END_LOWBAT, END_PING };
enum RecKind : uint8_t { REC_SEG = 1, REC_PING = 2 };
enum RecFlags : uint8_t {
  F_MOTORS_WERE_ON = 1,  // motors were running when the record opened (rest values are not at rest)
  F_USB_PRESENT = 2,     // USB / charger present: battery readings are charging readings
  F_MAG_SAT = 4,         // mag_rest or a sample averaged into mag_load had an axis within 5 % of the RES 2 range
                         // (+-22000 LSB): it may clip
  F_MAG_STALE = 8,       // no magnetometer sample within 100 ms before the start
};

struct LogRec {          // 36 bytes
  uint32_t t_start_ms;   // bot millis() when the frame was decoded (crystalless clock, +-2 % on battery)
  uint32_t t_end_ms;
  uint8_t  kind;
  int8_t   L, R;         // applied drive (after the DRIVE_MAX clamp); for PING the drive at that time
  uint8_t  end_reason;
  uint16_t n_frames;     // valid frames for this bot while the record was open (incl. the SEG frame); PING
                         // record: PINGs in the run (t_start_ms = first, t_end_ms = last)
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
static volatile bool motorsOn = false;           // volatile: also read by the SysTick deadman
static volatile uint32_t lastFrameMs = 0;        // last frame that commanded the drive (see handleFrame)
static uint32_t driveChangeMs = 0;
static volatile uint32_t deadmanCuts = 0;        // EN cut by the SysTick deadman (loop() was stuck with motors on)

static bool i2cOk = false;                       // false: Wire not started (a line held LOW since boot): SEG refused
static uint32_t i2cRetryMs = 0;                  // last bus recovery try while !i2cOk (pollI2cRetry)
static uint16_t mvSys = 0, mvBat = 0;
static uint16_t rawSys = 0, rawBat = 0;          // last AVR words read without a bus error, before the bounds ('i')
static bool usbPresent = false, lowBat = false;
static uint8_t lowCount = 0;
static uint32_t pwrLastMs = 0;
static uint32_t accMvSum = 0;
static uint16_t accMvN = 0, accMvMin = 0xFFFF;

static bool magOk = false, magPending = false;
static uint32_t magStartMs = 0, magTryMs = 0, magLastMs = 0;
static int16_t magLast[3] = {MAG_NONE, MAG_NONE, MAG_NONE};
static bool magLastSat = false;                  // magLast has an axis near the range limit (F_MAG_SAT)
static int32_t accMag[3];
static uint16_t accMagN = 0;
static bool accMagSat = false;

static bool flashOn = false;
static uint32_t flashUntil = 0;

static int32_t dumpIdx = -1;     // -1 idle; 0..nRecs-1 next record to print; nRecs = footer
static uint8_t dumpHdr = 0;      // dump header lines (info, stats, columns) already queued

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

// ------------------------------------------------------------------------------------------------ deadman
// D12 = PA15 = BUZ_ENABLE is EN of both DRV2603s: EN LOW stops both motors in either PWM mode. These write the
// PORT register directly (one store), so any interrupt handler can use them, HardFault included.
static inline bool enIsHigh() {
  const PinDescription &p = g_APinDescription[BUZ_ENABLE];
  return (PORT->Group[p.ulPort].OUT.reg & (1ul << p.ulPin)) != 0;
}
static inline void enCut() {
  const PinDescription &p = g_APinDescription[BUZ_ENABLE];
  PORT->Group[p.ulPort].OUTCLR.reg = 1ul << p.ulPin;
}

// The core calls sysTickHook() from SysTick_Handler every 1 ms (priority 2), so this command watchdog runs even
// when loop() is stuck (a Wire transfer, a CDC write, a library call). It only pulls EN LOW; commandWatchdog()
// does the bookkeeping (segment close, PWM 128, wd_trips) when loop() runs again. The margin keeps it from
// racing loop()'s own 300 ms check in normal operation. C linkage: the core's weak symbol is a C function.
extern "C" int sysTickHook(void) {
  if (motorsOn && (uint32_t)(millis() - lastFrameMs) > (uint32_t)(WATCHDOG_MS + DEADMAN_MARGIN_MS) && enIsHigh()) {
    enCut();
    deadmanCuts++;
  }
  return 0;                                      // 0: the core's handler still advances millis()
}

// ------------------------------------------------------------------------------------------------ hw watchdog
#if USE_HW_WDT
// UF2 bootloader (uf2-samdx1 inc_uf2.h, main.c check_start_application): DBL_TAP_PTR is the last word of RAM.
// After a reset that is not power-on it waits 500 ms for a double tap unless that word holds
// DBL_TAP_MAGIC_QUICK_BOOT. During the wait a QT Py bootloader >= 3.11 (BOARD_NEOPIXEL_POWERPIN = PA15) drives
// EN HIGH while D2/D3 float: full drive. Both constants are also present in the bootloader bundled with the core.
static volatile uint32_t *const UF2_DBL_TAP_PTR = (volatile uint32_t *)(HMCRAMC0_ADDR + HMCRAMC0_SIZE - 4);
static const uint32_t UF2_DBL_TAP_MAGIC_QUICK_BOOT = 0xF02669EFul;
static_assert(HMCRAMC0_ADDR + HMCRAMC0_SIZE - 4 == 0x20007FFCul, "DBL_TAP_PTR of a SAMD21 with 32 KB RAM");

static void hwWdtBegin() {                       // same clocking as Adafruit SleepyDog: GCLK2 = OSCULP32K / 32
  GCLK->GENDIV.reg = GCLK_GENDIV_ID(2) | GCLK_GENDIV_DIV(4);
  while (GCLK->STATUS.bit.SYNCBUSY);
  GCLK->GENCTRL.reg = GCLK_GENCTRL_ID(2) | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_OSCULP32K | GCLK_GENCTRL_DIVSEL;
  while (GCLK->STATUS.bit.SYNCBUSY);
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_WDT | GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK2;
  while (GCLK->STATUS.bit.SYNCBUSY);
  WDT->CTRL.reg = 0;                    while (WDT->STATUS.bit.SYNCBUSY);
  WDT->CONFIG.reg = WDT_CONFIG_PER_1K;  while (WDT->STATUS.bit.SYNCBUSY);   // 1024 cycles at 1.024 kHz ~ 1 s
  WDT->EWCTRL.reg = WDT_EWCTRL_EWOFFSET_512;                                // early warning at ~0.5 s
  WDT->INTFLAG.reg = WDT_INTFLAG_EW;
  WDT->INTENSET.reg = WDT_INTENSET_EW;
  // Highest priority: no other interrupt preempts the handler below. But the M0+ never preempts at equal priority,
  // and USB (TinyUSB sets 0), IRremote's TC3 and any IRQ left at its reset default share priority 0: an ISR stuck
  // there (or in Dummy_Handler) also blocks the early warning, and SysTick (2, the deadman) with it. EN then stays
  // HIGH until the WDT reset at ~1 s, with no quick-boot flag. Left as is: lowering those IRQs is untested with
  // TinyUSB and IRremote.
  NVIC_SetPriority(WDT_IRQn, 0);
  NVIC_ClearPendingIRQ(WDT_IRQn);
  NVIC_EnableIRQ(WDT_IRQn);
  WDT->CTRL.reg = WDT_CTRL_ENABLE;      while (WDT->STATUS.bit.SYNCBUSY);
}
static void hwWdtFeed() {
  if (!WDT->STATUS.bit.SYNCBUSY) WDT->CLEAR.reg = WDT_CLEAR_CLEAR_KEY;   // never block on the sync
}
// Early warning: no loop() pass for ~0.5 s (0.43-0.64 s over the OSCULP32K tolerance). Cut EN (SysTick may be
// starved too), set the quick-boot magic so the bootloader skips its 500 ms EN-HIGH wait after the reset, then
// reset at once: any CLEAR value other than the 0xA5 key "will issue an immediate system reset" (SAM D21
// datasheet 18.6.2.4; the write may stall the bus for its ~3 ms sync). The WDT issues that reset, so RCAUSE
// should read WDT (0x20) as after a time-out (INFERRED; bench check B.9 reads it). The magic is written only
// here and in HardFault_Handler, each time right before this reset: left in RAM it would swallow the first tap
// of a double-tap.
extern "C" void WDT_Handler(void) {
  enCut();
  *UF2_DBL_TAP_PTR = UF2_DBL_TAP_MAGIC_QUICK_BOOT;
  WDT->CLEAR.reg = 0;                            // not WDT_CLEAR_CLEAR_KEY: immediate reset
  for (;;) { }
}
#else
static void hwWdtBegin() {}
static void hwWdtFeed() {}
#endif

// A fault stops loop() and SysTick alike (HardFault outranks both): cut EN now instead of when the WDT fires.
// With the WDT running, wait for its early warning (polled: WDT_Handler cannot preempt HardFault), then set the
// magic and reset at once as WDT_Handler does. The wait keeps a build that faults on every pass to ~2 resets/s,
// each of which may give a >= 3.11 bootloader a few ms of EN HIGH. Without the WDT (USE_HW_WDT=0, or a fault in
// setup() before hwWdtBegin()) it spins with EN LOW until a manual reset.
extern "C" void HardFault_Handler(void) {
  enCut();
#if USE_HW_WDT
  if (WDT->CTRL.bit.ENABLE) {
    while (!WDT->INTFLAG.bit.EW) { }             // <= ~0.64 s after the last feed
    *UF2_DBL_TAP_PTR = UF2_DBL_TAP_MAGIC_QUICK_BOOT;
    WDT->CLEAR.reg = 0;                          // not WDT_CLEAR_CLEAR_KEY: immediate reset, RCAUSE WDT
  }
#endif
  for (;;) { }
}

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

// ------------------------------------------------------------------------------------------------ I2C bus
// Wire in core 1.7.17 has no time-outs: a target holding SCL low leaves the SERCOM spin loops waiting forever,
// and a SAMD reset does not free it. Guards:
//  * i2cRecover() before Wire.begin(): if SDA is held low, 9 SCL clocks, then a STOP, up to 3 times. If a line is
//    still LOW, Wire is not started (i2cOk = false), so setup() cannot hang in Wire on a wedged bus; loop()
//    retries about once per second (pollI2cRetry) and starts Wire once the bus is free.
//  * i2cTimeouts() after Wire.begin(): the SCL low time-out (25-35 ms) ends a stuck transfer with BUSERR. It
//    needs GCLK_SERCOM_SLOW at 32 kHz: GCLK1, which the core runs from OSC32K (XOSC32K with a crystal).
//    The inactive-bus time-out (INACTOUT, 50-205 us) stays off: it could cut the AVR's clock stretching.
//  * i2cClearError() before every transfer: INTFLAG.ERROR is cleared only by writing 1 and the core never does,
//    so after one error (a time-out, a lost arbitration) every later transfer would fail at once.
//  * i2cError() after every transfer: the core still counts the bytes of a transfer that failed partway (they
//    read 0xFF). ERROR collects BUSERR, ARBLOST, LOWTOUT and LENERR.
static Sercom *const WIRE_SERCOM = SERCOM1;      // PERIPH_WIRE = sercom1 on graspionBody (D4 = PA16, D5 = PA17)

static void sclLow()     { digitalWrite(PIN_WIRE_SCL, LOW); pinMode(PIN_WIRE_SCL, OUTPUT); }   // OUT=0 first:
static void sclRelease() { pinMode(PIN_WIRE_SCL, INPUT_PULLUP); }                               // never drive HIGH
static bool sclWaitHigh() {                      // a target may stretch the clock: up to ~1 ms
  for (uint16_t i = 0; i < 1000 && digitalRead(PIN_WIRE_SCL) == LOW; i++) delayMicroseconds(1);
  return digitalRead(PIN_WIRE_SCL) == HIGH;
}
static bool i2cRecover() {                       // Wire not started; a few ms; true if both lines are free
  pinMode(PIN_WIRE_SDA, INPUT_PULLUP);           // the body has 5k1 pull-ups; the internal ones only add to them
  sclRelease();
  delayMicroseconds(10);
  for (uint8_t a = 0; a < 3 && digitalRead(PIN_WIRE_SDA) == LOW && sclWaitHigh(); a++) {
    // 9 clocks whatever SDA reads: a target reset mid-transfer that is sending a byte reaches its ACK slot, reads
    // the released SDA as a NACK and lets go. Stopping at its first '1' bit would leave it mid-byte, and its next
    // '0' bit would then block the STOP.
    bool scl = true;
    for (uint8_t i = 0; i < 9 && scl; i++) {
      sclLow(); delayMicroseconds(5);
      sclRelease(); scl = sclWaitHigh(); delayMicroseconds(5);
    }
    if (!scl) break;                             // SCL held low: nothing more to do here
    sclLow(); delayMicroseconds(5);              // STOP: SDA low while SCL is low, then SCL high, then SDA high
    digitalWrite(PIN_WIRE_SDA, LOW); pinMode(PIN_WIRE_SDA, OUTPUT); delayMicroseconds(5);
    sclRelease(); sclWaitHigh(); delayMicroseconds(5);
    pinMode(PIN_WIRE_SDA, INPUT_PULLUP); delayMicroseconds(10);
  }
  const bool ok = digitalRead(PIN_WIRE_SDA) == HIGH && sclWaitHigh();
  pinMode(PIN_WIRE_SDA, INPUT); pinMode(PIN_WIRE_SCL, INPUT);   // as at reset (pull-ups off); Wire takes the pins
  return ok;
}
static void i2cTimeouts() {                      // after Wire.begin(); CTRLA is enable-protected
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_SERCOMX_SLOW | GCLK_CLKCTRL_GEN_GCLK1 | GCLK_CLKCTRL_CLKEN;
  while (GCLK->STATUS.bit.SYNCBUSY);
  SercomI2cm &m = WIRE_SERCOM->I2CM;
  m.CTRLA.bit.ENABLE = 0;  while (m.SYNCBUSY.bit.ENABLE);
  m.CTRLA.reg |= SERCOM_I2CM_CTRLA_LOWTOUTEN;
  m.CTRLA.bit.ENABLE = 1;  while (m.SYNCBUSY.bit.ENABLE);
  m.STATUS.bit.BUSSTATE = 1;  while (m.SYNCBUSY.bit.SYSOP);   // bus idle, as SERCOM::enableWIRE() does
}
static void i2cStart() {                         // after a successful i2cRecover(); once per boot
  Wire.begin();                                  // 100 kHz: the AVR's I2C speed limit is UNKNOWN
  i2cTimeouts();
  i2cOk = true;
}
static inline void i2cClearError() { WIRE_SERCOM->I2CM.INTFLAG.reg = (uint8_t)SERCOM_I2CM_INTFLAG_ERROR; }
static inline bool i2cError() { return WIRE_SERCOM->I2CM.INTFLAG.bit.ERROR; }

// ------------------------------------------------------------------------------------------------ power
// AVR "BIOS" at 0x36: plain 5-byte read, no register pointer. Bytes 0-1 = SYS / USB mV, 2-3 = mVbat,
// big-endian (INFERRED, HARDWARE.md 3.3: verify against a multimeter before trusting MVBAT_ABORT).
// A transfer that failed partway still returns 5 bytes (the rest read 0xFF): INTFLAG.ERROR catches it.
// Implausible values (0xFFFF, a battery outside MVBAT_PLAUSIBLE..MVBAT_MAX) are read errors too. rawSys/rawBat
// keep the words before those bounds, so 'i' shows what the AVR sends (bench check B.1: the encoding is INFERRED).
static bool readPower(uint16_t &s, uint16_t &b) {
  uint8_t buf[5];
  if (!i2cOk) return false;
  i2cClearError();
  if (Wire.requestFrom((uint8_t)0x36, (uint8_t)5) != 5 || i2cError()) return false;
  for (uint8_t i = 0; i < 5; i++) buf[i] = (uint8_t)Wire.read();
  s = rawSys = (uint16_t)((buf[0] << 8) | buf[1]);
  b = rawBat = (uint16_t)((buf[2] << 8) | buf[3]);
  return s <= MVSYS_MAX && b >= MVBAT_PLAUSIBLE && b <= MVBAT_MAX;
}
static uint16_t readBatNow() {                   // 0 = read error (counted; usbPresent and the latch untouched)
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
  else if (magLastSat) r.flags |= F_MAG_SAT;     // the rest baseline itself may clip
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

// A run of PINGs (the link test sends ~9/s, which would fill MAX_RECS in 45 s) shares one record: n_frames
// counts them, t_end_ms is the last one. A PING inside a segment only counts as a frame of the segment.
static void logPing(uint32_t now) {
  if (recOpen) { recs[nRecs - 1].n_frames++; return; }
  if (nRecs > 0) {
    LogRec &p = recs[nRecs - 1];
    if (p.kind == REC_PING && p.n_frames < 0xFFFF) { p.n_frames++; p.t_end_ms = now; return; }
  }
  LogRec *r = newRec(REC_PING, now);
  if (!r) return;
  r->end_reason = END_PING;
  r->n_frames = 1;
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
  if (lowBat) {                                  // never drive again until 'r' over USB or a reset
    setDrive(0, 0, now);
    if (f.op != CALIB_OP_STOP && f.op != CALIB_OP_DRIVE) flash(true, now);
    return;
  }
  // Only frames that command the drive feed the command watchdog (lastFrameMs, set before setDrive so the
  // SysTick deadman never sees fresh motion with a stale stamp). PING, unknown ops and a DRIVE outside a
  // segment do not: they must not keep earlier motion alive.
  switch (f.op) {
    case CALIB_OP_STOP:
      lastFrameMs = now;
      closeRec(END_STOP, now);
      setDrive(0, 0, now);
      break;
    case CALIB_OP_SEG:
      lastFrameMs = now;
      closeRec(END_NEXT_SEG, now);
      if (!i2cOk) {                              // no battery reads, so no low-battery abort: never calibrate.
        setDrive(0, 0, now);                     // Red flash as for the latch, so the PC pauses for the operator
        flash(true, now);                        // (power-cycle the bot; 'i' shows i2c=0)
        break;
      }
      openSeg(f.L, f.R, now);
      flash(false, now);
      break;
    case CALIB_OP_DRIVE:
      if (!segActive) {                          // SEG missed or the segment ended (watchdog, RECENTRE): the PC
        st.drive_no_seg++;                       // thinks a segment runs that the bot does not have. Stop: never
        setDrive(0, 0, now);                     // unlogged motion, and the PC sees no flash
        break;
      }
      lastFrameMs = now;
      if (recOpen) {
        LogRec &r = recs[nRecs - 1];
        r.n_frames++;
        if ((f.L != curL || f.R != curR) && r.n_changes < 255) r.n_changes++;
      }
      setDrive(f.L, f.R, now);
      break;
    case CALIB_OP_RECENTRE:
      lastFrameMs = now;
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

// Called before pollIR(), so a loop() stall longer than WATCHDOG_MS always ends the segment, even when a frame
// decoded during the stall is waiting. Also does the bookkeeping after a SysTick deadman cut.
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
  if (b == 0) return;                            // read error or implausible value (counted in pwr_err)
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
// returns a status byte: bit 5 SM_MODE (conversion still running), bit 4 ERROR, bit 2 RS (reset done),
// bits 1:0 D (an RR / RM response carries 2 * D + 2 data bytes).
static const uint8_t MLX_ADDR = MLX90393_I2C_ADDRESS;   // 0x10 (variant.h); the Adafruit default 0x0C is wrong
static const uint8_t MLX_SM_XYZ = 0x3E, MLX_RM_XYZ = 0x4E, MLX_EX = 0x80, MLX_RT = 0xF0, MLX_RR = 0x50, MLX_WR = 0x60;
static const uint8_t MLX_ST_SM = 0x20, MLX_ST_ERR = 0x10, MLX_ST_RS = 0x04, MLX_ST_D = 0x03, MLX_ST_FAIL = 0xFF;
static const uint16_t MLX_CONF1 = 0x0C | (7u << 4);   // HALLCONF 0xC, GAIN_SEL 7 ("1x" in the Adafruit naming)
static const uint16_t MLX_CONF3_MASK = 0x07FF;        // RES_Z, RES_Y, RES_X, DIG_FILT, OSR (OSR2 kept)
static const uint16_t MLX_CONF3 = (2u << 9) | (2u << 7) | (2u << 5) | (2u << 2) | 0u;   // 18 bit, filter 2, OSR 0
static const float MLX_LSB_XY = 0.601f, MLX_LSB_Z = 0.968f;   // uT/LSB for HALLCONF 0xC, GAIN_SEL 7, RES 18
static const int32_t MLX_SAT_LSB = 21000;             // 95 % of the RES 2 range, +-22000 LSB (datasheet Table 21)

static uint8_t mlxCmd(const uint8_t *tx, uint8_t ntx, uint8_t *rx, uint8_t nrx) {
  if (!i2cOk) return MLX_ST_FAIL;
  i2cClearError();
  Wire.beginTransmission(MLX_ADDR);
  Wire.write(tx, ntx);
  if (Wire.endTransmission() != 0 || i2cError()) return MLX_ST_FAIL;
  if (Wire.requestFrom(MLX_ADDR, (uint8_t)(nrx + 1)) != nrx + 1 || i2cError()) return MLX_ST_FAIL;
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
static bool mlxBegin() {                         // blocking (~10 ms): setup(), or pollI2cRetry() with the motors off
  if (!i2cOk) return false;
  uint8_t c = MLX_EX;
  mlxCmd(&c, 1, nullptr, 0);
  c = MLX_RT;
  i2cClearError();
  Wire.beginTransmission(MLX_ADDR); Wire.write(c);
  if (Wire.endTransmission() != 0 || i2cError()) return false;
  delay(5);
  i2cClearError();
  if (Wire.requestFrom(MLX_ADDR, (uint8_t)1) != 1 || i2cError() || !(Wire.read() & MLX_ST_RS)) return false;
  uint16_t r0, r2;
  if (!mlxRead(0, r0) || !mlxWrite(0, (uint16_t)((r0 & ~0x007Fu) | MLX_CONF1))) return false;
  if (!mlxRead(2, r2) || !mlxWrite(2, (uint16_t)((r2 & ~MLX_CONF3_MASK) | MLX_CONF3))) return false;
  return mlxRead(0, r0) && (r0 & 0x7F) == MLX_CONF1 && mlxRead(2, r2) && (r2 & MLX_CONF3_MASK) == MLX_CONF3;
}

static int16_t magUT(uint8_t hi, uint8_t lo, float lsb, bool &sat) {   // RES 18: unsigned with a 0x8000 offset
  const int32_t c = (int32_t)((hi << 8) | lo) - 32768;                // |c| * 0.968 <= 31719: always fits int16
  if (c >= MLX_SAT_LSB || c <= -MLX_SAT_LSB) sat = true;
  return (int16_t)lroundf((float)c * lsb);
}

static void pollMag(uint32_t now) {
  if (!magOk) return;
  if (!magPending) {
    if (now - magStartMs < MAG_PERIOD_MS) return;
    magStartMs = now;
    const uint8_t s = mlxCmd(&MLX_SM_XYZ, 1, nullptr, 0);
    // Stamped after the SM has gone out (now is loop-top time, up to ~1.5 ms earlier), so MAG_CONV_MS holds
    if (s != MLX_ST_FAIL && !(s & MLX_ST_ERR)) { magPending = true; magStartMs = magTryMs = millis(); }
    else st.mag_err++;
    return;
  }
  if (now - magStartMs < MAG_CONV_MS || now - magTryMs < MAG_RETRY_MS) return;
  magTryMs = now;
  uint8_t rx[6];
  const uint8_t s = mlxCmd(&MLX_RM_XYZ, 1, rx, 6);
  // Valid: ERROR clear and 6 data bytes announced. SM_MODE is not checked: it can still read 1 up to 132 us
  // after DRDY (TINT_RDY, datasheet Table 8 note 6) with valid data, and the RM has cleared DRDY, so a retry
  // would only get ERROR. A read before the conversion ends sets ERROR (datasheet 15.2) and is retried.
  // All six data bytes 0xFF (outside the +-22000 LSB range on every axis): the bus failed after the status.
  const bool valid = s != MLX_ST_FAIL && !(s & MLX_ST_ERR) && (s & MLX_ST_D) == 2;
  const bool allFF = valid && (rx[0] & rx[1] & rx[2] & rx[3] & rx[4] & rx[5]) == 0xFF;
  if (valid && !allFF) {
    bool sat = false;
    magPending = false;
    magLast[0] = magUT(rx[0], rx[1], MLX_LSB_XY, sat);
    magLast[1] = magUT(rx[2], rx[3], MLX_LSB_XY, sat);
    magLast[2] = magUT(rx[4], rx[5], MLX_LSB_Z, sat);
    magLastSat = sat;                            // newRec flags a clipped mag_rest
    magLastMs = now;
    if (recOpen && motorsOn && (now - driveChangeMs) >= SETTLE_MS) {
      for (uint8_t k = 0; k < 3; k++) accMag[k] += magLast[k];
      accMagN++;
      if (sat) accMagSat = true;                 // only samples averaged into mag_load flag it
    }
  } else if (valid || now - magStartMs > MAG_GIVEUP_MS) {   // valid but all 0xFF: DRDY is gone, do not retry
    magPending = false;
    st.mag_err++;
  }
}

// I2C down (a line held LOW since boot): retry the bus recovery about once per second while the motors are off
// (a few ms per failed try). Once the bus is free, Wire starts and the MLX is set up (~10 ms), and SEG works again.
static void pollI2cRetry(uint32_t now) {
  if (i2cOk || motorsOn || now - i2cRetryMs < I2C_RETRY_MS) return;
  i2cRetryMs = now;
  if (!i2cRecover()) return;
  i2cStart();
  magOk = mlxBegin();
}

// ------------------------------------------------------------------------------------------------ USB serial
// Every reply goes through one line buffer that is written only as far as the CDC TX FIFO (256 B) has room.
// Adafruit_USBD_CDC::write() loops on yield() while the FIFO is full and the host holds DTR, so a host that
// stops reading would otherwise stall loop(). A new command is read only after the previous reply is out.
static char txBuf[300];                          // longest lines: "# I ..." and "# S ..." (ten-digit counters), < 280 B
static uint16_t txLen = 0, txPos = 0;            // pending line: txBuf[txPos..txLen)

static bool txPump() {                           // writes what fits; true once nothing is pending
  if (txPos < txLen) {
    if (!Serial) { txLen = txPos = 0; return true; }   // host closed the port (DTR low): drop the line
    const int room = Serial.availableForWrite();
    if (room > 0) txPos += Serial.write((const uint8_t *)txBuf + txPos, (size_t)min(room, (int)(txLen - txPos)));
    if (txPos < txLen) return false;
  }
  txLen = txPos = 0;
  return true;
}
static void txSet(int n) {                       // queue the line snprintf just wrote into txBuf
  txLen = (uint16_t)constrain(n, 0, (int)sizeof(txBuf) - 1);
  txPos = 0;
}

static void queueInfo(const char *pre) {
  txSet(snprintf(txBuf, sizeof(txBuf),
      "%sI fw=" FW_VERSION " proto=%d id=%u id_override=%d serial=%08lX%08lX%08lX%08lX pwm_hz=%d drive_max=%d"
      " watchdog_ms=%u mvbat_abort=%u hw_wdt=%d bench_hang=%d rcause=0x%lX mag=%d i2c=%d mvsys=%u mvbat=%u"
      " raw=%04X%04X lowbat=%d nrec=%u millis=%lu\r\n",
      pre, (int)CALIB_PROTO_VERSION, (unsigned)botId, (int)BOT_ID_OVERRIDE, (unsigned long)serialWords[0],
      (unsigned long)serialWords[1], (unsigned long)serialWords[2], (unsigned long)serialWords[3],
      PWM_40KHZ ? 40000 : 732, (int)DRIVE_MAX, (unsigned)WATCHDOG_MS, (unsigned)MVBAT_ABORT, (int)USE_HW_WDT,
      (int)BENCH_HANG, (unsigned long)resetCause, magOk ? 1 : 0, i2cOk ? 1 : 0, (unsigned)mvSys, (unsigned)mvBat,
      (unsigned)rawSys, (unsigned)rawBat, lowBat ? 1 : 0, (unsigned)nRecs, (unsigned long)millis()));
}

static void queueStats(const char *pre) {
  txSet(snprintf(txBuf, sizeof(txBuf),
      "%sS rx_ok=%lu crc_err=%lu other_addr=%lu ir_other=%lu bad_op=%lu drive_no_seg=%lu bcast_stop=%lu"
      " wd_trips=%lu lowbat_trips=%lu pwr_err=%lu mag_err=%lu rec_overflow=%lu deadman_cuts=%lu\r\n",
      pre, (unsigned long)st.rx_ok, (unsigned long)st.crc_err, (unsigned long)st.other_addr,
      (unsigned long)st.ir_other, (unsigned long)st.bad_op, (unsigned long)st.drive_no_seg,
      (unsigned long)st.bcast_stop, (unsigned long)st.wd_trips, (unsigned long)st.lowbat_trips,
      (unsigned long)st.pwr_err, (unsigned long)st.mag_err, (unsigned long)st.rec_overflow,
      (unsigned long)deadmanCuts));
}

static void queueRec(const LogRec &r) {
  txSet(snprintf(txBuf, sizeof(txBuf), "%u,%lu,%lu,%d,%d,%u,%u,%u,%u,%u,%u,%u,%u,%d,%d,%d,%d,%d,%d\r\n",
           r.kind, (unsigned long)r.t_start_ms, (unsigned long)r.t_end_ms, r.L, r.R, r.end_reason, r.n_frames,
           r.n_changes, r.flags, r.mv_rest, r.mv_load_min, r.mv_load_mean, r.n_mag, r.mag_rest[0], r.mag_rest[1],
           r.mag_rest[2], r.mag_load[0], r.mag_load[1], r.mag_load[2]));
}

static void pollDump() {                         // header lines, records, footer: queue the next once one is out
  while (dumpIdx >= 0 && txPump()) {
    if (!Serial) { dumpIdx = -1; return; }       // host closed the port: abandon (the log is kept)
    if (dumpHdr == 0)      { queueInfo("# "); dumpHdr = 1; }
    else if (dumpHdr == 1) { queueStats("# "); dumpHdr = 2; }
    else if (dumpHdr == 2) {
      txSet(snprintf(txBuf, sizeof(txBuf), "%s\r\n",
                     "kind,t_start_ms,t_end_ms,L,R,end_reason,n_frames,n_changes,flags,mv_rest,mv_load_min,"
                     "mv_load_mean,n_mag,mag_rest_x,mag_rest_y,mag_rest_z,mag_load_x,mag_load_y,mag_load_z"));
      dumpHdr = 3;
    }
    else if (dumpIdx < (int32_t)nRecs) queueRec(recs[dumpIdx++]);
    else { txSet(snprintf(txBuf, sizeof(txBuf), "# end n=%u\r\n", (unsigned)nRecs)); dumpIdx = -1; }
  }
}

static void pollUsb() {
  pollDump();
  while (txPump() && dumpIdx < 0 && Serial.available() > 0) {   // one command at a time
    switch (Serial.read()) {
      case 'i': queueInfo(""); break;
      case 's': queueStats(""); break;
      case 'd': dumpIdx = 0; dumpHdr = 0; pollDump(); break;
      case 'c':
        nRecs = 0; recOpen = false; memset(&st, 0, sizeof(st)); deadmanCuts = 0;
        txSet(snprintf(txBuf, sizeof(txBuf), "K cleared\r\n"));
        break;
      case 'r':
        lowBat = false; lowCount = 0;
        txSet(snprintf(txBuf, sizeof(txBuf), "K lowbat cleared\r\n"));
        break;
#if BENCH_HANG
      case 'H':                                  // EN should fall WATCHDOG_MS + DEADMAN_MARGIN_MS later (deadman)
        lastFrameMs = millis();                  // or at the WDT early warning, then the MCU resets (rcause=0x20)
        setDrive(80, 80, lastFrameMs);
        for (;;) { }
#endif
      default: break;
    }
  }
}

// ------------------------------------------------------------------------------------------------ setup / loop
void setup() {
  // FIRST: D12 arrives as a floating input, or as an output driven HIGH by a QT Py UF2 bootloader >= 3.11
  // (it powers its NeoPixel from PA15 = D12), with D2/D3 floating: full drive until this line.
  pinMode(BUZ_ENABLE, OUTPUT); digitalWrite(BUZ_ENABLE, LOW);
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
  if (i2cRecover()) i2cStart();                  // a line still held LOW: I2C off, retried from loop() (i2c=0)
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
  commandWatchdog(now);
  pollIR(now);
  updateFlash(now);
  pollI2cRetry(now);
  pollPower(now);
  pollMag(now);
  pollUsb();
  hwWdtFeed();
}                                                // returning lets TinyUSB run (yield)
