// calib_bot.ino -- Phase 0 "learning to walk" calibration firmware for the GRASPion body (SAMD21E18A).
//
// UNTESTED on hardware: compiled only.
//
// What it does: receives addressed 32-bit NEC-timed frames from the overhead IR blaster on the TSOP (D7) and
// drives the two vibration motors as told. The frame format is in calib_protocol.h: bot ID, op, CRC-5, L and R
// drive codes. Frames with a bad CRC, frames for another ID and NEC repeat codes are ignored; the broadcast ID
// (0xFF) is honoured for STOP only. The bot ID is an FNV-1a hash of the SAMD21 128-bit serial (calib_bot_id),
// or BOT_ID_OVERRIDE when that build option is non-zero.
//
// Ops:
//  * STOP      motors off; closes the open segment.
//  * SEG       starts a segment at (L, R): sets the drive and flashes the top NeoPixel blue (~70 ms at ~15 %)
//              for the camera (time sync and link-latency measurement).
//  * DRIVE     keep-alive or level change inside the open segment (no flash). Outside a segment it stops the
//              motors and counts drive_no_seg: the PC thinks a segment runs that the bot does not have.
//  * RECENTRE  drives (L, R) without a segment; closes the open segment.
//  * PING      blue flash only; the motors and the watchdog are untouched.
//
// Command watchdog: WATCHDOG_MS (300) after the last frame that commanded the drive (STOP, SEG, DRIVE inside a
// segment, RECENTRE) the motors stop and the segment closes, counting wd_trips. PING and ignored frames never
// feed it, so they cannot keep earlier motion alive.
//
// Motors (HARDWARE.md 4.4): D2 = right, D3 = left, D12 = shared DRV2603 enable, LOW whenever both drives are 0.
// analogWrite(pin, 128 + drive) at the core's default 732 Hz; |drive| <= DRIVE_MAX (120 ~ 3.07 V on a motor
// rated 2.7 V, max 3.3 V).
//
// USB serial (115200, TinyUSB), one character per command:
//   'i'  I fw=calib_bot/3 proto=<n> id=<n> id_override=<n> serial=<hex> pwm_hz=732 drive_max=120
//        watchdog_ms=300 rcause=0x<hex> frames_ok=<n> millis=<n>
//   's'  S frames_ok=<n> crc_err=<n> other_addr=<n> drive_no_seg=<n> wd_trips=<n>
//   'c'  clears the stats (replies "K cleared")
// A reply is written only as far as the CDC FIFO has room, so a host that stops reading cannot stall loop().
//
// Build (HARDWARE.md 11): arduino-cli compile --fqbn graspion:samd:graspionBody:opt=fastest,usbstack=tinyusb
// calibration/firmware/calib_bot. NOT the QT Py M0 board: it drives D12 = motor enable HIGH at boot. Libraries:
// IRremote@4.7.1 and Adafruit NeoPixel. On an ID collision add
// --build-property "compiler.cpp.extra_flags=-DBOT_ID_OVERRIDE=<1..254>".

#if defined(USE_TINYUSB)
#include <Adafruit_TinyUSB.h>      // Serial links only when this header reaches the build (HARDWARE.md 11.11)
#endif
#define DECODE_NEC                 // NEC family only (NEC, NEC2, Onkyo, Apple); the raw 32 bits are used
#define EXCLUDE_EXOTIC_PROTOCOLS
#define EXCLUDE_UNIVERSAL_PROTOCOLS
#define NO_LED_FEEDBACK_CODE       // LED_BUILTIN (D13) is not connected on the body
#define RAW_BUFFER_LENGTH 100      // NEC needs 68 entries
#include <IRremote.hpp>

#include <Adafruit_NeoPixel.h>
#include "calib_protocol.h"
#include "calib_protocol_vectors.h"  // compile-time check against the Python mirror

// ------------------------------------------------------------------------------------------------ settings
#ifndef BOT_ID_OVERRIDE
#define BOT_ID_OVERRIDE 0          // 1..254 replaces the serial-derived ID (use on ID collisions)
#endif

#define FW_VERSION "calib_bot/3"

static const int16_t  DRIVE_MAX = 120;          // |drive| cap; 120 ~ 3.07 V on the motor (2.7 V rated, 3.3 V max)
static const uint16_t WATCHDOG_MS = 300;        // ~2-3 IR frame periods; raise if the bench link test shows
                                                // < 9 frames/s
static const uint16_t FLASH_MS = 70;            // >= 2 frames at 30 fps
static const uint8_t  FLASH_LEVEL = 38;         // ~15 % of one channel (400 mA overcurrent trip, HARDWARE.md 3.2)

// ------------------------------------------------------------------------------------------------ state
struct Stats {
  uint32_t frames_ok;              // valid frames addressed to this bot (broadcast STOPs are not counted)
  uint32_t crc_err;                // 32-bit frames that failed calib_unpack (CRC or a drive code of 0)
  uint32_t other_addr;             // valid frames for another bot ID
  uint32_t drive_no_seg;           // DRIVE with no open segment: motors stopped instead
  uint32_t wd_trips;               // command watchdog stops
};

static Adafruit_NeoPixel px(NUM_NEOPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);
static Stats st;

static uint8_t botId = 0;
static uint32_t serialWords[4];
static uint32_t resetCause = 0;                  // PM->RCAUSE at boot, reported by 'i'

static bool motorsOn = false;                    // at least one drive is non-zero (EN HIGH)
static bool segActive = false;                   // a SEG started and has not ended
static uint32_t lastFrameMs = 0;                 // last frame that commanded the drive (see handleFrame)

static bool flashOn = false;
static uint32_t flashUntil = 0;

// ------------------------------------------------------------------------------------------------ motors
static void setDrive(int16_t L, int16_t R) {
  L = constrain(L, -DRIVE_MAX, DRIVE_MAX);
  R = constrain(R, -DRIVE_MAX, DRIVE_MAX);
  motorsOn = (L != 0 || R != 0);
  if (!motorsOn) digitalWrite(BUZ_ENABLE, LOW);  // shared enable: LOW whenever both motors stop
  analogWrite(BUZ_LEFT, 128 + L);                // 128 is the true zero (the examples' 127 is -0.8 %)
  analogWrite(BUZ_RIGHT, 128 + R);
  if (motorsOn) digitalWrite(BUZ_ENABLE, HIGH);
}

// ------------------------------------------------------------------------------------------------ LED
static void ledSet(uint8_t r, uint8_t g, uint8_t b) {   // top pixel only; show() blocks IRQs ~60 us
  px.setPixelColor(NEOPIXEL_TOP, px.Color(r, g, b));
  px.setPixelColor(NEOPIXEL_BOTTOM, 0);
  px.show();
}
static void flashBlue(uint32_t now) {
  ledSet(0, 0, FLASH_LEVEL);
  flashOn = true;
  flashUntil = now + FLASH_MS;
}
static void updateFlash(uint32_t now) {
  if (flashOn && (int32_t)(now - flashUntil) >= 0) { ledSet(0, 0, 0); flashOn = false; }
}

// ------------------------------------------------------------------------------------------------ IR frames
static void handleFrame(uint32_t raw, uint32_t now) {
  CalibFrame f;
  if (calib_unpack(raw, &f) != CALIB_OK) { st.crc_err++; return; }
  if (f.id == CALIB_ADDR_BROADCAST) {
    if (f.op == CALIB_OP_STOP) { segActive = false; setDrive(0, 0); }
    return;
  }
  if (f.id != botId) { st.other_addr++; return; }
  st.frames_ok++;
  // Only frames that command the drive feed the command watchdog (lastFrameMs). PING, unknown ops and a DRIVE
  // outside a segment do not: they must not keep earlier motion alive.
  switch (f.op) {
    case CALIB_OP_STOP:
      lastFrameMs = now;
      segActive = false;
      setDrive(0, 0);
      break;
    case CALIB_OP_SEG:
      lastFrameMs = now;
      segActive = true;
      setDrive(f.L, f.R);
      flashBlue(now);
      break;
    case CALIB_OP_DRIVE:
      if (!segActive) {                          // SEG missed or the segment ended (watchdog, RECENTRE): the PC
        st.drive_no_seg++;                       // thinks a segment runs that the bot does not have. Stop, so
        setDrive(0, 0);                          // the PC sees no flash and no motion it cannot account for
        break;
      }
      lastFrameMs = now;
      setDrive(f.L, f.R);
      break;
    case CALIB_OP_RECENTRE:
      lastFrameMs = now;
      segActive = false;
      setDrive(f.L, f.R);
      break;
    case CALIB_OP_PING:
      flashBlue(now);
      break;
    default:
      break;
  }
}

static void pollIR(uint32_t now) {
  if (!IrReceiver.decode()) return;
  const IRData &d = IrReceiver.decodedIRData;
  const bool full = d.numberOfBits == 32 && !(d.flags & IRDATA_FLAGS_WAS_OVERFLOW) &&
                    (d.protocol == NEC || d.protocol == NEC2 || d.protocol == ONKYO || d.protocol == APPLE);
  const uint32_t raw = (uint32_t)d.decodedRawData;
  IrReceiver.resume();
  if (full) handleFrame(raw, now);               // NEC repeat codes carry no address: ignored
}

// Called before pollIR(), so a loop() stall longer than WATCHDOG_MS always ends the segment, even when a frame
// decoded during the stall is waiting.
static void commandWatchdog(uint32_t now) {
  if (motorsOn && (now - lastFrameMs) > WATCHDOG_MS) {
    segActive = false;
    setDrive(0, 0);
    st.wd_trips++;
  }
}

// ------------------------------------------------------------------------------------------------ USB serial
// One reply at a time, written only as far as the CDC TX FIFO (256 B) has room. Adafruit_USBD_CDC::write()
// loops on yield() while the FIFO is full and the host holds DTR, so a host that stops reading would otherwise
// stall loop(). The next command is read once the previous reply is out.
static char txBuf[224];                          // the 'i' line stays < 190 B even with ten-digit counters
static uint16_t txLen = 0, txPos = 0;            // pending reply: txBuf[txPos..txLen)

static bool txPump() {                           // writes what fits; true once nothing is pending
  if (txPos < txLen) {
    if (!Serial) { txLen = txPos = 0; return true; }   // host closed the port (DTR low): drop the reply
    const int room = Serial.availableForWrite();
    if (room > 0) txPos += Serial.write((const uint8_t *)txBuf + txPos, (size_t)min(room, (int)(txLen - txPos)));
    if (txPos < txLen) return false;
  }
  txLen = txPos = 0;
  return true;
}
static void txQueue(int n) {                     // queue the line snprintf just wrote into txBuf
  txLen = (uint16_t)constrain(n, 0, (int)sizeof(txBuf) - 1);
  txPos = 0;
}

static void pollUsb() {
  while (txPump() && Serial.available() > 0) {   // one command at a time
    switch (Serial.read()) {
      case 'i':
        txQueue(snprintf(txBuf, sizeof(txBuf),
            "I fw=" FW_VERSION " proto=%d id=%u id_override=%d serial=%08lX%08lX%08lX%08lX pwm_hz=732"
            " drive_max=%d watchdog_ms=%u rcause=0x%lX frames_ok=%lu millis=%lu\r\n",
            (int)CALIB_PROTO_VERSION, (unsigned)botId, (int)BOT_ID_OVERRIDE, (unsigned long)serialWords[0],
            (unsigned long)serialWords[1], (unsigned long)serialWords[2], (unsigned long)serialWords[3],
            (int)DRIVE_MAX, (unsigned)WATCHDOG_MS, (unsigned long)resetCause, (unsigned long)st.frames_ok,
            (unsigned long)millis()));
        break;
      case 's':
        txQueue(snprintf(txBuf, sizeof(txBuf),
            "S frames_ok=%lu crc_err=%lu other_addr=%lu drive_no_seg=%lu wd_trips=%lu\r\n",
            (unsigned long)st.frames_ok, (unsigned long)st.crc_err, (unsigned long)st.other_addr,
            (unsigned long)st.drive_no_seg, (unsigned long)st.wd_trips));
        break;
      case 'c':
        memset(&st, 0, sizeof(st));
        txQueue(snprintf(txBuf, sizeof(txBuf), "K cleared\r\n"));
        break;
      default:
        break;                                   // line endings and unknown commands
    }
  }
}

// ------------------------------------------------------------------------------------------------ setup / loop
void setup() {
  // FIRST: D12 arrives as a floating input, or as an output driven HIGH by a QT Py UF2 bootloader >= 3.11
  // (it powers its NeoPixel from PA15 = D12), with D2/D3 floating: full drive until this line (HARDWARE.md 11.1).
  pinMode(BUZ_ENABLE, OUTPUT); digitalWrite(BUZ_ENABLE, LOW);
  pinMode(IR_TX, OUTPUT); digitalWrite(IR_TX, HIGH);            // IR LED off; Serial1 is never started
  analogWriteResolution(8);
  setDrive(0, 0);
  resetCause = PM->RCAUSE.reg;

  serialWords[0] = *(volatile uint32_t *)0x0080A00C;            // SAMD21 128-bit serial, iSerial word order
  serialWords[1] = *(volatile uint32_t *)0x0080A040;
  serialWords[2] = *(volatile uint32_t *)0x0080A044;
  serialWords[3] = *(volatile uint32_t *)0x0080A048;
  botId = BOT_ID_OVERRIDE ? (uint8_t)BOT_ID_OVERRIDE
                          : calib_bot_id(serialWords[0], serialWords[1], serialWords[2], serialWords[3]);

  Serial.begin(115200);                          // never while(!Serial): it blocks forever on battery
  px.begin();
  flashBlue(millis());                           // boot blink (cleared by updateFlash)

  IrReceiver.begin(IR_RX, DISABLE_LED_FEEDBACK); // D7 becomes GPIO: Serial1 RX is dead from here on
}

void loop() {
  const uint32_t now = millis();
  commandWatchdog(now);
  pollIR(now);
  updateFlash(now);
  pollUsb();
}                                                // returning lets TinyUSB run (yield)
