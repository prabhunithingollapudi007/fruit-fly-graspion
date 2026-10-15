// ir_blaster.ino -- USB-serial -> IR bridge for the GRASPion calibration rig (Phase 0).
//
// UNTESTED: compiled with arduino-cli for arduino:avr:nano only; never run. Bench-test it with a TSOP and a
// scope, then with one bot (calibration/README.md, step B), before any calibration run.
//
// Boards: Arduino Nano (ATmega328P, FQBN arduino:avr:nano) or any RP2040 board with IRremote support
// (Pico, QT Py RP2040, ...). Native-USB boards (RP2040) avoid the 1-16 ms latency of a USB-UART chip
// (FEASIBILITY.md 3.3). Library: IRremote@4.7.1.
//
// Wiring: IR_LED_PIN -> 100 R -> gate of a logic-level N-MOSFET (10 k gate pull-down); 4-8 TSAL6200 940 nm
// LEDs, each with its own series resistor, from +5 V to the drain. See calibration/README.md for values.
//
// Serial protocol, 115200 baud, one ASCII line per request, one reply line per request:
//   F <hex8>          send a raw 32-bit frame (rejected with "E crc" unless calib_check() passes)
//   f <hex8>          send a raw frame without checking it (bench tests of the bot's CRC / address filter)
//   X <id> <op> <L> <R>  pack and send (manual tests from a serial terminal)
//   G <ms>            set the minimum start-to-start frame period, 100..2000 (default 110 ms = NEC repeat period)
//   P                 ping (no IR)          -> "K P <millis>"
//   V                 version / settings    -> "I ir_blaster/3 proto=2 pin=.. period_ms=.."
// Replies: "K <hex8> <t_start_ms> <t_end_ms>" after the frame has left the LED, or "E <reason>". An empty line
// gets no reply; a line longer than 47 characters is dropped whole and answered "E long". A CAN byte (0x18)
// drops the line received so far, unrun and without a reply: the PC's resync sends CAN + '\n', so a request
// left half-written by a killed PC process can never run.
// A request that arrives before the period has elapsed waits; the PC sends one request and waits for the
// reply (no pipelining), so the 64-byte serial buffer cannot overflow during the blocking send.
//
// Frame length: without the NEC inverse bytes it depends on the payload, 49.84 ms + 1.12 ms per 1 bit
// (50-86 ms; 68 ms only for 16 one bits). The bot's receiver ends a frame after 8 ms of silence
// (IRremote RECORD_GAP_MICROS), so the period floor of 100 ms keeps even the longest frames apart.
//
// Safety: the blaster has no state that keeps a bot moving. If the PC stops sending, every bot stops by its
// own command watchdog (~300 ms).

#if defined(USE_TINYUSB)
#include <Adafruit_TinyUSB.h>        // SAMD/RP2040 TinyUSB stack: linked only when the sketch or a library header it includes pulls it in
#endif
#define NO_LED_SEND_FEEDBACK_CODE
#include <IRremote.hpp>
#include "calib_protocol.h"
#include "calib_protocol_vectors.h"  // compile-time check against the Python mirror

#ifndef IR_LED_PIN
#define IR_LED_PIN 3                 // Nano: any pin works with IRremote's software carrier; 3 is the IRremote default
#endif
#define FW_VERSION "ir_blaster/3"    // 3: CAN drops a partial line
#define PERIOD_MIN_MS 100            // longest frame (85.7 ms) + the receiver's 8 ms end-of-frame gap + margin
#define PERIOD_MAX_MS 2000
#define LINE_CANCEL '\x18'           // CAN: drop the partial line (graspion_calib.bridge.RESYNC)

static uint16_t periodMs = 110;
static uint32_t lastStartMs = 0;
static bool sentOnce = false;
static char line[48];
static uint8_t lineLen = 0;
static bool lineDrop = false;        // overlong line: skip to its '\n', then reply "E long"

static void sendFrame(uint32_t raw) {
  if (sentOnce) {
    while ((uint32_t)(millis() - lastStartMs) < periodMs) {}
  }
  const uint32_t t0 = millis();
  lastStartMs = t0;
  sentOnce = true;
  IrSender.sendNECRaw(raw, 0);       // 32 bits LSB first, no repeats; blocks 50-86 ms (number of 1 bits)
  const uint32_t t1 = millis();
  char out[40];
  snprintf(out, sizeof(out), "K %08lX %lu %lu", (unsigned long)raw, (unsigned long)t0, (unsigned long)t1);
  Serial.println(out);
}

static bool parseHex32(const char *s, uint32_t &v) {
  while (*s == ' ') s++;
  if (!*s) return false;
  char *end;
  v = strtoul(s, &end, 16);
  while (*end == ' ' || *end == '\r') end++;
  return *end == '\0';
}

static void handleLine(char *s) {
  const char cmd = s[0];
  uint32_t raw;
  switch (cmd) {
    case 'F':
      if (!parseHex32(s + 1, raw)) { Serial.println(F("E parse")); return; }
      if (calib_check(raw) != CALIB_OK) { Serial.println(F("E crc")); return; }
      sendFrame(raw);
      return;
    case 'f':
      if (!parseHex32(s + 1, raw)) { Serial.println(F("E parse")); return; }
      sendFrame(raw);
      return;
    case 'X': {
      long id, op, L, R;
      if (sscanf(s + 1, "%ld %ld %ld %ld", &id, &op, &L, &R) != 4 || id < 0 || id > 255 || op < 0 || op > 7) {
        Serial.println(F("E parse"));
        return;
      }
      sendFrame(calib_pack((uint8_t)id, (uint8_t)op, (int)L, (int)R));
      return;
    }
    case 'G': {
      const long ms = atol(s + 1);
      if (ms < PERIOD_MIN_MS || ms > PERIOD_MAX_MS) { Serial.println(F("E range")); return; }
      periodMs = (uint16_t)ms;
      Serial.print(F("K G ")); Serial.println(periodMs);
      return;
    }
    case 'P':
      Serial.print(F("K P ")); Serial.println(millis());
      return;
    case 'V':
      Serial.print(F("I " FW_VERSION " proto=")); Serial.print(CALIB_PROTO_VERSION);
      Serial.print(F(" pin=")); Serial.print(IR_LED_PIN);
      Serial.print(F(" period_ms=")); Serial.println(periodMs);
      return;
    case '\0':
      return;
    default:
      Serial.println(F("E ?"));
  }
}

void setup() {
  pinMode(IR_LED_PIN, OUTPUT);
  digitalWrite(IR_LED_PIN, LOW);     // MOSFET off: no IR while idle
  Serial.begin(115200);
#if defined(IR_SEND_PIN)
  IrSender.begin();                  // the library fixed the pin (hardware-PWM platforms)
#else
  IrSender.begin(IR_LED_PIN);
#endif
}

void loop() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\n') {
      if (lineDrop) {
        Serial.println(F("E long"));
      } else {
        line[lineLen] = '\0';
        handleLine(line);
      }
      lineLen = 0;
      lineDrop = false;
    } else if (c == LINE_CANCEL) {   // resync: whatever came before must not run
      lineLen = 0;
      lineDrop = false;
    } else if (c != '\r' && !lineDrop) {
      if (lineLen < sizeof(line) - 1) line[lineLen++] = c;
      else lineDrop = true;          // overlong line: drop all of it (its tail must not run as a command)
    }
  }
}
