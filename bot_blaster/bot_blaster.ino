// bot_blaster.ino -- a GRASPion body used as a USB-serial -> IR remote for another bot.
//
// UNTESTED on hardware. Speaks the same serial protocol as calibration/blaster/ir_blaster, so the
// graspion_calib PC bridge drives it unchanged. Mount it upside down above the arena: the bot's IR LED is
// top-emitting and the target's TSOP faces lens-up.
//
// IR path (HARDWARE.md 4.5): D6 -> AVR, which adds the 38 kHz carrier -> IR LED. The carrier is ON while D6
// is LOW (INFERRED), so D6 idles HIGH and IRremote sends the bare envelope (USE_NO_SEND_PWM: mark = LOW).
// An idle LOW would key ~135 mA through the LED on USB.
//
// Serial, 115200 baud, one line per request, one reply per request:
//   F <hex8> / f <hex8>  send a raw 32-bit NEC-timed frame -> "K <hex8> <t_start_ms> <t_end_ms>"
//   G <ms>               minimum start-to-start period, 100..2000 (default 110) -> "K G <ms>"
//   P                    ping -> "K P <millis>"
//   V                    version -> "I bot_blaster/1 proto=2 pin=6 period_ms=.."
// A CAN byte (0x18) drops the partial line (the PC's resync). F does not check the CRC here; the target bot
// does. proto=2 is reported for the PC's version check: keep it equal to CALIB_PROTO_VERSION.
//
// Build: graspion:samd:graspionBody:opt=fastest,usbstack=tinyusb, library IRremote@4.7.1.

#include <Adafruit_TinyUSB.h>
#define USE_NO_SEND_PWM
#define NO_LED_SEND_FEEDBACK_CODE
#include <IRremote.hpp>

#define IR_PIN 6        // IR_TX: Serial1 TX -> AVR -> IR LED; Serial1 is never started

static uint16_t periodMs = 110;
static uint32_t lastStartMs = 0;
static bool sentOnce = false;
static char line[48];
static uint8_t lineLen = 0;
static bool lineDrop = false;

static void sendFrame(uint32_t raw) {
  while (sentOnce && (uint32_t)(millis() - lastStartMs) < periodMs) {}
  const uint32_t t0 = millis();
  lastStartMs = t0;
  sentOnce = true;
  IrSender.sendNECRaw(raw, 0);   // blocks 50-86 ms; ends with a space, so D6 is HIGH again
  char out[40];
  snprintf(out, sizeof(out), "K %08lX %lu %lu", (unsigned long)raw, (unsigned long)t0, (unsigned long)millis());
  Serial.println(out);
}

static void handleLine(char *s) {
  char *end;
  switch (s[0]) {
    case 'F':
    case 'f': {
      const uint32_t raw = strtoul(s + 1, &end, 16);
      while (*end == ' ') end++;
      if (end == s + 1 || *end) { Serial.println(F("E parse")); return; }
      sendFrame(raw);
      return;
    }
    case 'G': {
      const long ms = atol(s + 1);
      if (ms < 100 || ms > 2000) { Serial.println(F("E range")); return; }
      periodMs = (uint16_t)ms;
      Serial.print(F("K G ")); Serial.println(periodMs);
      return;
    }
    case 'P': Serial.print(F("K P ")); Serial.println(millis()); return;
    case 'V': Serial.print(F("I bot_blaster/1 proto=2 pin=6 period_ms=")); Serial.println(periodMs); return;
    case '\0': return;
    default: Serial.println(F("E ?"));
  }
}

void setup() {
  pinMode(BUZ_ENABLE, OUTPUT); digitalWrite(BUZ_ENABLE, LOW);   // motors off, and they stay off
  pinMode(IR_PIN, OUTPUT); digitalWrite(IR_PIN, HIGH);          // IR LED off
  Serial.begin(115200);
  IrSender.begin(IR_PIN);
}

void loop() {
  while (Serial.available() > 0) {
    const char c = (char)Serial.read();
    if (c == '\n') {
      if (lineDrop) Serial.println(F("E long"));
      else { line[lineLen] = '\0'; handleLine(line); }
      lineLen = 0;
      lineDrop = false;
    } else if (c == '\x18') {
      lineLen = 0;
      lineDrop = false;
    } else if (c != '\r' && !lineDrop) {
      if (lineLen < sizeof(line) - 1) line[lineLen++] = c;
      else lineDrop = true;
    }
  }
}
