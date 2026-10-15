# GRASPion Hardware Reference

Hardware reference for the **GRASPion** brainbot: a programmable vibration-driven robot (bristlebot) for active-matter research. It was built at the GRASP lab, University of Liège, with PULS at FAU Erlangen-Nürnberg. This document is written for AI coding agents and humans working in `fruit-fly-graspion`. It aims to let you program and reason about the bot **without re-reading the upstream schematics, datasheets, or paper**.

- **Upstream repo:** https://github.com/GRASP-LAB/GRASPion, read at `main` HEAD `fd11645` (2026-10-13). The repo has **no LICENSE file**; the GitHub API reports `license: null`.
- **Paper:** Novkoski, Mélard, Delens, Wéry, Noirhomme, Pande, Maier, Smith & Vandewalle, *GRASPion: an Open-Source, Programmable Brainbot for Active Matter Research*. Rev. Sci. Instrum. **97**, 014704 (2026), doi:10.1063/5.0303152. Preprint: arXiv:2509.07437 (v1 only). Revised open postprint: https://orbi.uliege.be/handle/2268/343620
- **Project site:** https://graspion.be. It lists the price as 120 EUR and offers a contact form for ordering.
- **Compiled:** 2026-10-14. Sources were the upstream repo files, the four schematic PDFs (rendered and read), manufacturer datasheets, the paper and related papers, followed by an adversarial cross-check pass.
- **Updated:** 2026-10-15, with toolchain facts verified by compiling while building [calibration/](calibration/README.md) (§9.1, §11), and the UF2 bootloader's effect on the motor enable, read from the uf2-samdx1 sources (§4.4, §9.2). Nothing in this update has run on a bot.

**Confidence labels used below.**

| Label | Meaning |
|---|---|
| *(no label)* | Read directly from a primary source: schematic, repo file, datasheet, or paper. |
| **INFERRED** | Reasoned from sources but not stated anywhere. Treat it as a strong hypothesis. |
| **UNKNOWN** | No source exists. Measure it on hardware or ask the authors. |
| **UNTESTED** | Code written for this doc that has not been compiled or run on a bot. |

---

## 0. TL;DR for agents

| Item | Value |
|---|---|
| Size and mass | 60 × 30 × 15 mm overall, **17 g**. The printed body alone is 60 × 30 × 9 mm, 6 g. |
| Main MCU (you program this) | **ATSAMD21E18A** (Cortex-M0+, 48 MHz, 256 KB flash, 32 KB RAM). It is an Adafruit QT Py M0 clone with a UF2 bootloader and **no crystal**. |
| Coprocessor ("BIOS") | **AVR32DD20**. It handles the power button, power rails, charger, IR carrier and orange LED. Its firmware is **not public**. It is an I2C target at **0x36**. |
| Locomotion | 2× Vybronics **VZ3TH8B171700L** ERM vibration motors (2.7 V rated), each driven by a TI **DRV2603** in ERM mode. The robot is differential-drive-like: one motor makes it rotate about a front leg, both make it go straight. |
| Motor API | D2 = right, D3 = left, D12 = shared enable. `analogWrite` value **128 = stop** (127 in the examples), >128 = one direction, <128 = the other. |
| Sensors (body) | 3-axis magnetometer **MLX90393** (I2C 0x10), ambient-light phototransistor **TEMT6202** (A0), IR receiver **TSOP75438** 38 kHz (D7) |
| Front module v1 | **APDS-9960** proximity/RGB-colour/gesture sensor (I2C 0x39). INT is on D8 and the IR-LED supply enable on D10. |
| Actuators / outputs | 2 NeoPixel RGB LEDs on D11 (index 0 = bottom/rear, 1 = top), a 940 nm IR LED (Serial1 TX on D6, through the AVR), an orange charge LED (AVR-controlled) |
| Storage | W25Q16JV, 16 Mbit = **2 MB** SPI NOR flash on SPI1 |
| Power | Li-ion coin cell (the v1 schematic names **LIR2477**, typically 160–200 mAh). STNS01 charger/LDO, USB-C sink, runtime **> 90 min** (paper), 3.1 V logic rail. |
| Speed | Up to **16 cm/s** (paper Fig. 1a) or about 15 cm/s (README). Typical calibrated range is 4.7–11.5 cm/s. |
| Toolchain | Arduino. FQBN **`graspion:samd:graspionBody:opt=fastest,usbstack=tinyusb`**, but `opt=small` for any sketch using an Adafruit BusIO-based library (MLX90393, APDS9960, so all three upstream examples; item 9, §9.1). Boards Manager URL: `https://raw.githubusercontent.com/GRASP-LAB/GRASPion/main/package_graspion_index.json` |
| Head board 2026 (in development) | STM32U575 + u-blox ANNA-B112 (nRF52832, BLE) + I2S MEMS mic. It plugs into the same 10-pin connector. There is no STM32 firmware yet. |

**Top gotchas.** Section 11 has the full list.
1. Use the **GRASPion Body** board, not "Adafruit QT Py M0". The QT Py variant drives D12 (motor enable) HIGH at boot. Start every sketch with `pinMode(BUZ_ENABLE, OUTPUT); digitalWrite(BUZ_ENABLE, LOW);`. A QT Py UF2 bootloader ≥ 3.11, if a bot carries one, also holds D12 HIGH while it runs (uploads, double-tap recovery): lift the bot (§9.2).
2. Default `analogWrite` PWM is **732 Hz**, but the DRV2603 is specified for a 10–250 kHz PWM input. Section 4.4 has a fix.
3. Press the **power button** before uploading: the SAMD is only powered when the AVR switches the 3.1 V rail on.
4. When the bot dies from over-discharge or overcurrent, **only USB power revives it**. The button does not.
5. `IrReceiver.begin(7)` kills Serial1 RX. A continuous 4800-baud UART over IR violates the TSOP's burst rules.
6. MLX90393 sits at **0x10**, but the Adafruit library defaults to 0x0C. Always pass the address.
7. With TinyUSB, `#include <Adafruit_TinyUSB.h>` in every sketch. `Serial` links only if that header reaches the build, from the sketch or from a library header (`Adafruit_NeoPixel.h` includes it); a sketch with neither fails to link. `loop()` must return or call `delay()`/`yield()`, or uploads break. Never call `Serial.end()`.
8. **None of the three upstream examples builds with the recommended FQBN.** `remoteControl.ino`, `sensing.ino` and `colors.ino` all include and use `Adafruit_MLX90393`, which pulls in BusIO (item 9). `colors.ino` also has its own compile errors (§11 item 12).
9. Libraries built on **Adafruit BusIO** **fail to build at `opt=fastest`** (and at `opt=faster` and `opt=dragons`): MLX90393 and BusIO alone are verified; APDS9960 includes the same BusIO header (not compiled itself). Use `opt=small` or raw `Wire` (§9.1).

---

## 1. Hardware revisions and source files

| Board | Upstream file | EAGLE name / date | Status |
|---|---|---|---|
| Body **v1** | `circuit/circuitSchema.pdf` | `blatte24V1.sch`, 2025-02-11 | Original |
| Body **V1.1** | (none) | Silkscreen "Blatte24V1.1" | The board in the paper's photos (Fig. 3). No schematic is published. |
| Body **V1.3** | `circuit/graspionBodyV1.3.pdf` | `blatte24V1.3`, 2026-10-06 | **Current. This doc describes V1.3 unless stated otherwise.** |
| Front module v1 (APDS-9960) | `circuit/headSchema.pdf` | `blatteHeadV1.sch`, 2025-02-11 | Shipped "front module" |
| Head 2026 (STM32 + BLE + mic) | `circuit/GraspionHeadDev2026.pdf` | `blatteHead2026.sch`, 2026-10-07 | In development |

- The repo has **no BOM, `.sch`/`.brd` source, or Gerber files**. The PDFs are the only electrical source.
- "blatte" is the internal project name. `BLATTE_TX/RX` net names refer to the body.
- Changes from v1 to V1.3. All SAMD and AVR pin assignments, rails and connector pinouts are **unchanged**.
  - Magnetometer: MLX90393SLQ-ABA-011 (UTDFN-8) became **MLX90393ELW-ABA-012** (QFN-16). The I2C address is 0x10 on both.
  - Motor footprints: 7 paralleled footprints became 4 (M1, M2 left; M3, M4 right). **The right pair is now wired with reversed polarity.**
  - Battery: the "BT1 = LIR2477" symbol became two contacts B1 (−) and B2 (+), with no cell value printed.
  - N-FET: DMN2053UVT became **DMN3135LVT-7**. The P-FET is DMP3164LVT on both.
  - IR parts: LED VSMB1940ITX01 became VSMB1940X01, and the receiver became TSOP75438**TT**.
  - Flash CS pull-up: 10k became 5k1.
  - Head connector: X3 plus a stacked X3' became a single X3.

---

## 2. System overview

```
                        ┌─────────────────────── BODY BOARD (V1.3) ───────────────────────┐
 USB-C ──VBUS──► STNS01 charger/LDO ──SYS──┬─► AVR32DD20 "BIOS" (VDD)                     │
   │ D+/D-            │  │ LDO 3.1V         ├─► IR LED (via 27Ω, N-FET gated by AVR TXMOD) │
   │                  │  └──► P-FET(ENLDO)──► +3V1 ─► SAMD21, MLX90393, TSOP, flash,      │
   │                  │                              light sensor, I2C pull-ups, HEAD p3  │
   │             Li-ion coin cell (B1/B2)   └─► P-FET(ENSYS)──► V+ ─► 2× DRV2603 (motors),│
   │                                                             2× NeoPixel, HEAD p1     │
   └──────────────► SAMD21E18A (Arduino) ◄──I2C(0x36)──► AVR ◄── SW1 power button         │
                     │ D2 ─► right DRV2603, D3 ─► left DRV2603, D12 ─► both EN ─► ERMs      │
                     │ D6 (Serial1 TX) ─► AVR ─► 38 kHz IR LED     D7 ◄─ TSOP75438        │
                     │ D11 ─► NeoPixel bottom ─► NeoPixel top ─► NEOPIXO (HEAD p2)         │
                     │ D4/D5 I2C ─► MLX90393 (0x10), AVR (0x36), HEAD                      │
                     │ A0 ◄─ TEMT6202 light sensor     SPI1 ─► W25Q16 2 MB flash           │
                     └─ SPI (D8/D9/D10) + A1 + I2C + power ─► 10-pin HEAD connector X3 ─────┘
                                                                     │
                     Front module v1: APDS-9960 (0x39)  ─or─  Head 2026: STM32U575 + nRF52 BLE + mic
```

On a body plus a 2026 head the robot has **four MCUs**: SAMD21 (user code), AVR32DD20 (power/IR), STM32U575 and nRF52832. The nRF52 README says "three" because it omits the AVR.

---

## 3. Body board V1.3: power

### 3.1 Rails

| Rail | Source | Voltage | Loads |
|---|---|---|---|
| **VBUS** | USB-C X1 | Recommended 4.55–5.4 V. The STNS01 treats VIN as valid for UVLO < VIN < OVP: UVLO 4.18 V typ falling (300 mV hysteresis), OVP 5.9 V typ (5.6–6.2 V). | STNS01 IN only. The board is a sink: CC1 and CC2 each have 5k1 to GND. D+/D− go **directly** to the SAMD with no series resistors or ESD protection. |
| **SYS** | STNS01 SYS (power path) | = VIN when USB is valid, otherwise ≈ VBAT (2.8–4.2 V). Always on while the STNS01 is on. | AVR32DD20 VDD, IR LED anode, and the source of the V+ switch |
| **LDO** | STNS01 LDO | 3.1 V (3.007–3.193 V), rated "up to 100 mA" | Only the +3V1 switch |
| **+3V1** | LDO → DMP3164LVT P-FET half A, gate **ENLDO** (AVR PA2) | 3.1 V | SAMD21 (VDDIN/VDDANA), MLX90393, TSOP75438, W25Q16, TEMT6202 collector, I2C pull-ups (5k1), AVR VDDIO2, HEAD X3 pin 3, SWD X2 pin 1. **The whole 2026 head runs from this rail.** |
| **V+** | SYS → DMP3164LVT half B, gate **ENSYS** (AVR PA1) | ≈ VBAT on battery, ≈ 5 V on USB | Both DRV2603 (motors), both NeoPixels, HEAD X3 pin 1, SWD X2 pin 7 |

- The P-FETs turn on when the gate is **LOW** (INFERRED from the P-FET topology). There are no gate pull resistors, so while the AVR is in reset the rail state is undefined (INFERRED).
- All ADC readings are ratiometric to +3V1 (≈ 3.1 V ± 3 %).

### 3.2 Battery and charger (STNS01, datasheet DocID024654)

| Parameter | Value |
|---|---|
| Cell | Li-ion coin cell. The v1 schematic names **LIR2477**: Ø24.5 × 7.7 mm, 3.6–3.7 V nominal, **160–200 mAh depending on vendor**. The V1.3 schematic shows only contacts B1 (−) and B2 (+); on the V1.1 board photo, B1 is a centre spring and B2 an edge clip. The paper and site give no capacity. |
| Charge | CC/CV to **4.2 V ± 1 %**. R_ISET = 1 kΩ sets fast charge **200 mA** (180–220). Pre-charge 40 mA below 3.0 V; termination 20 mA. Timers: pre-charge 30 min, fast charge 10 h. |
| Charge time | **UNKNOWN**. Roughly 1–1.5 h by arithmetic (INFERRED). 200 mA is about 1 C or more for this cell. |
| Runtime | **> 90 min** (paper) or "90 minutes" (site), at an unspecified duty |
| Over-discharge cut | VBAT < 2.8 V (2.75–2.85) for 400 ms turns the device **off**. Release needs USB **and** VBAT > 3.0 V. |
| Discharge overcurrent | **400–650 mA for 14 ms** turns the device off until USB is connected |
| Overcharge | 4.275 V |
| Temperature (NTC) | R5 = NTCS0603E3153HMT (15 kΩ, B = 3600 K) on the **PCB**. ST designed the thresholds for 10 kΩ / β 3370, so the effective charge window is about **11 °C to 55 °C** (INFERRED). Charging pauses in a cold room. |
| SD (ship mode) | Driven by AVR PD7. Only works when VIN is invalid. Exit only by plugging in USB. |
| CEN / /CHG / BATMS | AVR PD5 / PD6 / PD4. /CHG is open-drain, low while charging, and toggles at 1 Hz on faults. |
| SYS rating | The datasheet says "up to 100 mA" for external loads, but V+ loads exceed this. The hard limit on battery is the 400 mA overcurrent trip. |

**Consequences for experiments:**
- Every protection shutdown also removes power from the AVR, so **the button cannot restart the bot. Plug it into USB.** The README also notes that a bot reassembled after the battery was disconnected needs USB power to start.
- **Worst-case current** (INFERRED): two motors at 77–110 mA each, NeoPixels at full white (about 60 mA each for 5050 WS2812-class parts), IR LED 60–135 mA, APDS LED pulses up to 100 mA × boost, plus logic. This can exceed the 400 mA minimum trip. **Limit NeoPixel brightness while the motors run.**
- Stop experiments at about 3.3–3.4 V on the battery for consistent motor speed, which saturates below that (INFERRED). NeoPixels are already below the WS2812B 3.5 V VDD minimum once the battery drops below about 3.5 V, so colours may shift earlier.
- Do not hot-plug head modules while the bot is on. The STNS01 shuts down without deglitch if LDO < 0.7 V or SYS < VBAT − 0.8 V (INFERRED risk).

### 3.3 Reading battery and USB state from the SAMD

The AVR answers a plain 5-byte I2C read at **0x36**, with no register pointer:

| Bytes | Meaning |
|---|---|
| 0–1 | `mVusb`, big-endian. **INFERRED** to be the SYS rail (the AVR's own VDD), since no VBUS net reaches the AVR. |
| 2–3 | `mVbat`, big-endian. **INFERRED** to be measured on BATMS via AVR PD4/AIN4, the only battery-voltage net reaching the AVR. |
| 4 | Unused by the examples. Meaning **UNKNOWN**. |

- USB or a charger is present when `mVusb >= 4500`, as in the examples.
- A USB **data host** is present when `TinyUSBDevice.mounted()` returns true; a charger alone never mounts.
- Whether the AVR accepts writes (power-off, ship mode, LED control) is **UNKNOWN**.

```cpp
// UNTESTED. Uses uint8_t: the examples' char buffer only works because char is unsigned on ARM.
bool readPower(uint16_t &mVsys, uint16_t &mVbat) {
  uint8_t b[5];
  if (Wire.requestFrom(0x36, 5) != 5) return false;
  for (int i = 0; i < 5; i++) b[i] = Wire.read();
  mVsys = (b[0] << 8) | b[1];
  mVbat = (b[2] << 8) | b[3];
  return true;
}
```

### 3.4 Power button and boot

- **SW1** (C&K RKB2SJK250, 250 gf) connects **AVR PF6/RESET** to GND. It is not wired to the SAMD or the charger.
- "Press once to power on / make the USB port appear" (README) is **AVR firmware behaviour**. The AVR then drives ENLDO and ENSYS to enable +3V1 and V+ (INFERRED). Long-press and power-off behaviour is **UNKNOWN**.
- The AVR can hold the SAMD in reset through Q1A (DMN3135LVT N-FET, gate on AVR PA7, drain on SAMD /RESET).
- The SAMD has **no reset button**. Its /RESET is only on the SWD header X2 pin 10.
- The orange **LED1** (AVR PA0 through 1 kΩ) is the "charging status LED" in the paper's photo. It is controlled by AVR firmware, not directly by the charger.

---

## 4. Body board V1.3: main MCU and peripherals

### 4.1 SAMD21 pin map

The variant is `arduino/samd/variants/graspionBody/`. Its pin table is **byte-identical to the Adafruit `qtpy_m0` table**; only the macros and `initVariant()` differ.

| Arduino | Port | Macro(s) | Schematic net → function | PWM (analogWrite) | ADC | EXTINT |
|---|---|---|---|---|---|---|
| D0 / A0 | PA02 | `A0`, `LIGHT_SENSOR`, `DAC0`, `SS` | LIGHTSNS ← TEMT6202 light sensor. **Also the DAC pin: never `analogWrite(A0, …)`.** `SS` (default SPI chip-select) also points here, so never use the default SS. | — | AIN0 | 2 |
| D1 / A1 | PA03 | `A1` | A1 → HEAD X3 pin 7 | — | AIN1 | 3 |
| D2 / A2 | PA04 | `BUZ_RIGHT` | BUZRPWM → right DRV2603 PWM | TCC0 CC0 | AIN4 | 4 |
| D3 / A3 | PA05 | `BUZ_LEFT` | BUZLPWM → left DRV2603 PWM | TCC0 CC1 | AIN5 | 5 |
| D4 | PA16 | `SDA` | I2C SDA (SERCOM1), HEAD pin 10, SWD pin 8 | TCC2 CC0 | — | 0 |
| D5 | PA17 | `SCL` | I2C SCL (SERCOM1), HEAD pin 9, SWD pin 6 | TCC2 CC1 | — | 1 |
| D6 / A6 | PA06 | `IR_TX`, Serial1 TX | TXSAMD → **AVR PC1**. The AVR then modulates the IR LED. | TCC1 CC0 | AIN6 | 6 |
| D7 / A7 | PA07 | `IR_RX`, Serial1 RX | RXSAMD ← **TSOP75438 OUT** (active-low, direct) | TCC1 CC1 | AIN7 | 7 |
| D8 / A8 | PA11 | `SCK` | SPI SCK (SERCOM2) → HEAD pin 8. **APDS INT** on front module v1. | TCC0 CC3 | AIN19 | 11 |
| D9 / A9 | PA09 | `MISO` | SPI MISO (SERCOM2) → HEAD pin 4 | TCC1 (shares CC1 with D7) | AIN17 | 9 |
| D10 / A10 | PA10 | `MOSI` | SPI MOSI (SERCOM2) → HEAD pin 6. **APDS IR-LED enable** on front module v1. | TCC0 CC2 | AIN18 | 10 |
| D11 | PA18 | `PIN_NEOPIXEL`, `NUM_NEOPIXELS`=2, `NEOPIXEL_BOTTOM`=0, `NEOPIXEL_TOP`=1 | NEOPIX → LED3 (bottom) → LED4 (top) → NEOPIXO → HEAD pin 2 | — | — | 2 |
| D12 | PA15 | `BUZ_ENABLE` | BUZEN → EN of **both** DRV2603 | — | — | 15 |
| D13 | PA27 | `LED_BUILTIN` | **Not connected.** Placeholder kept for core compatibility. | — | — | — |
| D14 | PA23 | `SCK1` | SCK2 → flash (SPI1, SERCOM3) | (aliases CC1 = D3) | — | 7 |
| D15 | PA19 | `MISO1` | MISO2 ← flash | TC3 | — | 3 |
| D16 | PA22 | `MOSI1` | MOSI2 → flash | (aliases CC0 = D2) | — | 6 |
| D17 | PA08 | `SS1` | FLASH_CS (5k1 pull-up). Never use as PWM. | (TCC1, aliases CC0 = D6) | AIN16 | NMI |
| 18 | PA28 | `PIN_USB_HOST_ENABLE` | Not connected | — | — | — |
| 19 / 20 | PA24 / PA25 | USB D− / D+ | USB-C | — | — | — |
| — | PA30 / PA31 | — | SWCLK / SWDIO → SWD X2 pins 4 / 2 | — | — | — |
| — | PA00, PA01, PA14 | — | Not connected (no 32 kHz crystal) | — | — | — |

- **Net-name trap:** schematic nets SCK/MISO/MOSI are Arduino `SPI` (SERCOM2, which goes to the HEAD). SCK2/MISO2/MOSI2 are Arduino `SPI1` (SERCOM3, which goes to the flash).
- `variant.h` also defines: `MLX90393_I2C_ADDRESS 0x10`, `ADC_RESOLUTION 12`, `PINS_COUNT 21`, `EXTERNAL_FLASH_DEVICES GD25Q16C` (wrong: the board has a W25Q16JV), and I2S pins inherited from the QT Py, which are **unusable** on this board.
- Build flags are `-DCRYSTALLESS -DADAFRUIT_QTPY_M0 -DARDUINO_GRASPION_BODY`. Libraries therefore treat the body as a QT Py.
- **`analogRead` returns 10-bit values by default.** Call `analogReadResolution(12)` for 12-bit. Full scale is VDDANA = +3V1.

### 4.2 AVR32DD20 "BIOS" coprocessor

| AVR pin | Net | Role |
|---|---|---|
| PA0 | LED1 orange through 1 kΩ | Charge/status LED |
| PA1 | ENSYS | V+ P-FET gate |
| PA2 | ENLDO | +3V1 P-FET gate |
| PA3 | TXMOD | IR LED N-FET gate (TCB1 WO or CCL LUT0 OUT capable) |
| PA4–PA6 | — | Not connected |
| PA7 | Q1A gate | Pulls SAMD /RESET low |
| PC1 | TXSAMD | Receives SAMD Serial1 TX (USART1 RxD) |
| PC2 / PC3 | SDA / SCL | I2C client at 0x36 (MVIO domain at +3V1) |
| PD4 / PD5 / PD6 / PD7 | BATMS / CEN / /CHG / SD | STNS01 control |
| PF6/RESET | SW1 to GND | Power button |
| PF7/UPDI | UPDI → SWD X2 pin 9 | AVR programming |
| VDD / VDDIO2 | SYS / +3V1 | — |

- The AVR firmware is **not published in any GRASP-LAB repo**. Not documented: the register map, carrier polarity, button logic, and I2C speed limit.
- Datasheet caveat (INFERRED risk): PORTD runs at SYS level, up to 5.4 V on USB, while the STNS01 CEN and SD absolute maximum is 4.0 V.

### 4.3 I2C bus

The SAMD Wire bus is SERCOM1 on D4/D5 at 100 kHz by default. It has **5k1 pull-ups to +3V1 on the body only**. The bus is also exported on HEAD X3 pins 9/10 and SWD X2 pins 6/8.

| Address | Device | Where | Notes |
|---|---|---|---|
| **0x10** | MLX90393 magnetometer | Body (v1 and V1.3) | ELW-ABA-012 with A0 = A1 = GND gives 0x10; SLQ-ABA-011 is fixed at 0x10. The **Adafruit library default is 0x0C**: always call `begin_I2C(MLX90393_I2C_ADDRESS)`. |
| **0x36** | AVR32DD20 BIOS | Body | 5-byte power read (§3.3). The address is set in firmware. Do not add another device at 0x36; MAX1704x fuel gauges, for example, use it. |
| **0x39** | APDS-9960 | Front module v1 | Fixed address, so only one APDS is possible |
| ? | STM32U575 I2C1 (PB6/PB7), nRF52 TWI (P0.16/P0.18) | Head 2026 | Role and address **UNKNOWN** (no firmware). If either is a master, the bus is multi-master. |

### 4.4 Motors (DRV2603 × 2 → VZ3TH8B171700L × 2)

**Wiring:**
- Each side has its own DRV2603: left = IC4 (PWM from D3), right = IC5 (PWM from D2).
- **EN of both drivers is D12.**
- **LRA/ERM is tied to GND, so both drivers run in ERM mode.**
- Driver VDD comes from V+.

**Motor contacts:**
- Each driver has **two paralleled contact-pad pairs**: M1‖M2 (left) and M3‖M4 (right).
- The bot has **two motors**, held by spring contacts in two body compartments. Each motor rests on one pad pair; the duplicates are alternative positions (INFERRED from the paper photo showing 4 pad pairs).
- **On V1.3 the right pair is wired with reversed polarity** relative to the left. v1 had all motors "+" on OUT+. The purpose is **UNKNOWN**. The paper says CW and CCW drive have the same physical effect, so only |drive| matters for motion.

**DRV2603 ERM transfer function** (TI SLOS754C):
- VOUT = VFS × (duty/50 % − 1). So 100 % = full forward, **50 % = off**, 0 % = full reverse.
- VFS ≈ 3.6 V open load, about 3.3 V into the 26 Ω motor. It is supply-compensated, so it does not change between USB and battery as long as V+ is high enough.
- The PWM input frequency must be **10–250 kHz**.
- Other figures: VDD 2.5–5.2 V; idle current with EN high is 1.7–2.5 mA; start-up 1.3 ms after EN rises; internal output PWM about 20 kHz.

**Motor** (Vybronics VZ3TH8B171700L, cylindrical ERM with spring contacts):

| Parameter | Value |
|---|---|
| Voltage | Rated 2.7 V, operating 2.3–3.3 V; start is only guaranteed at ≥ 2.3 V |
| Current | ≤ 100 mA rated (77 mA typ), 110 mA starting |
| Resistance | 26 Ω |
| Speed | 13 000 ± 2500 rpm (≈ 217 Hz) |
| Response | 50 % in 50 ms, 90 % in 100 ms |
| Mass | 0.55 g |

**Example motor API.** This is a condensed excerpt. The function bodies are identical in all three `examples/*.ino`, which use `uint8_t` parameters and do the init inside `setup()`.

```cpp
#define BUZ_EN_PIN 12
#define BUZR_PIN 2   // right motor
#define BUZL_PIN 3   // left motor
void motorsInit(){ pinMode(BUZ_EN_PIN, OUTPUT); digitalWrite(BUZ_EN_PIN, LOW); analogWriteResolution(8); } // call first in setup()
void buzCw (uint8_t val, uint8_t pin){ digitalWrite(BUZ_EN_PIN,HIGH); analogWrite(pin,127+min(abs(val),128)); } // val 0..127
void buzCcw(uint8_t val, uint8_t pin){ digitalWrite(BUZ_EN_PIN,HIGH); analogWrite(pin,127-min(abs(val),127)); }
void buzStop(uint8_t pin){ if(pin==0){analogWrite(BUZR_PIN,127);analogWrite(BUZL_PIN,127);digitalWrite(BUZ_EN_PIN,LOW);} else analogWrite(pin,127); }
```

`pinMode(BUZ_EN_PIN, OUTPUT)` is essential, and `pinMode(OUTPUT)` then `LOW` must be the very first statement of `setup()`. On SAMD a `digitalWrite` to a non-output pin only toggles the internal pull-up, so EN would float. D12 reaches `setup()` in one of two states, depending on the UF2 bootloader (§9.2):
- a floating input: the bootloader bundled with the core (v1.23.1-…-gb149075) powers its NeoPixel from PA19;
- an output driven HIGH, with D2/D3 floating, so the motors may run at full drive: a QT Py UF2 bootloader ≥ 3.11 powers its NeoPixel from PA15 = D12.

**Kinematics** (paper):
- Left motor only → the bot **turns right**, rotating about a front leg. Right motor only → it turns left.
- Both motors → straight.
- A single-motor turn drifts about 5 mm over many rotation periods.

**Facts about this API** (Adafruit ArduinoCore-samd 1.7.17, `wiring_analog.c`):
- The default `analogWrite` PWM is **48 MHz / 65536 = 732.42 Hz** (TCC0, PER = 0xFFFF, DIV1, value << 8). That is about **14× below the DRV2603's 10 kHz minimum**. The real effect on the motor is **UNKNOWN**: expect ripple and noise, and measure it with a scope.
- Value 127 gives 49.6 % duty, about −0.8 % drive. **128 is the true zero.** Value 255 gives 99.6 %, so 100 % is unreachable.
- The enable line is shared, so during a single-motor turn the "stopped" motor still has its H-bridge switching at about 0 V. Set **EN LOW** when both motors stop.
- At full drive (254) the output is about 3.3 V, the motor's **maximum** operating voltage. The rated 2.7 V is about 90 % duty: analogWrite ≈ 231, or buzCw ≈ 104 (INFERRED).
- Below about V+ = 3.4 V, high duty saturates and speed drifts with battery level. Use ≤ 85–90 % for consistent speed (INFERRED).
- The minimum reliable power is about 55–60 on the buzCw scale. The examples floor at 55 and the lab's lookup tables start at 60.

**In-spec PWM.** UNTESTED. 40 kHz on TCC0.
- Call it with **EN LOW**.
- Afterwards, write the duty only with `motorSet` and **never call `analogWrite` on D2/D3 again**.
- It also changes the PWM period of D8/D10.
- `s = ±1` gives about 3.35 V, slightly above the motor's 3.3 V maximum. The rated 2.7 V is about |s| = 0.8.

The snippet resets COUNT and overwrites CCB because `analogWrite` leaves a buffered compare value pending (STATUS.CCBV), which would otherwise load a 100 % duty at the next update.

```cpp
void motorsPwmInit(uint16_t per = 1199) {                 // 48 MHz/(per+1) = 40 kHz
  analogWrite(BUZ_RIGHT, 128); analogWrite(BUZ_LEFT, 128); // let the core set pin mux + GCLK0->TCC0 + NPWM
  TCC0->CTRLA.bit.ENABLE = 0; while (TCC0->SYNCBUSY.bit.ENABLE);
  TCC0->COUNT.reg = 0;        while (TCC0->SYNCBUSY.bit.COUNT);   // COUNT may exceed the new PER
  TCC0->PER.reg = per;        while (TCC0->SYNCBUSY.bit.PER);
  TCC0->CC[0].reg  = TCC0->CC[1].reg  = (per + 1) / 2;            // 50 % = stop
  TCC0->CCB[0].reg = TCC0->CCB[1].reg = (per + 1) / 2;            // overwrite values analogWrite left pending
  while (TCC0->SYNCBUSY.reg);
  TCC0->CTRLA.bit.ENABLE = 1; while (TCC0->SYNCBUSY.bit.ENABLE);
}
void motorSet(uint8_t ch /*0 = RIGHT (D2), 1 = LEFT (D3)*/, float s /*-1..1*/, uint16_t per = 1199) {
  TCC0->CCB[ch].reg = (uint32_t)((s + 1.f) * 0.5f * (per + 1) + 0.5f);
  while (TCC0->SYNCBUSY.reg & (TCC_SYNCBUSY_CCB0 << ch));
}
```

**Per-bot calibration is required.** Bots are mechanically asymmetric. In the lab's digital-twin work (github.com/GRASP-LAB/brainbot-digital-twin), left/right powers of 60–120 (INFERRED to be on the buzCw 0–127 scale; the body-side parser is not published) gave a centre-of-mass speed of **4.7–11.5 cm/s** and |ω| up to about 3.5 rad/s. One bot drove straight at (L, R) = (60, 80).

### 4.5 IR link

| Direction | Path |
|---|---|
| **TX** | SAMD `Serial1` TX (D6/PA06) → AVR PC1 → AVR adds the **38 kHz carrier** → PA3 TXMOD → N-FET → **VSMB1940X01** 940 nm LED (±60°, top-emitting) with 27 Ω from **SYS**. LED on-current is about 61 mA at 3.0 V, 106 mA at 4.2 V and 135 mA on USB. Above about 3.9 V on SYS this exceeds the LED's **100 mA DC absolute maximum**; it is within rating only because the carrier pulses it (IFM 200 mA at 50 % duty, tp ≤ 100 µs). A stuck-on TXMOD would overstress the LED. The AVR modulation is stated in the authors' unmerged `readme-graspion-body-proposal` branch. |
| **RX** | **TSOP75438TT** (38 kHz, AGC4, active-low, "TT" = top-view/lens-up) → D7/PA07 **directly**. The AVR is **not** in the RX path, which contradicts the paper's text. |

- **Polarity** (INFERRED): the carrier is ON while D6 is LOW, so the TSOP output reproduces the UART level. **Keep D6 HIGH when idle.** `Serial1.begin()` already does this; a floating line could key about 100 mA of IR continuously.
- **Remote control:** the examples decode **NEC** with IRremote (`IrReceiver.begin(7)`). The comments name a Grundig 19935 universal remote with TV code 1359. They check only `decodedIRData.command` and ignore the address, so any NEC remote drives every bot.
- **`IrReceiver.begin(7)` switches PA07 to GPIO, so Serial1 RX is dead while IRremote runs.** Serial1 TX still works.
- **UART over IR at 4800 baud** (`Serial1.begin(4800)` in the examples) **violates the TSOP75438 AGC4 limits**, so continuous data will be lost. The limits are:
  - minimum burst 10 cycles (a 4800-baud bit is only 7.9 cycles);
  - bursts over 35 cycles need a gap of more than 15× the burst length;
  - at most 1700 bursts per second.
- No example implements bot-to-bot IR messaging. The IR TX traffic in the examples is:
  - the sketch filename at boot (`Serial1.println(__FILE__)`, all three examples);
  - `colors.ino`'s coupling value every 5 s;
  - the mVusb/mVbat values printed by the 0x13 handlers.
- A plausible but UNTESTED alternative is to send **NEC frames** with IRremote, without starting Serial1; the AVR supplies the carrier:
  ```cpp
  #define USE_NO_SEND_PWM     // before #include <IRremote.hpp>
  pinMode(IR_TX, OUTPUT); digitalWrite(IR_TX, HIGH);  // idle level; IrSender.begin() only stores the pin
  IrSender.begin(IR_TX);
  ```
  With `USE_NO_SEND_PWM`, marks are LOW and spaces HIGH, which matches the inferred polarity.
- Each bot will probably hear its own transmissions through reflections (INFERRED; check on hardware). **Range and data rate are UNKNOWN.**
- The receiver faces lens-up and the LED is top-emitting, so bot-to-bot links probably rely on wide angles or reflections (INFERRED). Check on hardware.

### 4.6 Sensors on the body

**Ambient light**, TEMT6202FX01 on A0:
- An NPN phototransistor with an eye-like response (peak 550 nm), on a 5.1 kΩ emitter load. V ≈ I_photo × 5.1 kΩ.
- Typical 23 µA at 100 lx, i.e. about 1.2 mV/lx, or about 2.6 lx per 10-bit count.
- It **saturates at about 2–4 klx**, so bright lamps clip it.
- It reads about 0 in the dark. **Do not use it alone as a random seed** (§11).

**Magnetometer**, MLX90393 (0x10):
- 3-axis Hall, 16-bit, 50 mT range, 0.161–3.22 µT/LSB (XY), I2C ≤ 400 kHz. INT and TRIG are **not connected**, so poll it.
- The Adafruit library initialises at gain 1x, 16-bit, OSR_3, FILTER_7. With those settings `readData()` **blocks for about 210 ms**. For control loops, lower the filter/OSR or use `startSingleMeasurement()` and `readMeasurement()`.
- **The Adafruit library does not build at `opt=fastest`** (nor `opt=faster` or `opt=dragons`; §9.1). Build with `opt=small`, or drive the sensor over `Wire` directly. `calibration/firmware/calib_bot/calib_bot.ino` does the latter, with non-blocking conversions (compiled; UNTESTED on a bot).
- The ERM magnets and motor currents disturb it. Sample with the motors stopped (EN LOW plus about 100 ms) or calibrate under the operating state (INFERRED).
- The sensor's axis orientation relative to the body is **UNKNOWN**. Find it by rotating the bot flat through 360° and flipping it.

**Front-module sensors:** see §5.

### 4.7 RGB LEDs

- 2× NeoPixel, GRB, 800 kHz, on D11. No part number is given; the photo suggests a 5050 WS2812B-class part on top and a small side-emitting part at the rear.
  - Index **0 = `NEOPIXEL_BOTTOM`**: PCB bottom side, rear tip, faces backward.
  - Index **1 = `NEOPIXEL_TOP`**: top side, front-centre.
  - The chain continues on HEAD pin 2 (NEOPIXO), so index 2 and up belong to a module.
- They are powered from **V+**, but the data line is 3.1 V logic (INFERRED margins):
  - On USB (V+ ≈ 5 V), WS2812B VIH = 3.5 V, so the data is out of spec.
  - On battery below about 3.5 V, VDD is below the WS2812B minimum, so colours can shift.
- The two LEDs likely differ in colour balance: commented code scales G×1.365 and B/1.05 on the bottom LED only.
- `colors.ino` and `sensing.ino` use **Adafruit_NeoPixel_ZeroDMA on SERCOM3**. That conflicts with SPI1 and the flash. `remoteControl.ino` uses plain Adafruit_NeoPixel, which disables interrupts for about 60 µs per `show()`.

### 4.8 Flash

- **W25Q16JVSNIQ**: 2 MB, JEDEC ID EF 40 15, SOIC-8, on SPI1 (D14–D17).
- `Adafruit SPIFlash` detects it from its built-in device list, even though `variant.h` declares GD25Q16C.
- It cannot be used at the same time as ZeroDMA NeoPixels on SERCOM3.
- Send power-down (B9h) for low-current sleep, otherwise it draws 10–50 µA standby.
- The SAMD21 has **no EEPROM**. Store per-bot calibration in this flash or in internal flash (FlashStorage-type libraries).

### 4.9 Connectors

**X3 "HEAD"**: 2×5 right-angle header at the front. Front modules plug in here.

| Pin | Net | Pin | Net |
|---|---|---|---|
| 1 | **V+** (battery / ≈ 5 V USB, AVR-switched) | 2 | NEOPIXO (NeoPixel chain out) |
| 3 | **+3V1** (shares the ≤ 100 mA LDO with the whole body) | 4 | MISO = D9 / PA09 |
| 5 | GND | 6 | MOSI = D10 / PA10 |
| 7 | A1 = D1 / PA03 (ADC, EXTINT, no PWM) | 8 | SCK = D8 / PA11 |
| 9 | SCL | 10 | SDA |

**Pin capabilities:**
- **ADC and EXTINT** are available on A1, D8, D9 and D10 (AIN1/19/17/18).
- **SPI** runs on SERCOM2 with fixed roles: D8 = SCK, D10 = MOSI, D9 = MISO. **There is no CS line on X3**, so an SPI module must use A1 (D1) as CS. Never use the default `SS`, which is A0, the light sensor. SPI is incompatible with front module v1 (§5).
- **A hardware UART** is possible only as TX = D10 (PAD2) with RX = D9 (PAD1) or D8 (PAD3), on SERCOM2. A SAMD21 USART can transmit only on PAD0 or PAD2, and SERCOM0 is already taken by Serial1/IR. The 2026 head uses TX = D10, RX = D9.
- **PWM:** D8 and D10 are on TCC0, so they share the motor PWM period. D9 is on TCC1 CC1, shared with D7.

**Electrical limits for new modules:**
- All X3 signal pins except NEOPIXO are **unprotected SAMD21 I/O in the +3V1 domain** (absolute maximum VDD + 0.6 V ≈ 3.7 V).
- Use 3.0–3.3 V logic powered from +3V1, or level-shift.
- Never pull SDA/SCL (or any signal) up to V+, which is ≈ 5 V on USB.
- Do not drive signal pins while +3V1 is off; that back-powers the SAMD.
- NEOPIXO is the top NeoPixel's DO and probably swings to V+ (INFERRED). Use it only as WS2812 data input.

**Mechanical interface:** X3 is an SMD 2×5 right-angle male header; the module's 2×5 through-holes slide onto it and the module stands vertically at the front (paper Fig. 3). Pitch, row spacing and module outline are **UNKNOWN** because no board files are published. Measure a real bot.

**X2 "SWD"**: 2×5. It looks like the ARM Cortex 10-pin layout, but **pins 6–9 are non-standard. Do not plug in a generic cable blindly.**

| Pin | Net | Pin | Net |
|---|---|---|---|
| 1 | +3V1 | 2 | SWDIO (SAMD PA31) |
| 3 | GND | 4 | SWCLK (SAMD PA30) |
| 5 | GND | 6 | SCL |
| 7 | **V+** | 8 | SDA |
| 9 | **UPDI** (AVR PF7) | 10 | RST (SAMD /RESET) |

**X1**: USB-C receptacle, a sink with 5k1 Rd resistors. SBU pins are not connected.

---

## 5. Front module v1 (APDS-9960), `circuit/headSchema.pdf`

| Part | Wiring |
|---|---|
| **APDS-9960** (Broadcom) | Proximity, ALS, RGBC colour and 4-direction gesture, with a 950 nm IR LED. I2C at fixed **0x39**, ≤ 400 kHz. VDD = +3V1 through a 22 Ω / 1 µF filter. **INT → net SCK = body D8**, with a 10 k pull-up to +3V1. |
| **AP2112K-3.3** LDO | VIN = **V+**. **EN = net MOSI = body D10.** VOUT feeds only the APDS **LEDA** (IR LED anode). |
| Connectors X1/X2 | Two overlaid 2×5 footprints with mirrored numbering. X2 matches the body X3 numbering. A1, MISO and NEOPIX are **unused** on this module. |

**Using it** (as in `sensing.ino` / `colors.ino`):

```cpp
pinMode(8, INPUT_PULLUP);                 // APDS INT (open-drain, active low)
pinMode(10, OUTPUT); digitalWrite(10, HIGH); // enable the APDS IR-LED supply (needed for proximity/gesture)
apds.begin(); apds.enableColor(true);     // Adafruit_APDS9960 defaults: 10 ms integration, 4x gain -> clear full scale ~4096
```

- **Build at `opt=small`.** `Adafruit APDS9960 Library` 1.3.1 depends on Adafruit BusIO (its header includes `Adafruit_I2CDevice.h`), which fails at `opt=fastest` (§9.1). The APDS library itself was not compiled. The examples that use this module fail with the recommended FQBN anyway: like `remoteControl.ino`, `sensing.ino` and `colors.ino` include and use `Adafruit_MLX90393`, the verified failing case (§9.1). `colors.ino` also has its own errors (§11 item 12).
- **Never call `SPI.begin()` with this module fitted.** It hands D8 and D10 to SERCOM2 and breaks INT and the LED enable.
- With D10 LOW, colour and ALS still work, but proximity and gesture have no IR LED.
- Below about 3.35 V of battery the AP2112K is in dropout, so VLEDA ≈ V+. VLEDA falls below the APDS 3.0 V minimum at about 3.05–3.15 V of battery, depending on LED drive current and boost. Proximity weakens there (INFERRED).
- The paper says that, being based on I2C, the module "can reach kilohertz speeds of sensing".
- Phototaxis in the revised paper and `sensing.ino` `launchRT` use this module's **APDS clear channel**: `buzPwr = c / cmax * 127`, floored at 55, with `cmax` defaulting to 4096 and recalibrated with MENU. The arena's LED panels sit below the bots. The body TEMT6202 (A0) faces up and was not used.

---

## 6. Head 2026 (in development), `circuit/GraspionHeadDev2026.pdf`

### 6.1 Parts

| Ref | Part | Notes |
|---|---|---|
| IC1 | **STM32U575CIU6** | Cortex-M33 with FPU and TrustZone, 160 MHz, 2 MB flash, 786 KB SRAM, UFQFPN48, non-SMPS (VCAP 4.7 µF). HSE is a 16 MHz TXC 8Z crystal with 12 pF caps. **No LSE.** |
| U1 | **u-blox ANNA-B112** | nRF52832: Cortex-M4F, 512 KB flash, 64 KB RAM, BLE 5.0, internal antenna (ANT_INT strapped to ANT), 6.5 × 6.5 mm. XL1/XL2 are grounded, so there is **no 32 kHz crystal**; the variant defines `USE_LFRC`. Ordering suffixes other than -70B ship **preflashed with u-blox u-connectXpress**, which drives P0.03, P0.31 and P0.27 as outputs. On this board those are the I2S_DI, I2S_SCK and LPUART1_TX nets. Erase the module over SWD before the STM32 drives I2S or LPUART1 (INFERRED contention risk; the fitted suffix is not on the schematic). |
| MIC1 | **PUI DMM-4026-B-I2S-R** | I2S MEMS mic: SNR 64 dB(A), −26 dBFS. LR = GND, so it outputs in the **left** slot. 32-bit slots, BCLK 2.048–4.096 MHz, so Fs ≥ 32 kHz. The datasheet recommends a 100 kΩ pull-down on SD; it is **missing**. |
| LED1 / LED2 | Red | LED1 on nRF P0.15; LED2 on STM32 PB15. Both active-high through 1 kΩ. The nRF variant's `LED_BLUE` is a deliberate compatibility alias for LED1 (commit 188328b): Bluefruit52Lib requires it and blinks it as the connection LED. **Do not remove it.** |
| SW1 | "boot" | Pulls STM32 BOOT0 high (10 k pull-down otherwise) |
| X1 | micro-USB | D+/D− to STM32 PA12/PA11. **VBUS is not connected, so USB cannot power the head.** No ESD protection. PA9, the OTG_FS VBUS-sense pin, carries I2S_WS on this board, so configure USB with **VBUS sensing disabled** and leave PA9 to SAI1_FS_A (INFERRED). |
| X2 | "bltcon" 2×5 | To the body HEAD connector |
| X3 | "swd" 2×3 | Pins: 1 SWCLK_STM, 2 SWDIO_STM, 3 D+, 4 D−, 5 SWDCLK_nRF, 6 SWDIO_nRF. **No GND, VTref or NRST.** |

- **No regulator.** The whole head runs from the body's **+3V1** (X2 pin 7), which shares the body's 100 mA LDO budget.
- V+ and NEOPIX reach the head but are **unused**.
- There are no I2C pull-ups on the head; it relies on the body's.

### 6.2 Nets

| Link | Pins |
|---|---|
| **Head ↔ body UART** | STM32 **UART4**: PA0 (TX) → 100 Ω → BLATTE_RX → body **D9/PA09**; body **D10/PA10** → BLATTE_TX → 100 Ω → STM32 PA1 (RX). An optional nRF path (P0.26/P0.24 via resistors R3/R4) is not populated. |
| STM32 ↔ nRF #1 | **LPUART1**: STM32 PA2 (TX) → nRF **P0.27** (nRF RX); nRF **P0.28** (TX) → STM32 PA3 (RX) |
| STM32 ↔ nRF #2 | **USART3**: STM32 PA7 (TX) → nRF **P0.30** (RX); nRF **P0.29** (TX) → STM32 PA5 (RX). This is the nRF variant's `Serial` (TX 29, RX 30), which the README also calls the DFU UART at 115200. |
| I2C | One shared bus: STM32 PB6/PB7 (I2C1), nRF P0.18/P0.16, and the body bus |
| I2S | Mic + nRF (P0.31 SCK, P0.02 WS, P0.03 SD; slave RX) + STM32 SAI1_A (PA8/PA9/PA10). The STM32 must be the clock master (INFERRED). |
| Control | STM32 PB0 → nRF RESET_N (10 k pull-up) |
| Extra lines to body | STM32 PB10 → body A1/PA03; nRF P0.23 → body D8/PA11. Purpose **UNKNOWN**. |

- **Mating** (INFERRED, strongly supported): head X2 and body X3 have mirrored numbering. Head odd pin *p* meets body pin 10−*p*; head even pin *p* meets body pin 12−*p*. This is the only mapping under which the power and I2C nets line up.

**Body-side UART for this head** (UNTESTED). Do not use `SPI` in the same sketch.

```cpp
Uart SerialHead(&sercom2, 9 /*RX PA09 <- STM32 PA0*/, 10 /*TX PA10 -> STM32 PA1*/, SERCOM_RX_PAD_1, UART_TX_PAD_2);
void SERCOM2_Handler() { SerialHead.IrqHandler(); }
// setup(): SerialHead.begin(115200);
```

- The lab's earlier STM32/BLE add-on (digital-twin paper, `brainbot-digital-twin/tinympc_deployment_code/fig8.ino`) sent `"M<left>,<right>\n"` at **115200 baud** to the body. `"M0,0\n"` is the safe stop, and values range 60–120. That is a different STM32 board, and the body-side parser is not published.

### 6.3 Firmware status and documentation errors

- **nRF52 Arduino package:** board `graspion:nrf52:graspionHead` (Adafruit nRF52 core 1.7.0, S132 6.1.1, linker script `nrf52832_s132_v6.ld`).
  - The package also patches `nrfx_i2s` in. There is **no Arduino I2S library**: call `nrfx_i2s_*` directly.
  - **First programming must be over SWD:** X3-5 SWDCLK_B112 and X3-6 SWDIO_B112, with GND from X2-5 or X1-5 because X3 has no GND pin. Flash the S132 6.1.1 SoftDevice, then the app (openocd `daplink_nrf52.cfg` or nrfjprog).
  - The board's upload tool is serial DFU (`nrfutil`), but **no graspionHead bootloader or SoftDevice step is shipped**, and the nRF's UART only reaches the STM32. Serial DFU through an STM32 USART3 bridge can work only after someone flashes a serial-DFU bootloader (INFERRED).
  - Selecting `debug_output=serial1` **fails to compile**: core 1.7.0 never declares `Serial1` on the nRF52832, which has a single UARTE, but `main.cpp` references it when `CFG_LOGGER=1`. Use the Serial or RTT option.
- **`arduino/nrf52/README.md` is stale.** It says app UART "P0.27 TX → STM32 PA3, P0.28 RX ← PA2". **The direction is reversed**: per the schematic, the nRF must receive on P0.27 and transmit on P0.28. `variant.h` (Serial on 29/30) is correct. Commit `714eb42` moved it there.
- **No STM32 firmware exists** in the repo.
- Paper text (revised version): a Bluetooth module "under test" will offer individual identification and about 10 m range.

---

## 7. Mechanics

| Part | Spec | Source |
|---|---|---|
| Body | Ellipse 60 × 30 mm, 9 mm tall without legs, 6 g (paper), **ABS**, printed on a Stratasys J35 PolyJet. Post-processing: tap the holes. File: `prints/GRASPion body.stl` (bounding box exactly 60 × 30 × 9 mm). The STL encloses only ≈ 3.0 cm³ (≈ 3.5 g solid), so the 6 g figure and the STL disagree. Weigh a body if mass matters for modelling. | Paper, STL |
| Leg plate, circular | 4 legs, **Ø0.8 mm, tilted 12°** (all toward −x), 4.98 mm long, on a 1 mm plate (29.3 × 26.2 mm footprint). Fixed with 2× M2 screws. | Paper, STL |
| Leg plate, rectangular | 4 legs, **0.6 × 1.8 mm cross-section, tilted 10°**. More durable and faster, but with less trajectory control. | STL, paper |
| Both leg plates | Carry an undocumented Ø0.8 mm centre pin on the leg side that protrudes 6.0 mm below the plate, **≈ 1.1 mm beyond the leg tips** (4.87–4.91 mm). Left on, the bot would rest on it, so it almost certainly has to be removed after printing (INFERRED; possibly a print aid). Check before running a bot. | STL |
| Wedge plate | Sits between the leg plate and the body to give a forward pitch. Sloped face **3.76°** (measured); the assembled pitch is **UNKNOWN**. | Paper, STL |
| Printing | **Legs:** PLA (Prusament PLA Galaxy) on a Prusa XL, 0.15 mm "STRUCTURAL" profile (`prints/… - BB LEGS.txt`), 15 % grid infill, 2 perimeters, no supports, **one object at a time** (`complete_objects = 1`); batch-printing legs weakens them. **Wedge:** same PLA; printer and profile are not stated (INFERRED to be the same). | README, `prints/*.txt` |
| Assembly order (paper Fig. 2) | Leg plate → wedge → body → 2 motors (spring contacts) → coin cell → PCB | Paper |

**Body frame and part placement.** INFERRED from the V1.1 photo; V1.3 may differ. Axes: +x = front (module end), +y = bot's left, +z = up.
- Front: the module faces +x.
- Top side:
  - top NeoPixel at front-centre;
  - MLX90393 next to it;
  - light sensor at centre-right;
  - IR receiver and IR LED on the right (−y) edge, the receiver toward the front and the LED toward the rear.
- Rear (−x): USB-C, the orange charge LED and the AVR.
- Underside:
  - the coin cell under the rear half;
  - motor pads under the front half;
  - the rear NeoPixel at the rear tip.
- Lab photos show white elliptical top caps with a black forward-pointing triangle, likely a heading marker for tracking.

---

## 8. Performance and experiments (for modelling)

| Quantity | Value | Source |
|---|---|---|
| Top speed | 16 cm/s (paper Fig. 1a); about 15 cm/s (README) | Paper, README |
| Calibrated speed range | 4.7–11.5 cm/s for powers 60–120; |ω| up to about 3.5 rad/s | Digital-twin lookup tables |
| Single-motor rotation period | About 1.71 s CW and 1.99 s CCW; centre-of-mass circle radius about 1.5–1.9 cm | INFERRED from paper figure data |
| Diffusive mode (`launchRT`) | Straight and turn durations each uniform in [400, 1200] ms; turn direction 50/50 | Paper, examples |
| Diffusive-mode statistics | Ballistic speed about 6.5 cm/s; crossover about 1 s; D_eff about 6–7 cm²/s | INFERRED from paper Fig. 5 data |
| Random walk (stigmergy paper) | Persistence 0.8 s, persistence length about 10 cm | arXiv 2608.24383 |
| Tracking | The GRASPion paper does not describe its tracking. The predecessor used an overhead camera with OpenCV; the stigmergy paper an overhead camera; the digital-twin work an optical camera plus an ArUco marker (mounting not stated). About 40 fps (INFERRED). | Papers |

**Demonstrated uses:**
- IR remote control and chiral rotation.
- Diffusive motion.
- Optical colour synchronisation, a discrete Kuramoto model. As `colors.ino` implements it: ω ← ω + K·sin(φ_d − φ) with K = +0.004 (attractive), where φ is the bot's own colour index and φ_d the nearest palette colour seen by the APDS. It uses a 6-colour palette and a DDS at 1 kHz on TCC2. The paper prints sin(φ − φ_d), which matches the code only for K < 0, so **follow the code's sign**.
- **Phototaxis**: diffusive speed scales with light received by the front-module APDS-9960, on an arena lit by LED panels underneath (revised paper only; see §5).
- Synthetic tissue and cells: bots inside soft membranes (site).
- Stigmergic transport among wooden cubes.
- Onboard TinyMPC with Bluetooth pose feedback at 30 Hz.

**Predecessor "Brainbot"** (Noirhomme et al., PR Applied 23, 064008, 2025): 8-bit MCU, **one** motor, 13 g, about 3.5 cm/s, 10 legs, microphone, micro-USB, 60 min runtime. Do not confuse its numbers with GRASPion's.

---

## 9. Toolchain: build, flash and identify bots

### 9.1 Board package

- **Index URL:** `https://raw.githubusercontent.com/GRASP-LAB/GRASPion/main/package_graspion_index.json`
- **Platforms:** `graspion:samd` 1.0.0, built on Adafruit ArduinoCore-samd 1.7.17, and `graspion:nrf52` 1.0.3. The index keeps only the latest version of each.
- **Body FQBN:** `graspion:samd:graspionBody:opt=fastest,usbstack=tinyusb`. The menu defaults are `opt=small` (-Os) and `usbstack=arduino`; the README asks for Ofast and TinyUSB.
  - **Exception:** a sketch that uses any library built on Adafruit BusIO does not build at `opt=fastest`, `opt=faster` or `opt=dragons`. That is verified for MLX90393 and for BusIO alone; APDS9960 includes the same BusIO header (not compiled itself). All three upstream examples include `Adafruit_MLX90393.h`, so none of them builds at `opt=fastest`. Use `opt=small,usbstack=tinyusb` for such sketches (verified). `opt=fast` (-O2) is UNTESTED for a whole sketch (see "Build failures" below).
- The upstream README still says to use "Adafruit QT Py M0", and `examples/upload_all.sh` uses `adafruit:samd:adafruit_qtpy_m0`. **Do not.** That variant drives D12 (motor enable) HIGH at boot while the PWM pins float. The authors' own unmerged README proposal recommends GRASPion Body.
- **Libraries** (exact Library Manager names). A version means it was compiled for graspionBody on 2026-10-15:
  - `Adafruit NeoPixel` (1.15.5)
  - `Adafruit DMA neopixel library` (samd only)
  - `Adafruit APDS9960 Library` (depends on BusIO, so `opt=small`)
  - `Adafruit MLX90393` (2.0.5; pulls in `Adafruit SSD1306`; depends on BusIO, so `opt=small`)
  - `Adafruit Unified Sensor` (1.1.15)
  - `Adafruit BusIO` (1.17.4; **fails at `opt=fastest`**)
  - `IRremote` (4.7.1; 4.x API, so pin `IRremote@4.7.1`)
- `Adafruit_ZeroDMA` and `Adafruit_TinyUSB` are bundled in the core.
- **With `usbstack=tinyusb`, `#include <Adafruit_TinyUSB.h>` in every sketch.** `Serial` links only if that header reaches the build, from the sketch or from a library header that includes it (see below). Guard the include with `#if defined(USE_TINYUSB)` if the sketch must also build with `usbstack=arduino`.

```bash
arduino-cli config init   # only needed once, on a fresh install
arduino-cli config add board_manager.additional_urls https://raw.githubusercontent.com/GRASP-LAB/GRASPion/main/package_graspion_index.json
arduino-cli core update-index && arduino-cli core install graspion:samd
arduino-cli lib install "Adafruit NeoPixel" "Adafruit DMA neopixel library" "Adafruit APDS9960 Library" "Adafruit MLX90393" "Adafruit Unified Sensor" "Adafruit BusIO" "IRremote@4.7.1"
arduino-cli compile --fqbn graspion:samd:graspionBody:opt=fastest,usbstack=tinyusb --output-dir build/mysketch mysketch
arduino-cli compile --fqbn graspion:samd:graspionBody:opt=small,usbstack=tinyusb --output-dir build/mysketch mysketch   # sketches using BusIO-based libraries
arduino-cli upload  --fqbn graspion:samd:graspionBody:opt=fastest,usbstack=tinyusb -p COMx --input-dir build/mysketch mysketch
```

The sketch `.ino` must sit in a folder of the same name.

**What was compiled** (2026-10-15): arduino-cli 1.5.2-rc.1 on Windows 11, `graspion:samd` 1.0.0 with its bundled arm-none-eabi-gcc 9-2019q4. Builds only; nothing was flashed or run.

| Sketch | FQBN options | Result |
|---|---|---|
| `calibration/firmware/calib_bot` (calib_bot/2, protocol 2; IRremote, NeoPixel, Wire; MLX90393 over raw Wire) | `opt=fastest,usbstack=tinyusb` | builds in all four variants: default 732 Hz PWM 60,540 B flash, `-DPWM_40KHZ=1` 60,916 B, `-DBENCH_HANG=1` 60,732 B, `-DUSE_HW_WDT=0` 60,292 B. Static RAM (.data + .bss) 752 + 20,672 = 21,424 B of 32 KB in every variant |
| Minimal Adafruit MLX90393 sketch | `opt=fastest,usbstack=tinyusb` | **fails**: `lo register required` |
| Same sketch | `opt=small,usbstack=tinyusb` | builds: 44 KB flash |
| Adafruit BusIO only (`Adafruit_I2CDevice`, no sensor library) | `opt=fastest,usbstack=tinyusb` | **fails**: same error |
| BusIO's `Adafruit_SPIDevice.cpp` alone, compiled with the core's flags (one file, no link) | `-Os` / `-O2` / `-O3` / `-Ofast` / `-Ofast -funroll-loops` (`opt=small` / `fast` / `faster` / `fastest` / `dragons`) | assembles / assembles / **fails** / **fails** / **fails** |
| `Serial` without `#include <Adafruit_TinyUSB.h>` (and no library that includes it) | `opt=fastest,usbstack=tinyusb` | **fails to link** |
| `calibration/blaster/ir_blaster` (ir_blaster/3, protocol 2; IR bridge, not for bots; /3 adds the CAN byte 0x18 line-cancel that the PC's resync sends) | `graspion:samd:adafruit_qtpy_m0:opt=fastest,usbstack=tinyusb` | builds once the TinyUSB header is included: 51,832 B flash |
| Same sketch, Nano blaster | `arduino:avr:nano` (arduino:avr 1.8.8, not the GRASPion core) | builds: 8,608 B flash, 562 B RAM |

Not run: `upload` (no bot attached), a whole sketch at `opt=fast`, `opt=faster` or `opt=dragons`, and any sketch with `Adafruit APDS9960 Library` or `Adafruit DMA neopixel library`, which were not installed.

**Build failures and workarounds** (verified by compiling as above):
1. **Adafruit BusIO at `opt=fastest`.** BusIO's `Adafruit_SPIDevice.cpp` (its bit-banged SPI, which arduino-cli compiles even when the sketch only uses I2C) calls the core's inline `delayMicroseconds` (`cores/arduino/delay.h`). Its asm loop `sub %0, #1` takes the counter with a `"+r"` constraint. At `-O3` and above (`opt=faster`, `opt=fastest`, `opt=dragons`), gcc 9.2 assigns `ip` (r12), and the Thumb-1 assembler rejects it: `Error: lo register required -- 'sub ip,#1'`. Any sketch that pulls BusIO into the build inherits this. That is verified for the MLX90393 library and for BusIO alone. `Adafruit APDS9960 Library` 1.3.1 includes the same `Adafruit_I2CDevice.h`, so it fails the same way (the library itself was not compiled). Workarounds:
   - build with **`opt=small`** (-Os; verified with a whole MLX90393 sketch). This applies to the whole sketch and core, not just BusIO;
   - `opt=fast` (-O2) costs less speed, but only the failing file was shown to assemble at -O2: a whole sketch at `opt=fast` is UNTESTED;
   - or drive the device over `Wire` directly, as `calibration/firmware/calib_bot/calib_bot.ino` does for the MLX90393 (datasheet command set, register layout from the Adafruit library; about 80 lines).
   - A `"+l"` (low-register) constraint in the core's `delay.h` should fix it at the source (INFERRED; not verified, and it means editing the installed core).
   - The root cause is the core's inlined `delayMicroseconds`, so in principle any file calling it can hit this at `-O3` or above; BusIO is the known case (INFERRED).
2. **TinyUSB include.** With `usbstack=tinyusb`, a sketch whose build never includes `<Adafruit_TinyUSB.h>` fails to link with `undefined reference to 'Serial'` and `Adafruit_USBD_CDC::begin`. The core's `Arduino.h` only declares `Serial`; `main.cpp` calls `TinyUSB_Device_Init` from the bundled TinyUSB library, which arduino-cli links only when the header reaches the build: from the sketch, or from a library header that includes it. `Adafruit_NeoPixel.h` 1.15.5 does so under `USE_TINYUSB` (lines 42-43), so a NeoPixel sketch probably links without its own include (INFERRED from the sources; not built). Include it explicitly in every sketch anyway. The link failure was seen on both `graspionBody` and `adafruit_qtpy_m0` of this core.
3. **Long paths on Windows.** If the arduino-cli data directory sits under a long path, gcc fails with `cannot execute 'cc1plus'` (Windows MAX_PATH). Keep the default `%LOCALAPPDATA%\Arduino15`, or map a short drive letter with `subst` and point `directories.data` there.

### 9.2 Upload, bootloader and recovery

- **Upload method:** bossac / SAM-BA with a 1200-baud touch. The app starts at offset 0x2000, above an 8 KB **UF2 bootloader** (QTPY_BOOT drive).
- **USB ID:** VID 0x239A, PID 0x80CB, shared with the QT Py, so board names are ambiguous in `board list`.
- **The bot must be ON:** press SW1 once. The battery is optional when USB is connected.
- **Bulk flashing:** `examples/upload_all.sh` is Linux-only (`/dev/ttyACM0..19`), recompiles for every bot and uses the wrong FQBN. Better: compile once and loop `arduino-cli upload` over ports with VID 239A, pressing each bot's button first.
- **Recovery** when a sketch kills USB (INFERRED, standard UF2 behaviour): double-tap X2 pin 10 (RST) to GND (X2 pin 3 or 5) to get the bootloader, or reflash over SWD (X2 pins 2/4/10 + GND). Adding a watchdog helps (`Adafruit SleepyDog Library` supports SAMD).
- **EN while the bootloader runs.** Which UF2 bootloader a bot carries is UNKNOWN: double-tap RST and read `INFO_UF2.TXT` on the QTPY_BOOT drive.
  - A QT Py UF2 bootloader ≥ 3.11 (the version is in `INFO_UF2.TXT`) powers its NeoPixel from PA15 = D12, so it holds **EN HIGH with D2/D3 floating: the motors may run at full drive**. This lasts as long as the bootloader runs: during double-tap recovery and uploads (the 1200-baud touch leaves the bootloader resident), and for its 500 ms double-tap wait after any reset that is not power-on. Lift the bot or hold its legs off the surface.
  - The bootloader bundled with the core (v1.23.1-…-gb149075) powers its NeoPixel from PA19 and leaves EN floating.
  - `calib_bot` sets the bootloader's quick-boot flag just before it resets at its watchdog's early warning or after a HardFault, which skips the 500 ms wait (calibration/README.md §1; UNTESTED). A WDT time-out, the fallback when the early warning is blocked, sets no flag. The real fix is a GRASPion build of uf2-samdx1 without `BOARD_NEOPIXEL_POWERPIN` (not done).
- **AVR (UPDI on X2 pin 9):** do not reflash it. Its firmware is not public.

### 9.3 Bot identity

- Each SAMD21 has a unique 128-bit serial number at words 0x0080A00C, 0x0080A040, 0x0080A044 and 0x0080A048.
- Both USB stacks expose it as the **USB iSerial** string. Map COM port to physical bot by serial number, and read the same words in firmware to get a bot ID.

---

## 10. Runtime resource map (SAMD21 with the examples' libraries)

| Resource | Used by | Free? |
|---|---|---|
| SysTick | `millis`/`delay` | — |
| TCC0 CC0–CC3 | analogWrite D2 (R motor), D3 (L motor), D10, D8; 732 Hz | Shared period, so changing PER affects all four |
| TCC1 | analogWrite D6, D7, D9, D17 | Free if those pins are not used for PWM |
| TCC2 | `colors.ino` 1 kHz DDS interrupt | Free otherwise |
| TC3 | IRremote receive, 50 µs tick, NVIC priority 0 | — |
| TC4 / TC5 | Servo / `tone()` | Free otherwise |
| SERCOM0 | Serial1 (IR TX D6 / RX D7) | — |
| SERCOM1 | Wire (I2C) | — |
| SERCOM2 | SPI on the HEAD connector, **or** a UART to the 2026 head | — |
| SERCOM3 | SPI1 flash **or** NeoPixel ZeroDMA, not both | — |
| EIC shared lines | D0 & D11, D1 & D15, D6 & D16, D7 & D14 | — |
| DAC | A0 = light-sensor node | **Never use** |

**Clock accuracy:**
- The board is **crystalless**: the DFLL48M locks to USB SOF only when a USB host is attached. On battery it runs open-loop at **47–49 MHz (±2 %)**, plus temperature drift.
- `millis`, PWM, baud rates and DDS frequencies therefore differ by up to about ±2 % between bots, and between USB and battery operation. **Expect intrinsic frequency spread in oscillator/synchronisation experiments.**

**TinyUSB stack (as the README requires):**
- `#include <Adafruit_TinyUSB.h>` in every sketch: `Serial` links only if that header reaches the build, from the sketch or a library header (§9.1).
- USB is serviced only in `yield()`, which runs after each `loop()` and inside `delay()`. A `loop()` that never returns or yields breaks uploads.
- **`Serial.end()` removes the COM port.** The examples call it when `mVusb < 4500`, so the port vanishes until reset.
- Never use `while(!Serial)`: it blocks forever on battery.

---

## 11. Gotchas and known bugs (consolidated)

1. **Board choice:** QT Py variant = D12 HIGH at boot, so the motors are enabled with floating PWM. Use `graspion:samd:graspionBody`, and first thing in `setup()` call `pinMode(BUZ_ENABLE, OUTPUT); digitalWrite(BUZ_ENABLE, LOW);`. `digitalWrite` alone does not drive a SAMD pin that is not an output. D12 reaches `setup()` either as a floating input (the bootloader bundled with the core, v1.23.1-…-gb149075, powers its NeoPixel from PA19) or as an output driven HIGH (a QT Py UF2 bootloader ≥ 3.11 powers it from PA15 = D12), so `pinMode(OUTPUT)` then `LOW` must be the very first statement. While a ≥ 3.11 bootloader runs (uploads, double-tap recovery), EN is HIGH regardless (§9.2).
2. **PWM frequency** is 732 Hz against the DRV2603's 10–250 kHz spec (§4.4). 127 is not neutral; **128 is**.
3. **Motor overdrive:** full scale (254) gives about 3.3 V, the motor's maximum. The rated 2.7 V is about analogWrite 231 (INFERRED).
4. **Power:** the button only works if the AVR still has SYS. After over-discharge, overcurrent (≥ 400 mA) or ship mode, **only USB revives the bot**.
5. **IR:** IRremote kills Serial1 RX. 4800-baud UART-over-IR breaks the TSOP AGC4 rules. NEC addresses are ignored. Keep D6 HIGH when idle.
6. **MLX90393:** the address is 0x10, but the library default is 0x0C. `readData()` blocks about 210 ms with the default filter. Motor magnets disturb readings. The Adafruit library does not build at `opt=fastest` (item 18).
7. **`randomSeed(analogRead(A0))`** gives about 0 in the dark, so all bots get the same sequence, and `randomSeed(0)` is a no-op. Seed from the 128-bit chip serial XOR a few ADC LSBs.
8. **`analogRead`** is 10-bit unless you call `analogReadResolution(12)`.
9. **SPI vs the APDS front module:** `SPI.begin()` breaks APDS INT (D8) and the LED enable (D10).
10. **SERCOM3:** ZeroDMA NeoPixels and the SPI1 flash cannot run together.
11. **TinyUSB:** `#include <Adafruit_TinyUSB.h>` in every sketch. arduino-cli links `Serial` only if that header reaches the build, from the sketch or from a library header (`Adafruit_NeoPixel.h` includes it). Keep `loop()` yielding, never call `Serial.end()`, never `while(!Serial)`.
12. **The upstream examples do not build with the recommended FQBN.**
    - All three (`remoteControl.ino`, `sensing.ino`, `colors.ino`) include and use `Adafruit_MLX90393` (line 4; objects at lines 27 / 31 / 55), which pulls in BusIO and fails at `opt=fastest` (item 18). This follows directly from the verified MLX90393 build (§9.1), not from the APDS9960 dependency, which is a second reason for `sensing.ino` and `colors.ino`.
    - None of them includes `<Adafruit_TinyUSB.h>` itself. `Serial` probably still links under TinyUSB, because their NeoPixel headers include it (INFERRED). All three call `Serial.end()`, which item 11 forbids under TinyUSB.
    - Workaround: build with `opt=small`, and for `usbstack=tinyusb` remove the `Serial.end()` calls and add the TinyUSB include (UNTESTED for the examples).
    - `colors.ino` also has its own errors: `launchDIFF()` and `launchBALL()` are undefined, and AV/0x0A is not handled. Its 0x13 handler, and `sensing.ino`'s, prints labels to `Serial` but values to `Serial1`.
13. **`readBrdPwr` uses a `char` buffer.** It works on ARM only because `char` is unsigned there. Use `uint8_t`.
14. **Divide-by-zero in the examples:** `x/c` and `c/cmax` when the clear count is 0 (for example pressing MENU in the dark sets `cmax = 0`).
15. **Documentation drift:**
    - The README says "two examples" (there are three) and recommends the QT Py board.
    - `variant.h` names the flash GD25Q16C.
    - The nRF README has the UART direction reversed.
    - The paper says the AVR handles IR reception; it does not.
    - The paper's photos show board **V1.1**, not V1.3.
16. **Plain-text extraction of the arXiv HTML** turns "9 mm / 6 g / 2 MB" into "99 mm / 66 g / 22 MB". Trust the PDF.
17. **Battery-dependent behaviour:** NeoPixel brightness, IR LED current and high-duty motor speed all change between USB and battery and with charge state. The DRV2603 compensates only while V+ ≥ about 3.4 V (INFERRED).
18. **Build failures** (verified by compiling, §9.1):
    - **Adafruit BusIO at `opt=fastest`:** the core's `delayMicroseconds` inline asm (`"+r"`) gets `ip` at `-O3` and above, and Thumb-1 rejects it: `lo register required -- 'sub ip,#1'`. So `opt=faster`, `opt=fastest` and `opt=dragons` all fail. This breaks the MLX90393 library (and so all three upstream examples) and APDS9960, which includes the same BusIO header. Use `opt=small` (verified), or raw `Wire` as in `calibration/firmware/calib_bot/calib_bot.ino`. `opt=fast` (-O2) is UNTESTED for a whole sketch.
    - **Windows:** a long arduino-cli data path gives `cannot execute 'cc1plus'` (MAX_PATH). Use the default `%LOCALAPPDATA%\Arduino15` or a `subst` drive.

**Example sketch IR command table** (NEC command byte):

| Cmd | Button | Action |
|---|---|---|
| 0x00 | UP | Both motors forward (straight) |
| 0x01 | DOWN | Stop all, exit modes |
| 0x02 | RIGHT | Left motor only (turn right) |
| 0x03 | LEFT | Right motor only (turn left) |
| 0x09 | MUTE | `sensing.ino`: stop, `buzPwr = 127` |
| 0x0A | AV | Run-and-tumble (`launchRT`); light-modulated speed in `sensing.ino` |
| 0x0F | DISP | `sensing.ino`: light-gradient search (`launchSearch`) |
| 0x11 | 2 | `sensing.ino`: toggle top LED white |
| 0x13 | 4 | Print mVusb/mVbat (buggy, see item 12) |
| 0x4A | MENU | `sensing.ino`: calibrate `cmax` = current APDS clear count |
| 0x80 / 0x81 | Vol+ / Vol− | `colors.ino`: `buzPwr` ±10, coupling +0.0005 / −0.001 |

---

## 12. Open questions (UNKNOWN; measure on hardware or ask the authors)

- **AVR firmware:**
  - the full 0x36 register map, including byte 4, and whether writes are accepted;
  - IR carrier polarity and duty, and the maximum IR baud;
  - button long-press and power-off behaviour;
  - when it resets the SAMD;
  - its I2C speed limit (does it tolerate 400 kHz?).
- Battery: actual cell vendor and capacity on V1.3, and charge time.
- NeoPixel part numbers.
- MLX90393 axis orientation in the body frame.
- Whether the fitted IR receiver matches the schematic's TSOP75438TT (lens-up), and IR range.
- X3 header pitch, row spacing and front-module outline (no board files).
- Real DRV2603 behaviour at 732 Hz PWM input.
- Why the right motor's polarity is reversed on V1.3.
- Leg-plate centre pin purpose and the assembled body pitch.
- Head 2026:
  - which STM32↔nRF link carries the application and which carries DFU;
  - the purpose of PB10 and P0.23;
  - the I2C roles of the STM32 and nRF;
  - whether the ANNA-B112 antenna tuning strip exists in the layout;
  - whether the nRF52 I2S can capture the mic's 32-bit slots;
  - whether the body's 100 mA LDO can supply the head's load.

---

## 13. Sources

- **Upstream repo files:**
  - `README.md`, `CITATION.cff`, `package_graspion_index.json`
  - `arduino/samd/*`, including `variants/graspionBody/variant.{h,cpp}` and `samd.boards.txt`
  - `arduino/nrf52/*`, including `variants/graspionHead/variant.{h,cpp}`
  - `examples/{remoteControl,sensing,colors}.ino`, `examples/upload_all.sh`
  - `prints/*`
  - `circuit/*.pdf`
  - Unmerged branch `readme-graspion-body-proposal` (commit a05e17d)
- **Datasheets:**
  - Microchip SAM D21 DS40001882H and AVR32DD20 DS40002413A
  - ST STNS01 DocID024654 Rev 4
  - TI DRV2603 SLOS754C
  - Vybronics VZ3TH8B171700L
  - Melexis MLX90393 rev 012
  - Vishay: TEMT6202 doc 80573, VSMB1940 doc 81933, TSOP752/754 doc 82494, NTCS0603E3 doc 29056
  - Winbond W25Q16JV
  - Diodes Inc.: DMP3164LVT, DMN3135LVT, DMN2053UVT, AP2112
  - Broadcom APDS-9960 AV02-4191EN
  - u-blox ANNA-B112 UBX-18011707
  - PUI DMM-4026-B-I2S-R
  - TXC 8Z series
  - ST STM32U575 DS13737 (read via distributor and metadata pages)
- **Software sources:**
  - Adafruit ArduinoCore-samd 1.7.17 (`wiring_analog.c`, `startup.c`, TinyUSB port)
  - Adafruit_nRF52_Arduino 1.7.0
  - IRremote 4.7.1
  - Adafruit MLX90393, APDS9960 and SPIFlash libraries
- **Build tests** (§9.1): arduino-cli 1.5.2-rc.1 with graspion:samd 1.0.0 (and arduino:avr 1.8.8 for the blaster's Nano build), Adafruit MLX90393 2.0.5, Adafruit BusIO 1.17.4, IRremote 4.7.1 and Adafruit NeoPixel 1.15.5, 2026-10-15; the library dependency lists come from the Arduino library index. The sketches are in [calibration/](calibration/README.md) (calib_bot/2, ir_blaster/3, protocol 2, recompiled after the last review fixes), plus minimal MLX90393, BusIO-only and no-TinyUSB-include test sketches.
- **Papers and site:**
  - RSI 97, 014704 (2026) / arXiv:2509.07437 / ULiège postprint
  - Noirhomme et al., PR Applied 23, 064008 (2025)
  - Mammadli et al., Phys. Rev. Research 8, 033164 (2026), and github.com/GRASP-LAB/brainbot-digital-twin
  - Wéry et al., arXiv:2608.24383 (2026)
  - graspion.be
