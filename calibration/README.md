# Phase 0 calibration ("learning to walk")

Tooling to measure each GRASPion's map from motor drive (L, R) to body motion (u1, u2, ω), plus its lag, dead time and noise. This is [FEASIBILITY.md](../FEASIBILITY.md) §5.2–5.4 implemented: an overhead camera tracks an ArUco marker, a USB-serial IR blaster sends addressed frames, and calibration firmware on the bot drives segments under a command watchdog.

> **Status: nothing here has run on a bot, a blaster or a real camera. Treat all of it as UNTESTED.**
>
> What *was* verified (2026-10-15):
> - **Compiled** after the last review fixes (calib_bot/2, ir_blaster/3, protocol 2) with arduino-cli 1.5.2-rc.1:
>   - `calib_bot` for `graspion:samd:graspionBody:opt=fastest,usbstack=tinyusb` (graspion:samd 1.0.0) in four variants: default 732 Hz PWM (60,540 B flash), `-DPWM_40KHZ=1` (60,916 B), `-DBENCH_HANG=1` (60,732 B) and `-DUSE_HW_WDT=0` (60,292 B). Static RAM (.data + .bss): 752 + 20,672 = 21,424 B of 32 KB in every variant.
>   - `ir_blaster` for `arduino:avr:nano` (arduino:avr 1.8.8; 8,608 B flash, 562 B RAM) and `graspion:samd:adafruit_qtpy_m0:opt=fastest,usbstack=tinyusb` (51,832 B flash). ir_blaster/3 adds the CAN line-cancel (byte 0x18) that the PC's bridge resync sends (§1).
>   - Libraries: IRremote 4.7.1, Adafruit NeoPixel 1.15.5.
>   - Both sketches include the protocol-2 `static_assert` vectors, so the C header and the Python mirror agree on ARM and AVR.
>   - **Rebuilt 2026-10-27** with arduino-cli 1.5.1, graspion:samd **1.0.1** and IRremote 4.7.1: all four `calib_bot` variants build (60,236 / 60,516 / 60,372 / 60,108 B flash for default / `PWM_40KHZ` / `BENCH_HANG` / `USE_HW_WDT=0`; static RAM 752 + 21,428 = 22,180 B), and `ir_blaster` builds unchanged (Nano 8,608 B, QT Py M0 51,832 B). Sizes per toolchain are in HARDWARE.md §9.1.
> - **134 offline tests** (run 2026-10-30 with Python 3.14.0, numpy 2.5.4, scipy 1.18.1, OpenCV 5.0.0): 133 pass, and 1 is skipped unless `GRASPION_TILEMAT_SIM` is set (§4). They cover:
>   - protocol and CRC, including every on-air burst of up to 5 bits, cross-checked against the C header through compile-time `static_assert` vectors (compiled on ARM and AVR);
>   - the twist/ICR fit on EllipticalBot2D trajectories. The skipped test is the one against the user's own `Ellipse2dObj.EllipticalBot2D`, imported read-only;
>   - tracking on rendered scenes (including mono cameras, LED-height parallax, reference survey errors, a homography published only after its checks, and rate-limited "reference off" warnings), the `track` command (LED height and camera height from the rig config, reference summary, warnings) and ChArUco intrinsics on rendered views;
>   - every scheduler safety path (re-centring stops, the exit path after Ctrl-C, the serial bridge against a fake blaster), the manual mode (Enter runs from where the bot stands, a nudge is RECENTRE frames ended by STOP and stops at the fence, a hidden marker prompts nothing, skip and quit, `schedule cells` and `run --only` through the CLI on the simulated rig), the refusal of a blaster or bot on another IR protocol version (fake blaster and fake bot serial ports; §3 build notes), a resync that drops a stale half-written request unrun, and the LED checks with NaN samples (including no LED sample before a SEG) and on mono cameras (warning, fallback to the bot's latch, `stalled`), against a simulated rig;
>   - the firmware's frame handling, mirrored in `sim.py`. `test_firmware_sim.py` pins the watchdog-feed rules: PINGs do not feed it (a SEG followed by PINGs trips 300 ms after the SEG), RECENTRE does, a DRIVE without a segment stops and counts `drive_no_seg`, and a run of PINGs shares one log record that a SEG splits. `SimFirmware.i2c_ok = False` mirrors the I2C-down SEG refusal (red flash, no segment, no battery reads); it has no test yet;
>   - these firmware and blaster tests run the Python models (`sim.SimFirmware`, `tests/simrig.FakeBlasterSerial`). The `.ino` code itself is covered only by compiling it with the `static_assert` vectors. The deadman, the hardware watchdog, the I2C time-outs and the bus recovery exist only in the firmware and have no offline test;
>   - the simulated IR channel: per-frame NEC lengths (50–86 ms), each frame handled `RECORD_GAP_S` + 0.5 ms (8.5 ms) after it ends, both of two frames closer than 8 ms lost, and the 100–2000 ms period enforced by `SimBridge`;
>   - the lag fit under heading diffusion and lost IR frames, and visible-stop detection with dropped frames and with a step down to a slow level;
>   - flash matching with a camera offset above the ping period (the anchor PINGs settle it), and an ambiguous offset that is not lent to other sessions;
>   - a full simulated calibration (schedule → scheduler → tracker CSV + bot log → analysis), compared against the known truth, plus the link test, bot-log alignment and abandoned segments.
>
> Labels follow [HARDWARE.md](../HARDWARE.md): **UNTESTED** = written but never run on hardware, **INFERRED** = reasoned, not measured, **UNKNOWN** = must be measured.

## Contents

```
calibration/
  firmware/calib_bot/      bot firmware (GRASPion body, SAMD21)
    calib_bot.ino
    calib_protocol.h          IR frame format (shared, byte-identical copy in blaster/)
    calib_protocol_vectors.h  generated compile-time test vectors (Python <-> C)
  blaster/ir_blaster/      USB-serial -> IR bridge (Nano, RP2040 or QT Py M0)
  pc/
    graspion_calib/        Python package (python -m graspion_calib ...)
      protocol.py   frame packing, CRC-5, bot ID hash (mirror of calib_protocol.h)
      bridge.py     blaster serial link, frames.csv log
      tracker.py    ArUco tracker, homography at marker height, sync-LED sampling
      intrinsics.py ChArUco camera calibration
      scheduler.py  segment runner: re-centring (automatic or manual), keep-alives, PC-side stops
      schedules.py  grid / cells / steps / PRBS / circles / ping / static / validation schedules
      extract.py    session loading, flash-based time alignment, per-segment windows
      twist.py      per-segment body twist (u1, u2, ω) and ICR
      forward_map.py dead-zone power-law forward map, grouped CV, bootstrap
      gp.py         optional GP comparator
      inverse_map.py closed-form inverse, feasible-set polytope, C export
      dynamics.py   first-order lag + dead time
      noise.py      heading diffusion (Brownian / OU), measurement noise, jitter
      botlog.py     bot USB log parsing and merge (battery, magnetometer, bot clock)
      report.py     analysis pipeline -> calib.json
      sim.py        simulated bot + firmware + IR + camera (tests and --sim dry runs)
    tests/                 offline test suite (python -m graspion_calib selftest)
    configs/rig_example.json
```

## 1. How it works

```
 PC ──USB──► IR blaster ──38 kHz NEC frames──► bot TSOP (D7, lens up)   bot ──► motors, top-LED flash
  ▲                                                                       │
  └──────────────── overhead camera (ArUco pose + LED colour) ◄───────────┘
```

**Segment timing:**
1. **Rest:** the PC sends STOP and waits 0.8 s.
2. **Start:** it sends `SEG(L, R)`. The bot starts the motors and flashes its top NeoPixel blue at about 15 % for 70 ms.
3. **Drive:** the PC sends `DRIVE(L, R)` keep-alives at the blaster period (110 ms by default; `bridge.period_ms` accepts 100–2000 ms), and level changes for step/PRBS segments.
4. **Stop:** after the segment duration it sends STOP.
5. **Re-centre:** between segments a bang-bang law (pivot, then straight) brings the bot back to the arena centre with `RECENTRE` frames, under its own stops (table below). With `run --recentre manual` the session instead stops and waits before every segment for the operator to place or nudge the bot and press Enter (§5 E).

**Safety layers:**

| Layer | Trigger | Action |
|---|---|---|
| Bot command watchdog | no frame that commands the drive (STOP, SEG, DRIVE inside a segment, RECENTRE) for 300 ms; PING and a DRIVE outside a segment do not count | drive 0 (analogWrite 128), EN LOW, segment closed (`s` → `wd_trips`) |
| Bot SysTick deadman | motors on and no drive-commanding frame for 350 ms, even with `loop()` stuck | EN LOW from the 1 ms SysTick interrupt; `loop()` closes the segment and counts `wd_trips` when it resumes (`s` → `deadman_cuts`) |
| Bot hardware watchdog | `loop()` makes no pass for ~0.5 s (0.43–0.64 s) | WDT early warning: EN LOW, the UF2 quick-boot flag set, then an immediate SAMD21 reset (a CLEAR write without the 0xA5 key; `i` → `rcause=0x20`, INFERRED). The WDT time-out at ~1 s (0.86–1.28 s) is the fallback when the early warning cannot run: its handler shares NVIC priority 0 with USB, IRremote's TC3 and any unexpected IRQ, so an ISR stuck there blocks both it and the SysTick deadman, and EN can stay HIGH until that reset, with no quick-boot flag (§8). A HardFault cuts EN at once, then waits for the early warning, sets the flag and resets, so a crash loop resets at most about twice a second. The flag is written only just before each reset, so it never swallows a RST tap. See the bootloader note below |
| Bot low battery | 3 consecutive mVbat readings < 3400 mV (on battery, under load or at rest); failed or implausible reads are skipped and counted in `pwr_err` | stop, latch, and answer every SEG, RECENTRE or PING with a red flash until reset or USB `r`. With I2C down (`i2c=0`) nothing can be read, so instead every SEG is answered with a red flash and no segment starts (I2C guard below) |
| PC marker loss | no detection for 0.25 s | STOP burst; segment marked `marker_loss`; wait for the marker, else operator pause |
| PC geofence | pose predicted `lookahead` ahead leaves the arena shrunk by `margin`. `lookahead` = camera latency 0.05 s (INFERRED) + 2 × blaster period + 0.1 s (0.37 s at 110 ms), plus the age of the latest pose. It is recorded as `geofence_lookahead_s` in `session.json`; `SafetyConfig.lookahead_s` overrides it | STOP burst; segment marked `geofence` (truncated) |
| PC re-centring stops | red flash (colour camera only); predicted pose (same lookahead) within `RecentreConfig.fence_m` = 3 cm of the arena edge while moving away from home; distance to home not down 2 cm in 4 s; distance up 3 cm in one straight run; 25 s timeout | STOP; event `recentre_lowbat` / `recentre_geofence` / `recentre_no_progress` / `recentre_diverging` / `recentre_timeout`, then the operator pause "re-centring failed (<reason>)", or the low-battery pause for a red flash. On a mono camera the `no_progress` pause adds a low-battery hint |
| PC low battery | red flash seen during a segment or during re-centring (colour camera only). If no LED sample was seen in the second before a SEG (LED patch outside the image), the segment's red check starts 0.6 s in, against the median of the last second: the blue sync flash lowers `led_r`, so the window's lowest sample is not a usable baseline | STOP; operator pause (charge or swap) |
| PC stall (mono camera only) | a segment of ≥ 1 s that flashed, with a motor at \|drive\| ≥ `SafetyConfig.stall_drive` (90), that never got `missed_motion_m` (3 mm) from its start nor turned `stall_turn_rad` (0.1 rad) | after the segment's STOP: status `stalled`, event `stalled`, the low-battery pause; the segment is retried like `lowbat` |
| PC exit or exception | anything | STOP to the bot (`stop_repeats` = 2) and a broadcast STOP (address 0xFF), each sent even if an earlier one fails. After a failure the bridge resyncs, because a Ctrl-C mid-reply leaves a stale reply. Resync sends CAN (0x18) and a newline, so the blaster drops a half-written request line unrun; a bare newline would have sent it. Opening the bridge resyncs too. The original exception (e.g. KeyboardInterrupt) propagates; `BridgeError` only if no final STOP was acknowledged |
| Blaster | none needed | it holds no state; if the PC stops, every bot stops by its own watchdog |

- **Bootloader (UNTESTED).** Every SAMD21 reset passes through the UF2 bootloader. A QT Py bootloader ≥ 3.11 drives D12 = EN HIGH while D2/D3 float, so the motors may run at full drive (about 3.3 V) until `setup()` runs. After a reset at the WDT early warning or after a HardFault, the quick-boot flag skips its 500 ms double-tap wait, so this should last only a few ms. The bootloader bundled with the core (v1.23.1-…-gb149075) leaves EN floating instead. Uploads, double-tap recovery, a WDT time-out (early warning blocked) and other resets get no quick-boot flag (§3 build notes, §5 B.9, §8).
- **I2C guard.** Wire in the core has no time-outs.
  - Before `Wire.begin()` the firmware frees a bus held low: while SDA reads low, 9 SCL clocks (whatever SDA reads during them), then a STOP, up to 3 times.
  - If a line stays low, Wire is not started (`i` → `i2c=0`): battery reads fail (`pwr_err` rises) and the magnetometer is not set up (`mag=0`), so the low-battery abort cannot trip. The bot then answers every SEG with a red flash and starts no segment: the PC pauses as for a low battery, and the operator power-cycles the bot. PING and RECENTRE are unaffected.
  - While the motors are off, the bot retries the recovery about once per second. Once the bus is free it starts Wire and the magnetometer, and SEG works again.
  - After `Wire.begin()`, the SERCOM's SCL low time-out (25–35 ms) ends a stuck transfer, so `loop()` keeps running. A transfer that fails partway (bus error, lost arbitration, time-out) is rejected through the SERCOM error flag instead of being read as 0xFF bytes.

**IR frame** (`calib_protocol.h`, protocol 2; one NEC-timed 32-bit frame, sent with IRremote `sendNECRaw`, read from `decodedRawData`):

| Byte | Content |
|---|---|
| 0 | bot ID 1..254 (0xFF = broadcast, STOP only; 0x00 is left to TV remotes) |
| 1 | op (3 high bits) and CRC-5 check bits (5 low bits) |
| 2 | L code = 128 + drive_L (drive −127..127; **128 = stop**; code 0 invalid) |
| 3 | R code = 128 + drive_R |

- The NEC inverse bytes are not used; a CRC-5 (poly 0x05, init 0x1F) replaces them. It runs over all 32 frame bits in on-air order (raw bit 0 first; the 5 check bits in byte 1 are chosen so that the register ends at 0). So it catches every single-bit error, every burst of up to 5 consecutive on-air bits, and every 2-bit error except raw bits 0 and 31 together (the first and last bit sent). It misses 1/16 of 6-bit bursts, 1/32 of longer ones and about 1/29 of 3- or 4-bit errors.
- Protocol 1 ran the CRC in another bit order than the air, so some short on-air bursts passed unseen (e.g. one that turned DRIVE into RECENTRE).
- IRremote's decoder does not reject marginal timings, so corrupted frames arrive as flipped bits: the CRC is what catches them.
- **Ops:**
  - 0 STOP;
  - 1 SEG: start a segment, flash, new log record;
  - 2 DRIVE: keep-alive or level change inside the open segment. Outside one it stops the motors (counted as `drive_no_seg`), so a lost SEG or a segment that already ended means no motion rather than unlogged motion;
  - 3 RECENTRE: drive, closes the segment, not logged;
  - 4 PING: flash only. It does not feed the command watchdog, and a run of PINGs shares one log record.
- **Bot ID:** FNV-1a over the SAMD21 128-bit serial, in the order of the USB iSerial string, folded to 1..254.
  - `register` reads it over USB and checks it against the Python hash.
  - With 254 values, collisions are likely in a fleet (about 16 % for 10 bots). `fleet-check` finds them; rebuild a colliding bot with `-DBOT_ID_OVERRIDE=<id>`.

**Drive scale:** `drive` is the signed scale of HARDWARE.md §4.4: `analogWrite(pin, 128 + drive)`, i.e. DRV2603 VOUT = drive/128 × VFS.
- In the 40 kHz mode the firmware writes the same duty (128 + drive)/256, so drive values mean the same voltage in both modes.
- The firmware caps |drive| at 120 (about 3.07 V on the motor; the rating is 2.7 V and the maximum 3.3 V).

## 2. Hardware and wiring (UNTESTED)

### IR blaster

| Part | Suggestion |
|---|---|
| Board | Arduino Nano (IR pin D3) or, better, a native-USB board: RP2040 or QT Py M0. A USB-UART chip adds 1–16 ms latency (FEASIBILITY.md §3.3). |
| Driver | Logic-level N-MOSFET (e.g. AO3400 or IRLML6344, which suit 3.3 V boards): gate ← 100 Ω ← `IR_LED_PIN`, 10 kΩ gate pull-down to GND |
| LEDs | 4–8× TSAL6200 (940 nm), each with its own series resistor from +5 V to the drain. About 20 Ω gives ~180 mA peak at IRremote's 30 % carrier duty (TSAL6200: 100 mA DC, 200 mA pulsed at ≤ 50 % duty; INFERRED, check the datasheet and the resistor power) |
| Supply | 470 µF bulk capacitor at the LEDs. Above 4 LEDs, use an external 5 V supply rather than USB. |
| Mounting | Overhead, pointing down. The V1.3 receiver is the lens-up TSOP75438TT; confirm that on V1.1 bots. Spread the LEDs so the arena corners still decode (link test, §5 B). |

### Bot

1. **Leg plate:** check for the Ø0.8 mm centre pin that protrudes about 1.1 mm below the leg tips, and remove it if present (HARDWARE.md §7, INFERRED print aid).
2. **Marker:** a cap with a 20–22 mm ArUco `DICT_4X4_50` marker (an ID the references do not use: they are IDs 0–3, or 0–4 with a fifth), printed and glued **with its top edge towards the bot's front**.
   - Leave the TSOP, the bot's IR LED and the **top NeoPixel uncovered**.
   - The NeoPixel must sit **outside the marker and its white border** (the tracker samples the LED there).
3. **Measure per bot** (calipers or a photo) and enter in the rig config:
   - `marker_offset`: marker centre relative to your body reference point, in the body frame, metres (forward, left);
   - `marker_yaw`: the marker's "up" direction (bottom edge → top edge of the printed marker) minus the body's forward direction; 0 for a marker glued with its top edge to the front;
   - `led_offset`: NeoPixel in the marker frame (forward, left);
   - `led_height`: LED height minus marker height, metres, negative when the LED sits below the marker surface (e.g. −0.010). Without it, the LED patch drifts toward the camera nadir by r·dh/(Hc − dh), several pixels near the walls. The tracker applies it when camera intrinsics are loaded (step C), or with `camera.camera_height_m`. `run` and `track` read `bots.<id>.led_height` and `camera.camera_height_m` from the rig config (both are in `rig_example.json`); intrinsics take precedence over `camera_height_m`.
   - The fitted map is the twist **of the reference point**. With offset 0 that is the marker centre; `geometry.transfer_twist` moves it to any other point.

### Camera and arena

- Rigid overhead mount.
  - A global-shutter camera at ≥ 60–120 fps with ≤ 2 ms exposure is needed for dynamics, noise and latency.
  - A 30 fps webcam is enough for the static map only.
  - Use flicker-free DC lighting.
  - A mono or NoIR camera needs a 940 nm-blocking filter, otherwise the blaster's reflections flicker in the images.
- **Prefer a colour camera.** The sync LED and the red low-battery signal need colour.
  - A mono camera (2-D frames, or B = G = R) is detected automatically. The tracker then logs the LED patch brightness in `led_b` ("luma" mode, `led_r` = NaN).
  - Red low-battery flashes then cannot be told from sync flashes, so the PC-side low-battery stop does not work; only the bot's own latch stops it.
  - `run` warns once at the start of the session (event `warning`, `session.json` → `warnings`).
  - A latched bot then shows up as re-centring without progress (the pause names the battery), or as a segment of at least 1 s with a motor at |drive| ≥ 90 that flashes but neither moves 3 mm nor turns 0.1 rad: status `stalled`, then the low-battery pause, retried like `lowbat` (§1).
  - Weaker segments between the latch and that point are logged `ok` with no motion. Drop them by `moved_m` in `segments.csv` (§8).
  - The OV9281 suggested in FEASIBILITY.md §6 is mono.
- **Reference markers (IDs 0–3, or more) on spacers at the height of the bot markers.** The homography is fitted at marker height, so there is no parallax error. Survey their centres in metres.
  - The coordinates must form a **right-handed** frame seen from above: x right, y up, ω > 0 = counter-clockwise.
  - The tracker refuses a mirrored layout, which would flip the sign of ω.
  - With exactly 4 references the homography is an exact fit, so the reprojection RMS cannot detect survey errors; it is reported as NaN ("exact 4-point fit").
  - The tracker instead checks each reference marker's side, aspect and diagonals against the others, and their size against `camera.reference_size_m`. It warns on gross errors (for example a 0.1 m typo), not on a 10 mm survey error.
  - **Use 5 or more references** (e.g. IDs 0–4): the reprojection RMS is then a real residual.
  - `run` and `track` print the tracker's `reference_summary()` (the RMS or "exact 4-point fit", and the worst reference shape), the LED signal mode (`chroma` or `luma`) and its warnings after the homography fit. Per-frame warnings are printed at the end: the last 5, with a count. A bumped reference warns on its first off frame, then every 300 frames (`REF_WARN_EVERY`, about 5 s at 60 fps).
- An arena of ≥ 1 m with foam walls. `arena.margin` (default 8 cm) defines the geofence.

## 3. Build and flash the firmware

```bash
arduino-cli config add board_manager.additional_urls https://raw.githubusercontent.com/GRASP-LAB/GRASPion/main/package_graspion_index.json
```
```bash
arduino-cli core update-index && arduino-cli core install graspion:samd arduino:avr
```
```bash
arduino-cli lib install "IRremote@4.7.1" "Adafruit NeoPixel"
```
```bash
arduino-cli compile --fqbn graspion:samd:graspionBody:opt=fastest,usbstack=tinyusb calibration/firmware/calib_bot
```
```bash
arduino-cli upload --fqbn graspion:samd:graspionBody:opt=fastest,usbstack=tinyusb -p COM7 calibration/firmware/calib_bot
```
```bash
arduino-cli compile --fqbn arduino:avr:nano calibration/blaster/ir_blaster
```

**Build options:**
- **PWM mode:** decide this before calibrating, because a map measured in one mode is invalid in the other.
  - The default `PWM_40KHZ=0` is the core's 732 Hz, used by all upstream examples.
  - For the in-spec 40 kHz TCC0 mode (HARDWARE.md §4.4), add `--build-property "compiler.cpp.extra_flags=-DPWM_40KHZ=1"`.
  - A/B-test 5 points on bot #1 first (FEASIBILITY.md §5.2 step 1).
- The other switches go into the same property, space-separated (e.g. `"compiler.cpp.extra_flags=-DPWM_40KHZ=1 -DBENCH_HANG=1"`):
  - `-DBOT_ID_OVERRIDE=<1..254>` resolves ID collisions.
  - `-DUSE_HW_WDT=0` disables the hardware watchdog. The SysTick deadman still cuts EN 350 ms after the last drive-commanding frame, but nothing resets a hung MCU and no quick-boot flag is set.
  - `-DBENCH_HANG=1` adds USB key `H` (drive both motors at 80, then hang `loop()`) for bench check B.9. Never use it for calibration runs; `i` shows `bench_hang=1`.
- `WATCHDOG_MS` (300) is a constant in `calib_bot.ino`. Raise it for a long blaster period (§5 B.4), then run `register` again.

**Build notes:**
- Press the bot's power button before uploading.
- **While the UF2 bootloader runs** (uploads, double-tap recovery, resets without the quick-boot flag), a QT Py bootloader ≥ 3.11 holds EN HIGH with floating PWM inputs, so the motors may run at full drive. Lift the bot or hold its legs off the surface. Bench check B.9 tells which bootloader a bot has.
- The blaster **must not** be flashed onto a bot: its QT Py build drives D12 (motor enable) HIGH at boot.
- The bot and the blaster must be built from the same `calib_protocol.h` (protocol 2: CRC in on-air bit order). Their `i` / `V` lines show `proto=`; a protocol-1 build rejects protocol-2 frames as CRC errors and vice versa.
  - `run` refuses a blaster whose `V` reports another protocol, and a bot whose `bots/<id>.json` does (a hand-written file without `proto` is not checked).
  - `register` refuses a bot whose `i` does and writes nothing.
  - Each error names the sketch to reflash.
- Flash blasters with ir_blaster/3 or later: the PC's resync relies on its CAN line-cancel (§1). An ir_blaster/2 build keeps the CAN byte in the line, so a stale frame request is rejected as `E parse` there instead of being dropped.
- `calib_bot` drives the MLX90393 directly over Wire and does **not** use the Adafruit MLX90393 library. That library pulls in Adafruit BusIO, whose `Adafruit_SPIDevice.cpp` fails to assemble at `opt=fastest` (also `opt=faster` and `opt=dragons`) on this core: the core's `delayMicroseconds` inline asm uses a `"+r"` constraint, and at `-O3` and above gcc can assign it a high register that Thumb-1 (Cortex-M0+) rejects. So **any sketch using the Adafruit MLX90393 library, including all three upstream examples, fails to build with `opt=fastest`**; build those with `opt=small` (HARDWARE.md §9.1).
- On Windows, gcc fails with "cannot execute cc1plus" when the arduino-cli data directory sits under a very long path (MAX_PATH). Use the default `%LOCALAPPDATA%\Arduino15`, a short folder such as `C:\Users\<you>\Arduino15`, or a short `subst` drive letter.
- **Installing from inside the Claude desktop app:** its shells have `AppData\Local` writes redirected into the app's private storage, so a default install there is invisible outside the app. Put the data directory in `C:\Users\<you>\Arduino15` and point the user variable `ARDUINO_CONFIG_FILE` at `C:\Users\<you>\Arduino15\arduino-cli.yaml` (setup in HARDWARE.md §9.1).
- **graspion:samd 1.0.1** (current since at least 2026-10-27; 1.0.0 before) defines `USE_NO_SEND_PWM` for every `graspionBody` build. `calib_bot` sends no IR, so this changes nothing for it; `bot_blaster` defines it too and only gets a "redefined" warning (HARDWARE.md §9.1).
- After changing `calib_protocol.h`, copy it to `blaster/ir_blaster/` and run `python -m graspion_calib.protocol --write-vectors`. The firmware then refuses to compile if C and Python disagree.

**Bot USB commands** (115200 baud; any terminal works, and `register` and `dump` use them):

| Key | Action |
|---|---|
| `i` | info line: firmware (`fw=calib_bot/2`), `proto`, ID, serial, PWM mode, watchdog, `hw_wdt`, `bench_hang`, reset cause (`rcause`; 0x20 = WDT), `mag`, `i2c` (0 = an I2C line has been held low since boot: battery and magnetometer are not read, `pwr_err` rises, every SEG gets a red flash, and the bot retries about once per second), battery (`mvsys`, `mvbat`: the last plausible reading), `raw` (the last AVR words before the plausibility bounds, bytes 0–1 then 2–3 as 8 hex digits, e.g. `raw=13880E74` = 5000 / 3700 mV; `00000000` until a read without a bus error), `lowbat`, number of log records |
| `d` | dump the log as CSV |
| `c` | clear the log and statistics (including `deadman_cuts`) |
| `s` | statistics: frames OK, CRC errors, other addresses, `drive_no_seg`, watchdog trips, `deadman_cuts`, `pwr_err`, `mag_err`, … |
| `r` | clear the low-battery latch |
| `H` | only with `-DBENCH_HANG=1`: drive both motors at 80, then hang `loop()` (bench check B.9) |

All replies are non-blocking: they are written as the USB FIFO has room, one command at a time (the next command is read only after the previous reply has gone out). A terminal that stops reading cannot stall the firmware, and pending output is dropped when the port closes.

The log holds 400 records in RAM and is lost when the bot loses power. A run of PINGs (link test) shares one record (`n_frames` = count, `t_start_ms`/`t_end_ms` = first/last PING). Clear it (`c`) after the bench checks and before every session, and dump it into the session folder after every session and every battery (§5 F).

## 4. PC setup

```bash
pip install -r calibration/pc/requirements.txt
```

Run the commands below from `calibration/pc`.

```bash
python -m graspion_calib selftest
```

`selftest` runs the offline suite (134 tests) in about 2 min. One test is skipped by default: it checks the twist fit against your own simulator class. To run it, set `GRASPION_TILEMAT_SIM=<tilemat_control_stack>/robots/elliptical_bot/controllers/mpc/acados_simple`. That environment needs a working matplotlib, because `Ellipse2dObj.py` imports it; otherwise the test stays skipped.

A dry run of the whole loop on the simulated rig:

```bash
python -m graspion_calib schedule grid --blocks 1 --out sessions/grid1.json
```
```bash
python -m graspion_calib run --sim --bot 90 --schedule sessions/grid1.json --out sessions/sim_grid
```
```bash
python -m graspion_calib analyze sessions/sim_grid --out analysis/sim
```

`schedule grid`, `prbs` and `validation` draw a fresh random seed each time. The seed is printed and recorded in the schedule JSON (`meta.seed`) and in `session.json` (`schedule_meta`); pass `--seed N` to reproduce a schedule.

## 5. Run procedure (FEASIBILITY.md §5.2)

**A. Register each bot** (USB, bot powered):
```bash
python -m graspion_calib register --port COM7
```
```bash
python -m graspion_calib fleet-check
```
Record the bot ID ↔ marker ID ↔ chip serial in `bots/` and in the rig config. `register` refuses a bot built with another IR protocol version (reflash `calib_bot`, then register again).

**B. Bench checks.** Do every one of these before the first map; none has been done yet. Start with 9 (bootloader), with the bot on a stand.

The serial-terminal checks (2, 3) talk to the blaster at 115200 baud, one command per line. The blaster drops any line over 47 characters whole and answers `E long`; an empty line gets no reply. A CAN byte (Ctrl-X, 0x18) drops the line typed so far, with no reply; the PC sends it to resync. `G <ms>` sets the period, 100–2000 ms.

1. **Battery bytes:** compare `i` → `mvbat` against a multimeter on the cell (the AVR byte meaning is INFERRED). If `mvbat` stays 0 or looks wrong, read `raw=SSSSBBBB`: the AVR's bytes 0–1 and 2–3 as sent, before the firmware's plausibility bounds. For example, a byte-swapped 3700 mV reads `740E` instead of `0E74`. Check `mag=1` and `i2c=1`.
2. **Watchdog:**
   - Send `X <id> 1 80 80` from a serial terminal on the blaster, then nothing. The motors must stop within about 300 ms (`s` → `wd_trips` increments).
   - PINGs must not keep it alive: paste `X <id> 1 80 80` and three `X <id> 4 0 0` lines in one go (under 64 bytes for a Nano; the blaster sends them one period apart). On a scope or in the video, the motors must stop about 300 ms after the SEG, not 300 ms after the last PING.
   - `X <id> 3 80 80` (RECENTRE) followed by `X <id> 2 80 80` (DRIVE without a segment) must stop the motors at once (`s` → `drive_no_seg`).
3. **Address filter and CRC:**
   - `X <other id> 1 80 80` must not move the bot;
   - `f <frame with one flipped bit>` must not move it (`s` → `crc_err`);
   - `X 255 0 0 0` (broadcast STOP) stops it.
4. **Link test:** with the camera running,
   ```bash
   python -m graspion_calib schedule ping --long-s 30 --out sessions/ping.json
   ```
   then `run` it, with the bot at the centre and at each corner. `analyze` reports `link_tests` (flash loss and frames/s).
   - A ping run starts with 3 anchor PINGs, each followed by 0.8 s of silence, then PINGs at the blaster period. In a back-to-back train, a camera offset above the period cannot be told from one a period shorter when the first or last PING is lost; the isolated anchors settle it. If the lag is still ambiguous, `link_tests[].offset_ambiguous` is true, with a warning.
   - Flash loss is counted one-to-one: a lost PING does not borrow the next PING's flash. Flash detection uses a rolling dark-level baseline, so it works with the LED lit about 2/3 of the time at the 110 ms period.
   - For the bot's own count, clear the log (`c`) before each position and read the PING record's `n_frames` in a dump afterwards (consecutive ping runs share one record).
   - If frames are lost at the default 110 ms period (e.g. the TSOP's AGC suppresses back-to-back frames), lengthen the blaster period (`bridge.period_ms`, 100–2000 ms) until loss is low. A longer period lengthens the geofence lookahead automatically (§1).
   - Then raise `WATCHDOG_MS` in the firmware to at least 2.5 periods, rebuild, and run `register` again so `bots/<id>.json` is updated. `run` refuses a driving schedule when `bots/<id>.json` has `watchdog_ms` < 2.5 × `period_ms`. Ping and static schedules are exempt, so the link test can run at any period.
5. **LED sync:** check the flash rise in `tracker.csv` (`led_b`: blue chroma on a colour camera, brightness on a mono camera). It must rise by more than 40, the flash threshold. Lower the exposure if the LED clips.
6. **Static noise:**
   ```bash
   python -m graspion_calib schedule static --long-s 60 --out sessions/static.json
   ```
   This gives the measurement-noise floor.
7. **Re-centring:** tune `RecentreConfig` (`scheduler.py`): pivot and straight drive pairs, `latency_s`. The defaults are guesses; equal drive drifts clockwise on the twin's bot.
   - Wrong guesses or a wrong `marker_yaw` stop re-centring within a few seconds instead of after 25 s. Look in `events.csv` for `recentre_diverging` (a sign error, e.g. `marker_yaw` off by π), `recentre_no_progress` (e.g. swapped pivot pairs) or `recentre_geofence`.
   - Their thresholds are `fence_m`, `progress_m`, `progress_s` and `diverge_m`. In the simulator, 145 random-start re-centrings in a 1 m arena never tripped them.
8. **PWM A/B test** on bot #1 if not decided.
9. **Bootloader and EN across resets** (bot on a stand):
   1. Identify the bootloader: double-tap RST (X2 pin 10 to GND) and read `INFO_UF2.TXT` on the QTPY_BOOT drive.
   2. Scope EN (D12) and one motor output.
      - Build with `-DBENCH_HANG=1` and send `H`. EN must fall about 350 ms after `H` (deadman). The MCU resets at the WDT early warning, about 0.5 s (0.43–0.64 s) after `H`; then `i` should show `rcause=0x20` (INFERRED for this immediate reset; note any other value), and EN may pulse HIGH only for a few ms in the bootloader.
      - Repeat with `-DBENCH_HANG=1 -DUSE_HW_WDT=0`: EN falls at about 350 ms and stays LOW. The MCU stays hung (no USB) until you reset it.
      - Watch EN through an upload, and through a double-tap.
   3. If EN is HIGH for about 0.5 s after a reset, or for the whole upload, the bot carries a ≥ 3.11 bootloader (open item, §8).
   4. Reflash without `BENCH_HANG`.
10. **I2C time-out:**
    - With the bot powered, short SCL (HEAD X3 pin 9 or SWD X2 pin 6) to GND for about 1 s. The bot must keep answering `s`, with `pwr_err` and `mag_err` rising and no reset (`rcause` unchanged).
    - Then power-cycle with SCL held low: `setup()` must finish, USB must enumerate, `i` must show `i2c=0`, and a SEG (`X <id> 1 80 80`) must get a red flash and no motion. Release SCL: within about 1 s `i` must show `i2c=1` and `mag=1`, and a SEG must drive again. The AVR shares this bus; how it reacts is UNKNOWN.
11. **Motor identity and turn direction** (once per bot, before the first map). HARDWARE.md §4.4's kinematics come from the paper and are not VERIFIED, and V1.3 wires the right motor pair with reversed polarity.
    ```bash
    python -m graspion_calib schedule circles --pairs "[[100,0],[0,100]]" --long-s 5 --out sessions/turns.json
    ```
    Then `run` it with the camera recording.
    - Left only (`[100, 0]`, D3) must turn the bot clockwise seen from above, so `theta` in `tracker.csv` falls. Right only (D2) must turn it counter-clockwise, so `theta` rises. Both motors must run.
    - If the video and `theta` disagree, the frame is mirrored or `marker_yaw` is wrong. If both show the opposite turn, the motors are swapped or seated wrongly, or §4.4 is wrong. Stop, record it, and resolve it before mapping.

**C. Camera intrinsics:**
```bash
python -m graspion_calib camera-calib --make-board board.png
```
Print the board and measure the squares. Take 15–30 views, then:
```bash
python -m graspion_calib camera-calib --images views/ --square-m <measured> --out camera.npz
```

**D. Rig config:** copy `pc/configs/rig_example.json` and fill it in:
- reference marker positions and `camera.reference_size_m`;
- arena;
- camera settings and intrinsics, and `camera.camera_height_m` (camera above the marker plane; used for the LED-height correction when there are no intrinsics);
- blaster port and period;
- per-bot geometry, including `led_height` (§2).

**E. Trial runs and manual mode.** Before the first full grid, run single cells with yourself as the re-centring loop:
```bash
python -m graspion_calib schedule cells --cells "[[80, 80], [0, 90], [90, 0]]" --out sessions/trial.json
```
```bash
python -m graspion_calib run --rig rig.json --bot 90 --schedule sessions/trial.json --out sessions/bot90_trial --recentre manual
```
- `schedule cells` runs the listed (L, R) pairs in the given order, `--blocks N` times each (default 1), with the grid's drive and rest times and the tag `cells`. The session is logged and analysed like any other, so trial cells can go into the same `analyze` call as a grid, or be kept apart.
- `--recentre manual` replaces the bang-bang re-centring. Before every segment the session sends STOP and waits, showing the bot's position, its distance from the arena centre and its heading. Place the bot by hand, or nudge it with `w` / `a` / `d` (or the arrow keys): each key press is one `RECENTRE` burst of `RecentreConfig.nudge_s` (0.6 s) with the re-centring drive pairs (`straight`, `pivot_ccw`, `pivot_cw`), ended by STOP and cut short by marker loss, by the arena fence while the bot moves away from home, or by a red flash (then the low-battery pause). `Enter` runs the segment from where the bot stands, under the segment's usual geofence and marker-loss stops; `x` skips it (event `skipped`, nothing logged in `segments.csv`); `q` ends the session (the final STOPs go out as always); `space` sends an extra STOP. The marker may be hidden during the pause (your hand): the status line says "marker not visible", nothing prompts, a nudge is refused, and the segment waits for the marker before it starts.
- `--only "0-3,7,12-"` runs a selection of a schedule's segments (indices as in the schedule file, in the order given), e.g. one cell of a grid again. The session's `schedule.json` and its segment numbers are the selection; `session.json` → `only` records it.
- The bot's command watchdog covers manual mode like everything else: a nudge is frames at the blaster period followed by STOP, and nothing drives while you hold the bot. The keys are read from the console without Enter (`msvcrt` on Windows, cbreak mode elsewhere); `--sim` has no operator and runs every segment at once.
- Count on more time than `schedule`'s estimate, which assumes about 4 s of automatic re-centring per segment: every pause is a handling point, so a full grid run by hand takes roughly twice as long. The same mode runs the few-cell subsets of a held-out bot.

**F. Static map:**
```bash
python -m graspion_calib schedule grid --blocks 5 --out sessions/grid.json
```
```bash
python -m graspion_calib run --rig rig.json --bot 90 --schedule sessions/grid.json --out sessions/bot90_grid_a
```
- The grid is L, R ∈ {0, 55, 60, 70, 80, 90, 100, 110, 120}: 80 cells, plus 8 reverse-polarity checks, in 5 randomised blocks of 3 s segments. That is about 23 min per 2 blocks, so expect more than one battery.
- Each `schedule` call draws a fresh order seed (§4). Generate a new schedule per bot and per battery rather than reusing one file.
- Clear the bot log (`c`) before the session. After it, and after each battery, dump the log into the session folder:
  ```bash
  python -m graspion_calib dump --port COM7 --out sessions/bot90_grid_a/bot_dump.csv
  ```
  Then clear it again.
- The analysis tolerates leftover records, an earlier session in the same dump, a missed first SEG and a bot reset mid-session. It warns when fewer than 80 % of SEGs match (`calib.json` → `bot_log_merge[].warning` and `warnings`). It leaves an ambiguous alignment unmatched, e.g. the same schedule with identical timing twice in one dump.
- To continue on a new battery, split the schedule's `segments` list; one folder per battery is fine, because `analyze` pools folders.

**G. Dynamics:** `schedule steps` and `schedule prbs` (switching every 0.1–0.5 s).

**H. Noise:** `schedule circles --pairs "[[100,60],[60,100]]" --long-s 60` gives CW and CCW circles of R ≈ 5–15 cm. Pick pairs from the static map.

**I. Validation:**
```bash
python -m graspion_calib schedule validation --forward-map analysis/bot90/forward_map.json --out sessions/val.json
```
The targets are in (v, ω); see §6. Options:
- `--forward-map` is required.
- `--p-min DRIVE [DRIVE]`: the lowest drive of the operating region, one value for both motors or two (L R). The default is per motor max(60, p0 + 5), the same as `analyze`. A value inside the dead zone exits with a message.
- `--v-bat V`: the battery voltage at which a `bat` map is inverted. The default is its median training voltage; other models ignore it.

The options are recorded in the schedule's `meta.args`.

**Scale:** repeat for ≥ 5 bots (8–10 better), plus one bot on 3 different days.

## 6. Analysis

```bash
python -m graspion_calib analyze sessions/bot90_grid_a sessions/bot90_grid_b sessions/bot90_steps --out analysis/bot90 [--gp]
```

**Pipeline** (`report.py`):

1. **Time alignment.** Every SEG and PING flash gives `cam_offset` = flash onset − blaster acknowledgement, corrected by half a frame (IR decode + camera latency). A command acknowledged at `t_ack` acts at `t_ack + cam_offset` in camera time.
   - Onsets are found against a rolling low-percentile (dark-level) baseline.
   - Each SEG or PING acknowledgement is matched one-to-one to an onset inside a gate centred on the session's lag. Segments whose flash was not found use the session median.
   - A session with no flash at all uses the `cam_offset` measured in the other analysed sessions, never a hard-coded value.
   - If none was measured anywhere, times stay relative to the acknowledgement and `calib.json` sets `dead_time_includes_cam_offset` = true. The dynamics `dead_time` is then the total delay (dead time + IR decode + camera latency), and its fit range widens to 0.6 s.
   - `latency[]` records `cam_offset_used`, `cam_offset_source` (`measured` / `other_sessions` / `none`), any `warning`, and `ambiguous`: a lag that matched as many frames as one a frame spacing away (a ping train without anchors whose first or last frame was lost). That session keeps its own offset but does not lend it to sessions without a flash.
2. **Per-segment twist** (`twist.fit_twist`).
   - Skips the first 0.5 s. The window ends at the STOP, or earlier if the bot visibly stopped, or where the bot log says its command watchdog or low-battery latch ended the segment.
   - A visible stop needs the bot to stay still for more than 0.3 s after its last moving sample, measured against the segment's highest 0.2 s median speed. Dropped camera frames do not count as a stop.
   - Heading spikes (e.g. a π flip) are removed before every unwrap: twist fit, velocity series, lag fit, heading diffusion and static noise.
   - ω comes from a line fit; (u1, u2) from one linear least-squares problem that uses the measured heading, so heading diffusion does not bias it. ICR = (−u2/ω, u1/ω) in the body frame.
   - Segments that never moved (`missed`), hit the low-battery latch (`lowbat`) or were judged `stalled` are excluded.
3. **Bot log merge** (if `bot_dump.csv` exists): per-segment mVbat at rest and under load, magnetometer at rest and under load (the offset map b(L, R) for Phase 2), and the bot clock rate.
   - Matching is a chained alignment by (L, R) and consistent spacing (see the `botlog.py` docstring).
   - `V_bat`, the `bat` regressor, is the rest voltage `mv_rest` read just before the drive is applied. The under-load mean sags with the drive itself; `mv_load_*` stay in `segments_fit.csv` as diagnostics.
4. **Forward map** (`forward_map.py`). Per motor f = max(0, |p| − p0)^γ; the twist is 0 when both motors are in the dead zone. Variants:
   - `lin`: twist linear in (1, f_L, f_R);
   - `int`: + f_L·f_R;
   - `cor`: u1 = v(f) + d·|ω̂|, the EllipticalBot2D / digital-twin kinematics;
   - `bat`: + battery terms (rest voltage), when the bot log is present. A `bat` map is evaluated at its median training voltage (`forward_map.meta.v_bat_median`) by the inverse map and the lag fit.

   Selection is by grouped cross-validation (folds of whole drive cells, so the score measures interpolation to unseen drive levels). The winner is refitted on all data. Cell-bootstrap draws go to `forward_map_bootstrap.json`.

   **Only the mean CoR offset d is identifiable** from steady-state twists. Any CW/CCW asymmetry is exactly absorbed by the linear v(f), so use `dist_rotPoint_to_CoM = [d, d]` with the fitted v(f).
5. **Inverse map and feasible set** (`inverse_map.py`): closed form f = A⁻¹([v, ω] − c), then p = p0 + f^(1/γ).
   - The polytope H·[v, ω] ≤ h is the image of both motors in [p_min, 120] (an exact parallelogram for `lin`/`cor`), for MPC constraints and RL action projection. By default p_min = max(60, p0 + 5) per motor, so a bot whose dead zone reaches 60 still gets an inverse. `inverse_map.json` records `p_min` and `p_min_auto`.
   - Single-motor pivots lie outside it and are listed separately.
   - `v` = u1 for `lin`, v = u1 − d|ω| for `cor`.
   - A C header with the closed-form inverse is written for onboard use (UNTESTED).
6. **Lag and dead time** (`dynamics.py`): first-order lag τ plus dead time d after decode, pooled over step/PRBS segments (else the start/stop transients). The model is passed through the same Savitzky–Golay filter as the measured velocity, so the derivative filter does not bias τ. Segments the bot abandoned early (bot log: watchdog or low battery; or a visible stop) are excluded; their count is `dynamics_skipped_abandoned` in `calib.json`.
   - For segments with several commanded levels (steps, PRBS), a visible stop counts only during levels that the fitted forward map expects to move the bot at least half as fast as the segment's fastest level. A step down to a slow or dead-zone level is therefore not taken for a stop.
   - The joint (u1, ω) fit weights each output by its own residual variance (profile likelihood). So ω, whose derivative carries the heading diffusion, does not dominate it.
   - A level change or STOP the bot missed (IR loss) takes effect at a repeat, one blaster period later. The fit re-times such a change to the repeat (or to never) when that clearly fits better, then refits. The number of re-timed changes is `late_changes` in `calib.json` → `dynamics`.
   - Without the re-timing, 10 % frame loss inflated τ from 58 to 71 ± 12 ms in simulation; with it, τ = 59 ± 4 ms.
7. **Noise** (`noise.py`):
   - heading diffusion from second differences of the heading, which cancel any constant ω exactly, so short segments with different ω pool without bias; an OU fit for long runs. Drive runs stop where the commanded drive stopped acting (bot-log trip time, or 0.3 s before a visible stop), so the constant-ω premise holds;
   - measurement noise from static runs (with `n_spikes`, the heading spikes dropped);
   - lateral slip u2 over all segments;
   - per steady circle run (`noise.circle_jitter`): speed jitter and lateral slip, the SD of the body velocity (0.2 s Savitzky–Golay window) around the twist fitted to the same window.

**Outputs** in `--out`:
- `segments_fit.csv`
- `forward_map.json`
- `forward_map_bootstrap.json`: the cell-bootstrap draws
- `inverse_map.json`
- `graspion_map.h`
- `calib.json`: the summary: latency, link tests, bot-log merge, forward map, polarity check, inverse map with the polytope, lag and dead time, noise, ICR, warnings. The bootstrap draws are not in it; they stay in `forward_map_bootstrap.json`.

**On a simulated rig with known truth** (simulated, 20 seed pairs): each pair is 2 grid blocks plus 8 step segments, as in the `test_pipeline` fixture, with grid schedule and world seed a = 1, 3, …, 39 and steps world seed a + 1. The pipeline recovered (mean ± SD, range in brackets):

| Quantity | Recovered | Truth |
|---|---|---|
| p0 | 52.0 ± 0.2 / 50.1 ± 0.6 | 52 / 50 |
| γ | 0.647 ± 0.009 / 0.601 ± 0.017 | 0.65 / 0.60 |
| CoR offset d | 15.38 ± 0.10 mm | 15.3 mm (mean) |
| Dead time | 30.6 ± 2.5 ms [26.4–36.1] | 30 ms |
| τ | 57.8 ± 2.1 ms [53.7–61.4] | 60 ms |
| D_r | 0.0100 ± 0.0003 rad²/s [0.0095–0.0109] | 0.010 rad²/s |
| Selected model | `cor` in 20 of 20 | — |

- The lag fit scatters by a few ms from seed to seed (8 step segments; frame quantisation ±8 ms at 60 fps).
- The 1–2 ms left against truth comes from the simulator's 2 ms physics step: with a 0.25 ms step the fit recovers a dead time of 29.8 ± 3.2 ms and τ = 58.7 ± 3.0 ms.
- This validates the code path, not the robot.

**Feeding `rl_arch_bench`** (FEASIBILITY.md §5.4):
- project actions onto `polytope`;
- (L, R) = inverse map;
- twist from `forward_map` (or a draw from `forward_map_bootstrap.json` per episode);
- lag τ and dead time + `cam_offset` + bridge time as the k-step delay;
- `D_r` and measurement noise;
- integrate at the reference point.

## 7. Session folder format

| File | Written by | Columns / content |
|---|---|---|
| `session.json` | scheduler | bot, geometry, arena, safety and re-centring settings, rig, PWM mode, protocol, counts, `period_s`, `geofence_lookahead_s`, `schedule_meta`; `led_mode` (`chroma` / `luma`, resolved); `warnings` (session warnings, e.g. mono camera). On the rig only: `homography_rms_m`, `ref_check`, `reference_summary` and `tracker_warnings` (the tracker's warnings at the homography fit) |
| `schedule.json` | `run` | the schedule as run: `{"meta": {kind, seed, created, args}, "segments": [...]}`. Older bare-list schedule files still load |
| `tracker.csv` | tracker | `frame, t, id, x, y, theta, led_b, led_r, side_m, n_ref`. Host `perf_counter` time at frame read; marker pose in world metres (`theta` = the marker's "up" direction); LED blue and red chroma (B−(G+R)/2, R−(G+B)/2) on a colour camera, or brightness and NaN in mono "luma" mode |
| `frames.csv` | bridge | `t_req, t_ack, raw, op, bot, L, R, seg, purpose`: every frame sent |
| `segments.csv` | scheduler | `seg, attempt, kind, tag, block, L, R, levels, duration_s, rest_s, t_seg_req, t_seg_ack, t_end_ack, status, flash_live, moved_m, x0, y0, th0, n_frames` |
| `events.csv` | scheduler | re-centring (`recentred`, `recentre_lowbat`, `recentre_geofence`, `recentre_no_progress`, `recentre_diverging`, `recentre_timeout`), marker loss, retries, pauses (`lowbat`, `stalled`), `warning`, aborts, `final_stop_error` |
| `bot_dump.csv` | `dump` (copy it in) | `# I …` info (with `i2c`, `raw` and `bench_hang`), `# S …` stats (with `deadman_cuts`), then `kind, t_start_ms, t_end_ms, L, R, end_reason, n_frames, n_changes, flags, mv_rest, mv_load_min, mv_load_mean, n_mag, mag_rest_xyz, mag_load_xyz`. Kind 1 = a SEG; kind 2 = a run of PINGs (`n_frames` = number of PINGs, `t_start_ms`/`t_end_ms` = first/last) |

Status values: `ok`, `missed` (no flash and no motion: the SEG was lost; retried once), `marker_loss`, `geofence`, `lowbat`, `stalled` (mono camera: a strong segment that flashed but neither moved nor turned, presumably the low-battery latch; excluded from analysis like `lowbat`).

## 8. Known limitations and open items

- **UNTESTED on hardware:** firmware, blaster, IR link rate and range, tracker with a real camera, re-centring gains, the manual mode's console key reader and nudges, PWM 40 kHz snippet, hardware watchdog clocking, the SysTick deadman, the WDT early warning, its immediate reset and `rcause`, the quick-boot flag, the I2C time-out, bus recovery and its once-per-second retry, the blaster's CAN line-cancel, AVR battery byte meaning, MLX90393 register settings, NeoPixel timing next to IR receive.
- **Offline tests model the C code.** The firmware and blaster tests run Python copies of their logic (`sim.SimFirmware`, `tests/simrig.FakeBlasterSerial`); `calib_bot.ino` and `ir_blaster.ino` are covered only by compiling them with the `static_assert` protocol vectors. A change to either sketch must be mirrored in those models by hand. The deadman, the hardware watchdog, the I2C guard and the bot's USB replies have no offline test, and `SimFirmware.i2c_ok` has no test yet.
- **WDT early warning priority:** its handler runs at NVIC priority 0, which USB (TinyUSB), IRremote's TC3 and any IRQ left at its reset default share, and the M0+ never preempts at equal priority. An ISR stuck there (or in `Dummy_Handler`) blocks the early warning and the SysTick deadman: EN can stay HIGH until the WDT time-out at ~1 s (0.86–1.28 s), and that reset sets no quick-boot flag. Lowering those IRQs is untested with TinyUSB and IRremote, so it is left as is.
- **Bootloader:** a QT Py UF2 bootloader ≥ 3.11 drives EN HIGH with D2/D3 floating whenever it runs. The firmware shortens only resets at the WDT early warning and after a HardFault (quick-boot flag); uploads, double-tap recovery, a WDT time-out and other resets keep the full EN-HIGH window. Fix: flash a GRASPion build of uf2-samdx1 without `BOARD_NEOPIXEL_POWERPIN`. That is a bootloader rebuild outside this repository (not done). Which bootloader the bots carry is UNKNOWN (§5 B.9).
- **I2C down** (`i2c=0`): the bot refuses SEG but still obeys RECENTRE, with no battery check. The PC sees the refusal as a red flash (colour camera); on a mono camera it looks like the low-battery latch (`stalled`, or `ok` with no motion, §2).
- An NEC-timed frame without inverse bytes takes 49.84 ms + 1.12 ms per 1 bit: 53–82 ms for valid frames (68 ms only for 16 one bits), at most 86 ms for any 32 bits. The bot's receiver ends a frame after 8 ms of silence, so the blaster's period range is 100–2000 ms. The PC waits up to max(1 s, period + 0.59 s) for each reply. Link-rate estimates should use the per-frame length, not 68 ms.
- The TSOP75438's AGC4 burst rules may limit the link to about 5 frames/s (FEASIBILITY.md §3.2); the link test measures this. Below about 7 frames/s, the 300 ms watchdog will trip on single lost frames.
- The marker-loss blind time grows with the blaster period: the bot moves for `marker_loss_s` plus the stop latency before the STOP lands, about 0.62 s at 110 ms and 1.2 s at 400 ms. Only the geofence lookahead scales with the period; at long periods, shorten `SafetyConfig.marker_loss_s` (`scheduler.py`) or widen `arena.margin` (rig config) by hand.
- **MLX90393:** the RES 2 range is ±22000 LSB (datasheet Table 21), about ±13 mT on X/Y and ±21 mT on Z. `F_MAG_SAT` flags a record whose rest sample (`mag_rest`), or any sample averaged into `mag_load`, came within 5 % of it (INFERRED). Check the flag in the first dumps.
- **Camera:**
  - Mono cameras lose the red low-battery signal (§2). The PC then notices a latched bot only when re-centring makes no progress or through the stall check, and the stall check judges only segments of ≥ 1 s driven at |drive| ≥ 90. Weaker segments run after the latch are logged `ok` with no motion, the bot log has no record for them, and `analyze` does not drop them yet: remove them by `moved_m` in `segments.csv`. `run` does not poll the bot's low-battery flag over USB.
  - The stall check runs on mono cameras only. On a colour camera, a strong segment that flashes but does not move (e.g. a motor or EN fault) is still logged `ok`.
  - The reference survey check catches gross errors only.
  - Tracker warnings raised after the homography fit (e.g. a bumped reference) are printed when `run` exits, but only those from the fit are stored in `session.json` (`tracker_warnings`).
- Without a bot log, a bot that stopped before the twist window starts (0.5 s after the SEG) does not look like a visible stop. Dump the log after every session (§5 F).
- The bot log lives in RAM and is lost on power loss. Persisting it to the 2 MB SPI flash is a possible follow-up.
- The scheduler drives one bot at a time. A shared IR channel divides the frame rate by N.
- Magnetometer axes relative to the body are UNKNOWN (HARDWARE.md §4.6); the log stores raw sensor axes in µT.
- Not implemented here: the hierarchical robot/day/segment variance decomposition (FEASIBILITY.md §5.3), online RLS (§5.5) and active learning. They need multi-bot data first.
