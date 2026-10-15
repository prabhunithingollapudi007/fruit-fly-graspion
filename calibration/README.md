# Phase 0 calibration ("learning to walk")

Tooling to measure each GRASPion's map from motor drive (L, R) to body motion (u1, u2, ω), plus its lag, dead time and noise. This is [FEASIBILITY.md](../FEASIBILITY.md) §5.2–5.4 implemented: an overhead camera tracks an ArUco marker, a USB-serial IR blaster sends addressed frames, and calibration firmware on the bot drives segments under a command watchdog.

> **Status: nothing here has run on a bot, a blaster or a real camera. Treat all of it as UNTESTED.**
>
> What *was* verified (2026-10-15):
> - **Compiles** with arduino-cli 1.5.2-rc.1:
>   - `calib_bot` for `graspion:samd:graspionBody:opt=fastest,usbstack=tinyusb` (graspion:samd 1.0.0) in both PWM modes: 61 KB flash, ~21 KB static RAM of 32 KB.
>   - `ir_blaster` for `arduino:avr:nano` (avr 1.8.8) and `graspion:samd:adafruit_qtpy_m0`.
>   - Libraries: IRremote 4.7.1, Adafruit NeoPixel 1.15.5.
> - **55 offline tests pass** (Python 3.12, numpy 2.3.5, scipy 1.15.2, OpenCV 4.13). They cover:
>   - protocol and CRC, cross-checked against the C header through compile-time `static_assert` vectors on both ARM and AVR;
>   - the twist/ICR fit on EllipticalBot2D trajectories, including the user's own `Ellipse2dObj.EllipticalBot2D`, imported read-only;
>   - tracking on rendered scenes and ChArUco intrinsics on rendered views;
>   - every scheduler and firmware safety path, against a simulated rig;
>   - a full simulated calibration (schedule → scheduler → tracker CSV + bot log → analysis), compared against the known truth.
>
> Labels follow [HARDWARE.md](../HARDWARE.md): **UNTESTED** = written but never run on hardware, **INFERRED** = reasoned, not measured.

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
      scheduler.py  segment runner: re-centring, keep-alives, PC-side stops
      schedules.py  grid / steps / PRBS / circles / ping / static / validation schedules
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
3. **Drive:** the PC sends `DRIVE(L, R)` keep-alives at the blaster period (110 ms), and level changes for step/PRBS segments.
4. **Stop:** after the segment duration it sends STOP.
5. **Re-centre:** between segments a bang-bang law (pivot, then straight) brings the bot back to the arena centre with `RECENTRE` frames.

**Safety layers:**

| Layer | Trigger | Action |
|---|---|---|
| Bot command watchdog | no valid frame for this bot for 300 ms | drive 0 (analogWrite 128), EN LOW, segment closed |
| Bot hardware watchdog | `loop()` stalls about 1 s (e.g. a stuck I2C bus) | SAMD21 reset; boot leaves the motors off |
| Bot low battery | 3 consecutive mVbat readings < 3400 mV (on battery, under load or at rest) | stop, latch, and answer every frame with a red flash until reset or USB `r` |
| PC marker loss | no detection for 0.25 s | STOP burst; segment marked `marker_loss`; wait for the marker, else operator pause |
| PC geofence | pose predicted 0.3 s ahead leaves the arena shrunk by `margin` | STOP burst; segment marked `geofence` (truncated) |
| PC low battery | red flash seen | STOP; operator pause (charge or swap) |
| PC exit or exception | anything | STOP to the bot, then a broadcast STOP (address 0xFF) |
| Blaster | none needed | it holds no state; if the PC stops, every bot stops by its own watchdog |

**IR frame** (`calib_protocol.h`; one NEC-timed 32-bit frame, sent with IRremote `sendNECRaw`, read from `decodedRawData`):

| Byte | Content |
|---|---|
| 0 | bot ID 1..254 (0xFF = broadcast, STOP only; 0x00 is left to TV remotes) |
| 1 | op (3 bits) and CRC-5 (5 bits) |
| 2 | L code = 128 + drive_L (drive −127..127; **128 = stop**; code 0 invalid) |
| 3 | R code = 128 + drive_R |

- The NEC inverse bytes are not used; a CRC-5 (poly 0x05, init 0x1F) over the 27 data bits replaces them. It catches every single-bit error and all but 1 of the 496 possible 2-bit errors.
- IRremote's decoder does not reject marginal timings, so corrupted frames arrive as flipped bits: the CRC is what catches them.
- **Ops:** 0 STOP, 1 SEG (start segment, flash, new log record), 2 DRIVE (keep-alive or level change; ignored unless a segment is open, so a lost SEG means no motion rather than unlogged motion), 3 RECENTRE (drive, closes the segment, not logged), 4 PING (flash only).
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
2. **Marker:** a cap with a 20–22 mm ArUco `DICT_4X4_50` marker (IDs ≥ 4; IDs 0–3 are the references), printed and glued **with its top edge towards the bot's front**.
   - Leave the TSOP, the bot's IR LED and the **top NeoPixel uncovered**.
   - The NeoPixel must sit **outside the marker and its white border** (the tracker samples the LED there).
3. **Measure per bot** (calipers or a photo) and enter in the rig config:
   - `marker_offset`: marker centre relative to your body reference point, in the body frame, metres (forward, left);
   - `marker_yaw`: marker heading minus body heading;
   - `led_offset`: NeoPixel in the marker frame (forward, left).
   - The fitted map is the twist **of the reference point**. With offset 0 that is the marker centre; `geometry.transfer_twist` moves it to any other point.

### Camera and arena

- Rigid overhead mount.
  - A global-shutter camera at ≥ 60–120 fps with ≤ 2 ms exposure is needed for dynamics, noise and latency.
  - A 30 fps webcam is enough for the static map only.
  - Use flicker-free DC lighting.
  - A mono or NoIR camera needs a 940 nm-blocking filter, otherwise the blaster's reflections flicker in the images.
- **Four reference markers (IDs 0–3) on spacers at the height of the bot markers.** The homography is fitted at marker height, so there is no parallax error. Survey their centres in metres.
  - The coordinates must form a **right-handed** frame seen from above: x right, y up, ω > 0 = counter-clockwise.
  - The tracker refuses a mirrored layout, which would flip the sign of ω.
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
- `-DBOT_ID_OVERRIDE=<1..254>` resolves ID collisions; `-DUSE_HW_WDT=0` disables the hardware watchdog.

**Build notes:**
- Press the bot's power button before uploading.
- The blaster **must not** be flashed onto a bot: its QT Py build drives D12 (motor enable) HIGH at boot.
- `calib_bot` drives the MLX90393 directly over Wire and does **not** use the Adafruit MLX90393 library. That library pulls in Adafruit BusIO, whose `Adafruit_SPIDevice.cpp` fails to assemble at `opt=fastest` on this core: the core's `delayMicroseconds` inline asm uses a `"+r"` constraint, and `-Ofast` can assign it a high register that Thumb-1 (Cortex-M0+) rejects. So **any sketch using the Adafruit MLX90393 library fails to build with the FQBN HARDWARE.md recommends.**
- On Windows, gcc fails with "cannot execute cc1plus" when the arduino-cli data directory sits under a very long path (MAX_PATH). Use the default `%LOCALAPPDATA%\Arduino15`, or map a short drive letter with `subst`.
- After changing `calib_protocol.h`, copy it to `blaster/ir_blaster/` and run `python -m graspion_calib.protocol --write-vectors`. The firmware then refuses to compile if C and Python disagree.

**Bot USB commands** (115200 baud; any terminal works, and `register` and `dump` use them):

| Key | Action |
|---|---|
| `i` | info line: ID, serial, PWM mode, watchdog, battery, reset cause, number of log records |
| `d` | dump the log as CSV (non-blocking) |
| `c` | clear the log and statistics |
| `s` | statistics: frames OK, CRC errors, other addresses, watchdog trips, … |
| `r` | clear the low-battery latch |

The log holds 400 segment records in RAM, so dump and clear it after every battery. It is lost when the bot loses power.

## 4. PC setup

```bash
pip install -r calibration/pc/requirements.txt
```

Run the commands below from `calibration/pc`.

```bash
python -m graspion_calib selftest
```

`selftest` runs the offline suite in about 45 s. To also check the twist fit against your own simulator class, set `GRASPION_TILEMAT_SIM=<tilemat_control_stack>/robots/elliptical_bot/controllers/mpc/acados_simple`. That environment needs matplotlib, because `Ellipse2dObj.py` imports it.

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

## 5. Run procedure (FEASIBILITY.md §5.2)

**A. Register each bot** (USB, bot powered):
```bash
python -m graspion_calib register --port COM7
```
```bash
python -m graspion_calib fleet-check
```
Record the bot ID ↔ marker ID ↔ chip serial in `bots/` and in the rig config.

**B. Bench checks.** Do every one of these before the first map; none has been done yet.
1. **Battery bytes:** compare `i` → `mvbat` against a multimeter on the cell (the AVR byte meaning is INFERRED). Check `mag=1`.
2. **Watchdog:** send `X <id> 1 80 80` from a serial terminal on the blaster, then nothing. The motors must stop within about 300 ms (`s` → `wd_trips` increments).
3. **Address filter and CRC:**
   - `X <other id> 1 80 80` must not move the bot;
   - `f <frame with one flipped bit>` must not move it (`s` → `crc_err`);
   - `X 255 0 0 0` (broadcast STOP) stops it.
4. **Link test:** with the camera running,
   ```bash
   python -m graspion_calib schedule ping --long-s 30 --out sessions/ping.json
   ```
   then `run` it, with the bot at the centre and at each corner. `analyze` reports `link_tests` (flash loss and frames/s). If frames are lost at the default 110 ms period (e.g. the TSOP's AGC suppresses back-to-back frames), lengthen the blaster period (`bridge.period_ms`) until loss is low, and raise `WATCHDOG_MS` in the firmware to at least 2.5 periods.
5. **LED sync:** check the flash rise in `tracker.csv` (`led_b`). Lower the exposure if the LED clips.
6. **Static noise:**
   ```bash
   python -m graspion_calib schedule static --long-s 60 --out sessions/static.json
   ```
   This gives the measurement-noise floor.
7. **Re-centring:** tune `RecentreConfig` (`scheduler.py`): pivot and straight drive pairs, `latency_s`. The defaults are guesses; equal drive drifts clockwise on the twin's bot.
8. **PWM A/B test** on bot #1 if not decided.

**C. Camera intrinsics:**
```bash
python -m graspion_calib camera-calib --make-board board.png
```
Print the board and measure the squares. Take 15–30 views, then:
```bash
python -m graspion_calib camera-calib --images views/ --square-m <measured> --out camera.npz
```

**D. Rig config:** copy `pc/configs/rig_example.json` and fill it in:
- reference marker positions;
- arena;
- camera settings and intrinsics;
- blaster port;
- per-bot geometry.

**E. Static map:**
```bash
python -m graspion_calib schedule grid --blocks 5 --out sessions/grid.json
```
```bash
python -m graspion_calib run --rig rig.json --bot 90 --schedule sessions/grid.json --out sessions/bot90_grid_a
```
- The grid is L, R ∈ {0, 55, 60, 70, 80, 90, 100, 110, 120}: 80 cells, plus 8 reverse-polarity checks, in 5 randomised blocks of 3 s segments. That is about 23 min per 2 blocks, so expect more than one battery.
- After each battery, dump the bot log into the session folder:
  ```bash
  python -m graspion_calib dump --port COM7 --out sessions/bot90_grid_a/bot_dump.csv
  ```
  Then clear it (`c`).
- To continue on a new battery, split the schedule; one folder per battery is fine, because `analyze` pools folders.

**F. Dynamics:** `schedule steps` and `schedule prbs` (switching every 0.1–0.5 s).

**G. Noise:** `schedule circles --pairs "[[100,60],[60,100]]" --long-s 60` gives CW and CCW circles of R ≈ 5–15 cm. Pick pairs from the static map.

**H. Validation:**
```bash
python -m graspion_calib schedule validation --forward-map analysis/bot90/forward_map.json --out sessions/val.json
```
The targets are in (v, ω); see §6.

**Scale:** repeat for ≥ 5 bots (8–10 better), plus one bot on 3 different days.

## 6. Analysis

```bash
python -m graspion_calib analyze sessions/bot90_grid_a sessions/bot90_grid_b sessions/bot90_steps --out analysis/bot90 [--gp]
```

**Pipeline** (`report.py`):

1. **Time alignment.** Every SEG and PING flash gives `cam_offset` = flash onset − blaster acknowledgement, corrected by half a frame (IR decode + camera latency). A command acknowledged at `t_ack` acts at `t_ack + cam_offset` in camera time.
2. **Per-segment twist** (`twist.fit_twist`).
   - Skips the first 0.5 s; ends at the STOP, or earlier if the bot visibly stopped (watchdog trip after lost frames).
   - Heading spikes (e.g. a π flip) are removed before unwrapping.
   - ω comes from a line fit; (u1, u2) from one linear least-squares problem that uses the measured heading, so heading diffusion does not bias it. ICR = (−u2/ω, u1/ω) in the body frame.
   - Segments that never moved (`missed`) or hit the low-battery latch are excluded.
3. **Bot log merge** (if `bot_dump.csv` exists): per-segment mVbat at rest and under load, magnetometer at rest and under load (the offset map b(L, R) for Phase 2), and the bot clock rate. Matching is by SEG order, (L, R) and consistent spacing.
4. **Forward map** (`forward_map.py`). Per motor f = max(0, |p| − p0)^γ; the twist is 0 when both motors are in the dead zone. Variants:
   - `lin`: twist linear in (1, f_L, f_R);
   - `int`: + f_L·f_R;
   - `cor`: u1 = v(f) + d·|ω̂|, the EllipticalBot2D / digital-twin kinematics;
   - `bat`: + battery terms, when the bot log is present.

   Selection is by grouped cross-validation (folds of whole drive cells, so the score measures interpolation to unseen drive levels). The winner is refitted on all data. Cell-bootstrap draws go to `forward_map_bootstrap.json`.

   **Only the mean CoR offset d is identifiable** from steady-state twists. Any CW/CCW asymmetry is exactly absorbed by the linear v(f), so use `dist_rotPoint_to_CoM = [d, d]` with the fitted v(f).
5. **Inverse map and feasible set** (`inverse_map.py`): closed form f = A⁻¹([v, ω] − c), then p = p0 + f^(1/γ).
   - The polytope H·[v, ω] ≤ h is the image of both motors in [60, 120] (an exact parallelogram for `lin`/`cor`), for MPC constraints and RL action projection.
   - Single-motor pivots lie outside it and are listed separately.
   - `v` = u1 for `lin`, v = u1 − d|ω| for `cor`.
   - A C header with the closed-form inverse is written for onboard use (UNTESTED).
6. **Lag and dead time** (`dynamics.py`): first-order lag τ plus dead time d after decode, pooled over step/PRBS segments (else the start/stop transients). The model is passed through the same Savitzky–Golay filter as the measured velocity, so the derivative filter does not bias τ.
7. **Noise** (`noise.py`):
   - heading diffusion from second differences of the heading, which cancel any constant ω exactly, so short segments with different ω pool without bias; an OU fit for long runs;
   - measurement noise from static runs;
   - lateral slip u2.

**Outputs** in `--out`:
- `segments_fit.csv`
- `forward_map.json`
- `forward_map_bootstrap.json`
- `inverse_map.json`
- `graspion_map.h`
- `calib.json`: everything, plus latency and link statistics.

**On a simulated rig with known truth** (2 grid blocks plus 8 step segments), the pipeline recovered:

| Quantity | Recovered | Truth |
|---|---|---|
| p0 | 52.1 / 49.7 | 52 / 50 |
| γ | 0.65 / 0.60 | 0.65 / 0.60 |
| CoR offset d | 15.3 mm | 15.3 mm (mean) |
| Dead time | 32 ms | 30 ms |
| τ | 56 ms | 60 ms |
| D_r | 0.0102 rad²/s | 0.010 rad²/s |
| Selected model | `cor` | — |

This validates the code path, not the robot.

**Feeding `rl_arch_bench`** (FEASIBILITY.md §5.4):
- project actions onto `polytope`;
- (L, R) = inverse map;
- twist from `forward_map` (or a bootstrap draw per episode);
- lag τ and dead time + `cam_offset` + bridge time as the k-step delay;
- `D_r` and measurement noise;
- integrate at the reference point.

## 7. Session folder format

| File | Written by | Columns / content |
|---|---|---|
| `session.json` | scheduler | bot, geometry, arena, safety and re-centring settings, rig, PWM mode, counts |
| `schedule.json` | `run` | the schedule as run |
| `tracker.csv` | tracker | `frame, t, id, x, y, theta, led_b, led_r, side_m, n_ref`. Host `perf_counter` time at frame read; marker pose in world metres; LED chroma (B−(G+R)/2, R−(G+B)/2) |
| `frames.csv` | bridge | `t_req, t_ack, raw, op, bot, L, R, seg, purpose`: every frame sent |
| `segments.csv` | scheduler | `seg, attempt, kind, tag, block, L, R, levels, duration_s, rest_s, t_seg_req, t_seg_ack, t_end_ack, status, flash_live, moved_m, x0, y0, th0, n_frames` |
| `events.csv` | scheduler | re-centring, marker loss, retries, pauses, aborts |
| `bot_dump.csv` | `dump` (copy it in) | `# I …` info, `# S …` stats, then `kind, t_start_ms, t_end_ms, L, R, end_reason, n_frames, n_changes, flags, mv_rest, mv_load_min, mv_load_mean, n_mag, mag_rest_xyz, mag_load_xyz` |

Status values: `ok`, `missed` (no flash and no motion: the SEG was lost; retried once), `marker_loss`, `geofence`, `lowbat`.

## 8. Known limitations and open items

- **UNTESTED on hardware:** firmware, blaster, IR link rate and range, tracker with a real camera, re-centring gains, PWM 40 kHz snippet, hardware watchdog clocking, AVR battery byte meaning, MLX90393 register settings, NeoPixel timing next to IR receive.
- The NEC frame takes 68 ms. The TSOP75438's AGC4 burst rules may limit the link to about 5 frames/s (FEASIBILITY.md §3.2); the link test measures this. Below about 7 frames/s, the 300 ms watchdog will trip on single lost frames.
- The bot log lives in RAM and is lost on power loss. Persisting it to the 2 MB SPI flash is a possible follow-up.
- The scheduler drives one bot at a time. A shared IR channel divides the frame rate by N.
- Magnetometer axes relative to the body are UNKNOWN (HARDWARE.md §4.6); the log stores raw sensor axes in µT.
- Not implemented here: the hierarchical robot/day/segment variance decomposition (FEASIBILITY.md §5.3), online RLS (§5.5) and active learning. They need multi-bot data first.
