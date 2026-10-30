# Phase 0 calibration: the drive-to-motion map of a GRASPion

Measure, per bot, how motor drive (L, R) becomes body motion (u₁, u₂, ω). One PC process sends IR frames through a second bot used as the blaster, tracks an ArUco marker on the bot with an overhead camera, and writes everything to a session folder; `analyze` turns the folders into per-segment twists and a fitted map.

**Status (2026-10-30): nothing here has run on a bot, a blaster or a real camera.** The firmware compiles; the PC code passes 99 offline tests on a simulated rig (`python -m graspion_calib selftest`, about 1 min). On 2026-10-30 the tooling was cut back to what the first table needs; what went, and why, is at the end.

## The first table

What you need: one bot flashed with `calib_bot`, a second body flashed with [bot_blaster](../bot_blaster/bot_blaster.ino) (verified as an IR remote on 2026-10-16) mounted upside down above the arena, a colour webcam looking straight down (30 fps is enough for the static map), four or five printed ArUco `DICT_4X4_50` markers (IDs 0–4) glued to spacers at the height of the bot's marker, one marker (another ID, 20–22 mm) on a cap on the bot with its top edge towards the front and the top NeoPixel left uncovered, and this repository's `.venv`. Run the commands from `calibration/pc`.

1. **Flash.** Press the bot's power button, then
   ```bash
   arduino-cli compile --fqbn graspion:samd:graspionBody:opt=fastest,usbstack=tinyusb calibration/firmware/calib_bot
   ```
   ```bash
   arduino-cli upload --fqbn graspion:samd:graspionBody:opt=fastest,usbstack=tinyusb -p COM7 calibration/firmware/calib_bot
   ```
   The blaster body gets `bot_blaster` the same way (build line in its header). Lift the bot during uploads: the bootloader may drive the motors.
2. **Register.** With the bot on USB:
   ```bash
   python -m graspion_calib register --port COM7
   ```
   writes `bots/<id>.json` (IR address, protocol, watchdog). Then check the basics with the keyboard remote, bot on the floor, blaster held above it:
   ```bash
   python ../../bot_blaster/teleop.py --port COM5 --id <id>
   ```
   `w` must move it, `a` must turn it left (counter-clockwise seen from above: right motor only), and it must stop within about 0.3 s after `q`. If `a` turns it right, note it: the motor identity in HARDWARE.md §4.4 is INFERRED, and `NudgeConfig` in `scheduler.py` has to be swapped.
3. **Rig.** Survey the reference markers' centres in metres in a right-handed frame seen from above (x right, y up); the tracker refuses a mirrored layout. Copy `pc/configs/rig_example.json` to `rig.json` and fill in the references, the arena box, the camera (index, size, fps), the blaster port, and for the bot its `marker_id`, `marker_offset`, `marker_yaw` (0 for a marker glued top-edge-forward) and `led_offset`. Check it:
   ```bash
   python -m graspion_calib track --rig rig.json --out check.csv --seconds 20
   ```
   It prints the homography fit and the bot's position every second; `check.csv` must have rows for the bot's marker.
4. **Link test.** Camera running, bot at the centre, then at a corner:
   ```bash
   python -m graspion_calib schedule ping --long-s 20 --out sessions/ping.json
   ```
   ```bash
   python -m graspion_calib run --rig rig.json --bot <id> --schedule sessions/ping.json --out sessions/ping_centre
   ```
   ```bash
   python -m graspion_calib analyze sessions/ping_centre --out analysis/ping
   ```
   `calib.json` → `link_tests[0].loss` is the fraction of frames the bot missed and `offset_p5_p50_p95` the camera's delay. If loss is high, raise `bridge.period_ms` in the rig config (100–2000) and `WATCHDOG_MS` in the firmware to at least 2.5 periods, then register again.
5. **First cells.**
   ```bash
   python -m graspion_calib schedule cells --cells "[[80, 80], [0, 90], [90, 0], [100, 100]]" --blocks 2 --out sessions/trial.json
   ```
   ```bash
   python -m graspion_calib run --rig rig.json --bot <id> --schedule sessions/trial.json --out sessions/bot<id>_trial
   ```
   Before every segment the session sends STOP and waits: the line shows where the bot is; place it near the centre by hand (or nudge it with `w`/`a`/`d`), press Enter, and it drives 3 s. `x` skips a segment, `q` ends the session. Then
   ```bash
   python -m graspion_calib analyze sessions/bot<id>_trial --out analysis/bot<id>_trial
   ```
   `segments_fit.csv` has one row per segment: L, R, u₁, u₂, ω and the fit quality. Look at it before anything else: does (0, 90) turn counter-clockwise, does (80, 80) go forward, do the two replicates agree?
6. **The grid.**
   ```bash
   python -m graspion_calib schedule grid --blocks 1 --out sessions/grid_a.json
   ```
   is 88 segments (L, R ∈ {0, 55, 60, 70, 80, 90, 100, 110, 120}, 80 cells plus 8 reverse-polarity checks, random order), about 15–20 minutes by hand. Run it as in step 5, one folder per run, a fresh schedule per run, and pool the folders:
   ```bash
   python -m graspion_calib analyze sessions/bot<id>_grid_a sessions/bot<id>_grid_b --out analysis/bot<id>
   ```
   `forward_map.json` is the fitted map; in `calib.json` look at `latency` (`found` / `sent`: the sync flash was seen), `replicate_scatter` (segment-to-segment noise per cell), `forward_map.meta.cv` (does the dead-zone power law beat its variants, and by how much), `polarity_check` (do reversed drives behave like forward ones) and `inverse_map` (the feasible region).

**Not now.** Camera intrinsics (four surveyed references give the homography; distortion costs millimetres), automatic re-centring, lag and dead time (step and PRBS segments), long circles for heading diffusion, the bot's own log with battery and magnetometer readings, the C header of the inverse map, the fleet analysis in `calibration_paper/popsim.py`. Each comes back when a measured table asks for it.

## How it works

```
 PC ──USB──► blaster body ──38 kHz NEC frames──► bot TSOP (D7, lens up)   bot ──► motors, blue top-LED flash
  ▲                                                                           │
  └──────────────── overhead camera (ArUco pose + LED colour) ◄───────────────┘
```

**One segment:** the pause (STOP, the operator places the bot, Enter) → STOP and 0.8 s rest → `SEG(L, R)`: the bot starts and flashes the top NeoPixel blue for 70 ms → `DRIVE(L, R)` keep-alives every 110 ms → STOP after 3 s. The flash is the clock: its onset in the video minus the blaster's acknowledgement is the camera offset, so every command time can be placed on the camera's clock.

**IR frame** (`calib_protocol.h`, protocol 2): 32 bits, NEC timing without the inverse bytes. Byte 0 bot ID (1..254; 0xFF = broadcast STOP), byte 1 op (3 bits) and a CRC-5 (poly 0x05, init 0x1F over all 32 bits in on-air order: every single-bit error and every burst of up to 5 bits is caught), bytes 2–3 L and R as 128 + drive (drive −120..120, 128 = stop). Ops: 0 STOP, 1 SEG, 2 DRIVE (outside a segment it stops the motors and counts `drive_no_seg`), 3 RECENTRE (drive without a segment: the nudges), 4 PING (flash only). The bot ID is FNV-1a over the SAMD21 serial folded to 1..254; `fleet-check` finds collisions, fixed by building with `-DBOT_ID_OVERRIDE=<id>`.

**Safety.** The bot stops its motors 300 ms after the last frame that commands the drive (STOP, SEG, DRIVE inside a segment, RECENTRE; PING does not count): if the PC or the link dies, every bot stops by itself. The PC stops a segment when the marker is lost for 0.25 s or the pose predicted one stop-latency ahead leaves the arena shrunk by `arena.margin` (8 cm); a nudge stops on marker loss or at the arena fence while moving away from home; on any exit the session sends STOP to the bot and a broadcast STOP. The blaster holds no state.

**Drive scale:** `analogWrite(pin, 128 + drive)`, D2 = right motor, D3 = left, D12 = enable, at the core's 732 Hz PWM. The firmware caps |drive| at 120 (about 3.07 V on a motor rated 2.7 V).

**Bot USB commands** (115200 baud): `i` one info line (`fw=calib_bot/3 proto=2 id=… serial=… watchdog_ms=300 …`), `s` statistics (`frames_ok crc_err other_addr drive_no_seg wd_trips`), `c` clear them. Build options: `-DBOT_ID_OVERRIDE=<1..254>`; `WATCHDOG_MS` is a constant in the sketch.

## Commands

| Command | Does |
|---|---|
| `register --port COMx` | read the bot's `i` line into `bots/<id>.json`; refuses another protocol version |
| `fleet-check` | IR-ID collisions in `bots/` |
| `schedule grid [--blocks 5] [--seed N]` | the 9×9 signed grid in randomised blocks (fresh seed per call, recorded) |
| `schedule cells --cells "[[L, R], …]" [--blocks 1]` | chosen cells in the given order |
| `schedule ping --long-s 20`, `schedule static --long-s 60` | link test; tracker noise floor with the bot at rest |
| `run --rig rig.json --bot ID --schedule S --out DIR [--only "0-3,7"] [--video]` | run a schedule with the operator pause before every drive segment; `--only` picks segments (indices as in the file; the session's `schedule.json` is the selection); `--sim` is a dry run on the simulator |
| `track --rig rig.json --out CSV [--video FILE]` | tracker only: live camera or a recorded video |
| `analyze DIR [DIR …] --out OUT [--bootstrap 50]` | session folders → `segments_fit.csv`, `forward_map.json`, `forward_map_bootstrap.json`, `inverse_map.json`, `calib.json` |
| `selftest` | the offline suite |

**Operator pause keys:** `w` / `a` / `d` (or the arrows) one 0.6 s nudge forward / pivot left / pivot right with the `NudgeConfig` drive pairs, `space` STOP, `Enter` run the segment from where the bot stands, `x` skip it (event `skipped`), `q` end the session. Your hand may hide the marker during the pause; the segment waits for it. Segment statuses: `ok`, `missed` (no flash and no motion: the SEG frame was lost, retried once), `marker_loss`, `geofence` (the last two end early and are still used).

## Analysis (`report.py`)

1. **Time alignment.** Every SEG and PING flash gives `cam_offset` = flash onset − blaster acknowledgement, minus half a camera frame. Flashes are matched one-to-one inside a gate around the session's typical lag; a session without any flash borrows the offset of the others (`cam_offset_source`), else times stay on the acknowledgement clock with a warning. The ping run starts with three isolated anchor PINGs so that a lag above the frame period cannot be confused with one a period shorter.
2. **Per-segment twist** (`twist.py`): skip the first 0.5 s; window to the STOP, or earlier if the bot visibly stopped (a watchdog trip after lost frames); ω from a line through the heading, (u₁, u₂) by least squares in the body frame using the measured heading; ICR = (−u₂/ω, u₁/ω). Heading spikes (π flips) are removed first.
3. **Forward map** (`forward_map.py`): f = max(0, |p| − p₀)^γ per motor, twist linear in (1, f_L, f_R) (`lin`), plus f_L·f_R (`int`), or u₁ = v(f) + d̄|ω̂| (`cor`, the digital twin's centre-of-rotation offset; only the mean offset is identifiable from steady-state data). (p₀, γ) by Nelder–Mead with the coefficients solved exactly inside; the variant chosen by cross-validation over whole drive cells; a cell bootstrap for uncertainty.
4. **Inverse map** (`inverse_map.py`): f = A⁻¹([v, ω] − c), p = p₀ + f^(1/γ); the image of [p_min, 120]² is the feasible parallelogram; single-motor pivots are listed separately.
5. **Noise:** the replicate scatter per cell (`replicate_scatter`) and the tracker's noise floor from static runs (`static_noise`).

On the simulated rig the pipeline recovers the truth (p₀ 52 / 50, γ 0.65 / 0.60, d̄ 15.3 mm); that validates the code, not the robot.

## Session folder

| File | Written by | Content |
|---|---|---|
| `session.json` | run | bot, geometry, arena, safety and nudge settings, `only`, rig, protocol, period, counts, warnings |
| `schedule.json` | run | the schedule as run |
| `tracker.csv` | camera thread | `frame, t, id, x, y, theta, led_b, led_r, side_m, n_ref` |
| `frames.csv` | bridge | `t_req, t_ack, raw, op, bot, L, R, seg, purpose`: every frame sent |
| `segments.csv` | scheduler | per segment: L, R, timing, `status`, flash seen, displacement, start pose |
| `events.csv` | scheduler | marker loss, retries, `skipped`, `nudge_*`, aborts |

## Tests and what is untested

`selftest` (99 tests, one skipped unless `GRASPION_TILEMAT_SIM` points at the lab's `EllipticalBot2D`) covers the protocol and CRC against compile-time vectors, the twist fit, the tracker on rendered scenes, every scheduler path (the pause and nudges with scripted keys, marker loss, geofence, lost SEG, Ctrl-C mid-reply, the fake blaster), the firmware's frame rules in a Python mirror, and a full simulated grid through `analyze` against known truth. **Untested on hardware:** the firmware (compiled only; the C code is covered only by compiling it), the blaster link rate and range, the tracker with a real camera, the key reader, everything.

Known gaps: the AVR power monitor is not read, so a flat battery is the operator's job (the body cuts off near 2.8 V, after which only USB revives it); the stall check for a bot that flashes but does not move is gone with it; a segment the bot abandoned early is detected only from the video.

## Removed on 2026-10-30

Cut because they stood between the first measurement and us, without a measurement asking for them; all of it is in git history (`calibration/` as of commit `6dde690`): the Nano/QT Py `ir_blaster` and its LED array (a `bot_blaster` body does the job), the firmware's RAM log and `dump`, battery and magnetometer reads with the low-battery latch and red flash, the I²C bus recovery, the hardware watchdog, SysTick deadman and bootloader quick-boot flag, the 40 kHz PWM option and the bench-hang option; the bang-bang re-centring with its five stop conditions; step, PRBS, circle and validation schedules; the lag and dead-time fit, heading-diffusion and circle-jitter noise models, the GP comparator, the battery model variant, the C header export; ChArUco camera calibration; the mono-camera stall detection. HARDWARE.md §9.1 still records the earlier builds.
