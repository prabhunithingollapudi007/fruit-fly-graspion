# Phase 0 calibration: the drive-to-motion map of a GRASPion

Measure, per bot, how motor drive (L, R) becomes body motion (u₁, u₂, ω). One PC process sends IR frames through a second bot used as the blaster, tracks an ArUco marker on the bot with an overhead camera, and writes everything to a session folder; `analyze` turns the folders into per-segment twists and a fitted map.

**Status (2026-11-12): the PC side has run a first full grid on a real robot** (see [With the lab's interim v4](#with-the-labs-interim-v4)), through the teammate's IR remote and receiver, not through `calib_bot` and `bot_blaster`, which are still compiled only. The PC code has no test suite (removed on 2026-11-12); `run --sim` dry-runs it. On 2026-10-30 the tooling was cut back to what the first table needs; what went, and why, is at the end.

## The first table

What you need: one bot flashed with `calib_bot`, a second body flashed with [bot_blaster](../bot_blaster/bot_blaster.ino) (verified as an IR remote on 2026-10-16) mounted upside down above the arena, a camera looking straight down (a colour webcam at 30 fps is enough for the static map; the lab's camera is a mono Basler, see [This rig's camera](#this-rigs-camera)), four or five printed ArUco `DICT_4X4_50` markers (IDs 0–4; `python markers/make_markers.py` from the repo root prints them as a vector A4 PDF, 50 mm, IDs 0–3 plus spares 4–5) glued to spacers at the height of the bot's marker, one marker (another ID, 20–22 mm) on a cap on the bot with its top edge towards the front and the top NeoPixel left uncovered, and this repository's `.venv`. Run the commands from `calibration/pc`.

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
3. **Rig.** On the lab rig, start from `pc/configs/rig_interim_v4.json` (see [With the lab's interim v4](#with-the-labs-interim-v4)): put in the bot's IR ID, the blaster port and the tag height you measured. Elsewhere, survey the reference markers' centres in metres in a right-handed frame seen from above (x right, y up); the tracker refuses a mirrored layout. Copy `pc/configs/rig_example.json` to `rig.json` and fill in the references, the arena box, the camera (index, size, fps, `camera_height_m`), the blaster port, and for the bot its `marker_id`, `marker_offset`, `marker_yaw` (0 for a marker glued top-edge-forward), `marker_height` (tag face above the reference markers' plane) and `led_offset` (null when the LED cannot be seen). Check it:
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

**Not now.** Our own camera calibration (the lab rig uses the teammate's, see below), automatic re-centring, lag and dead time (step and PRBS segments), long circles for heading diffusion, the bot's own log with battery and magnetometer readings, the C header of the inverse map, the fleet analysis in `calibration_paper/popsim.py`. Each comes back when a measured table asks for it.

## With the lab's interim v4

The lab's camera was calibrated by a teammate with `graspion_tracking` (brainbot_ws, branch `camera_calibration`). The current state is **interim v4** (commit d7814a5, 2026-11-12, still in progress on their side). It has a fixed, wider camera view, the arena markers moved out to the view's edges for more room, and the lens model of calib v1 (2026-11-11, same focus and iris) carried over to the new view. `pc/configs/calib_2026-11-12_interim_v4/` holds unchanged copies of its `intrinsics.json`, `arena.json` and `camera_settings.json`, and `pc/configs/rig_interim_v4.json` uses them. Calib v1's files are void since the markers moved. **Not yet run on the camera.**

- **Accuracy is not yet at 1 mm.** Their layout fit misses the 1 mm gates narrowly (corner rms 1.24 px, max 2.93 px). Refitting with only some of the markers moves robot positions by 0.8–1.2 mm median (up to 3.3 mm), and the 0 → 1 distance changes by up to 0.3 %. The cause is the lens model, which is extrapolated at the view's edges, where the markers now sit. For the drive-to-motion map that means about 0.3 % on speeds, small next to segment-to-segment noise. Copy the files in again when they finish v4 with tile shots at the edges (or check 0 → 1 with a steel tape).
- **References on the floor, tag at the robot's height.** The arena markers lie flat (z = 0), and the robot's tag is 22.9 mm above them (from the STLs; replace it with a ruler reading in `marker_height`). Through the floor homography the tag would come out 2.5 % too far from the point under the camera, up to about 10 mm at the edge, and every speed 2.5 % too high. The tracker puts each tag corner back on its camera ray at `marker_height`, using the camera centre from the homography and K.
- **Camera view.** 2700 × 2064 at offset (192, 0), 20 ms, 5 dB, Mono8. It is also saved in the camera as UserSet1, which loads at power-on; the camera delivers about 50 fps. `camera.backend: "pylon"` opens the Basler through pypylon, sets the view, exposure and gain again, and refuses frames of any other size. If a config gives no offsets, the view is centred, with the offsets rounded to the camera's steps. The lens model is moved to whatever offsets the camera reports; only the principal point shifts. The camera must not be attached to WSL or the teammate's container at the same time, and pylon Viewer must be closed. The pypylon code is UNTESTED on the camera, and how many of the 50 fps the tracker keeps up with is not measured.
- **No sync LED.** The tag covers the top NeoPixel, so `led_offset` is null. The bot then has no flash: segment times stay on the blaster's acknowledgement clock (fine for 3 s segments whose first 0.5 s is skipped), and the "missed" retry is off, because a bot that does not move would otherwise be dropped as a lost SEG. A really lost SEG then shows only as a still segment in `segments_fit.csv`. The ping link test sees nothing either: read the loss from the bot's own `s` counters over USB.
- **Arena.** Where the robot's centre stays 31 mm clear of the marker papers: x 86–860 mm, y 87–633 mm (774 × 546 mm, as their `arena_extent_v4.py` gives), with a 2 cm geofence margin. Markers 4 and 6 (bottom and top middle) set the height; moved to the sides they would give 774 × 680 mm, and then `arena.json` and the box must be redone. A 3 s run at full speed (about 6.6 cm/s, from the teammate's closed-loop trial) covers about 20 cm, which fits from the centre (473, 360) mm in every direction.
- **IR: the teammate's remote and receiver.** `bridge.type: "ir_remote_tx"` drives W10nY (COM5, `ir_remote_tx`) instead of `bot_blaster`. The robot runs their `ir_drive_rx`, which writes the PWM bytes to `analogWrite` as `calib_bot` does (128 + drive). Our STOP becomes `s`; SEG, DRIVE and RECENTRE become `pwm <128+L> <128+R> 1000`, re-sent only when (L, R) changes or every 0.5 s, since the remote repeats frames every 100 ms itself. If the PC dies, the robot stops within about 1.5 s. Not used: `register` (no `calib_bot` ID) and the ping test.
- **`run --auto`: no operator.** Before each segment the robot re-centres itself over the camera (`RecentreConfig` in `scheduler.py`). It pivots towards the centre with (0, 90) or (90, 0), drives with (85, 85), then pivots onto the long axis (heading 0° or 180°), in 1–6 s. A failed re-centre is retried 3 times, then the segment is skipped (rerun it with `--only`). Watch it with `python live_view/live_view.py <session> [--map forward_map.json]`: expected against measured track per segment.
- **First grid, 2026-11-12** (sessions `grid_a` + `grid_a2` + `first_cells_0/1`, `analysis/grid_v1`; one robot, one battery): 95 segments fitted, 3 cut short by the geofence. The `lin` model wins cross-validation narrowly; dead zones p0 = 54.5 (left) / 59.9 (right), exponents 0.25 / 0.20 (speed saturates above 70–80). Cross-validated error 0.70 cm/s in u1 and 0.35 rad/s in ω, against a replicate scatter of 0.51 cm/s and 0.19 rad/s (only 4 cells replicated). Reverse drives behave like forward ones. A sub-threshold motor still matters: at L = 80, R = 0 → 55 halves the turn rate (−1.98 → −1.03 rad/s), which the per-motor dead zone does not model.
- **Second grid, 2026-11-12** (`grid_b`, 88/88 ok; pooled fit `analysis/grid_v2`, 183 segments): the map stays put (p0 54.5 / 59.9, exponents 0.24 / 0.20, `lin`), and the two grids agree on average (B − A: −0.09 cm/s, −0.02 rad/s). The replicate scatter over 87 cells is 0.23 cm/s and 0.10 rad/s (median sd; p90 0.72 cm/s, 0.28 rad/s), well below the map's cross-validated error of 0.79 cm/s and 0.37 rad/s, so the model form, not the noise, limits the map; the `int` and `cor` variants do no better. The battery shows: within `grid_b` the speed falls from 1.03× to 0.91× the grid-A value from the first to the last third.
- **Validation on held-out drives, 2026-11-12** (`val_1`, 20/20 ok; 12 off-grid cells, 6 drives from the inverse map, (80, 80) first and last). The battery had not faded: (80, 80) read 0.99× and 1.03× its grid mean. The `grid_v2` map predicts the held-out drives to 0.68 cm/s in u1, as its cross-validation said (0.79), but only to 0.51 rad/s in ω (0.37 expected). Bilinear interpolation of the measured grid-cell means does better on the same 18 drives: 0.48 cm/s and 0.33 rad/s. The inverse map hits its speed targets (0.55 cm/s rms) but not always its turn rate: "straight at 5 cm/s", (65, 79), turned +0.43 rad/s. Single-motor pivots near the dead zone are the worst: (75, 0) turned −2.78 rad/s against −1.18 predicted.

## This rig's camera

Model numbers from the lab (lens corrected to the 6 mm model on 2026-11-09). Camera specs from Basler's product pages (read 2026-11-04); lens specs from the vision-dimension.com product page (read 2026-11-09). pylon 26.10.1 on the lab PC sees the camera as "Basler ace USB3 Vision Camera" (2026-11-09).

| Part | Specs |
|---|---|
| Camera | Basler ace U **acA3088-57um**: Sony IMX178, 3088 × 2064 px, 2.4 µm pixels, sensor 7.41 × 4.95 mm (1/1.8"), **mono**, **rolling shutter**, 59 fps at full resolution, 10/12 bit, USB 3.0, C-mount |
| Lens | Ricoh **FL-CC0614A-2M**: **6 mm**, F1.4–F16, for 2/3" sensors (covers the 1/1.8" sensor), C-mount, minimum object distance 0.10 m, manual focus and iris with lock screws, filter thread M46 × 0.5 |

On this sensor the lens sees about 63° × 45° (computed; the vendor's 71.2° horizontal is for a full 2/3" sensor). View against camera height H above the marker plane (thin lens, field ≈ 1.235 H × 0.825 H; computed, not measured):

| H | field of view | px/mm | 20 mm cap marker | 50 mm reference |
|---|---|---|---|---|
| 1.0 m | 123 × 82 cm | 2.5 | 50 px | 126 px |
| 1.35 m | 166 × 111 cm | 1.86 | 37 px | 93 px |
| 1.5 m | 185 × 123 cm | 1.67 | 33 px | 84 px |
| 2.0 m | 246 × 165 cm | 1.25 | 25 px | 63 px |

- **Height.** The short side of the image sets it: a 1 m arena plus the reference markers needs **H ≥ 1.35 m**. At 1 m the view is 82 cm tall, too short for the 1 m arena. The lab's earlier blob-tracking script used 16.26 px/cm (1.63 px/mm), which matches this camera at full resolution from H ≈ 1.5 m (INFERRED: assumes its video was not resized).
- **Focus.** Open the iris to F1.4, focus on a printed marker at marker height, then stop down and lock both rings. Depth of field at 1 m with a 2-pixel blur limit (computed): about 0.84–1.23 m at F1.4, about 0.65–2.1 m at F4.
- **Wide angle** (INFERRED, UNTESTED). Barrel distortion and parallax grow toward the edges of a 63° view. Parallax at the edge is about marker height × r / H, so a cap 17.5 mm up, 0.7 m off-centre at H = 1.35 m, is off by about 9 mm against floor-level references. Fit the homography with references raised to marker height. Measure the distortion on a first video before deciding whether the camera intrinsics are needed after all ("Not now" below).
- **Full resolution, no binning.** 2 × 2 binning halves px/mm and leaves a 20 mm cap marker at about 3 px per marker cell, where a simulated detection test (blur and noise, the tracker's detector) lost 7–20 % of frames; at full resolution the cap has 5–6 px per cell. A narrower region of interest does not raise the frame rate, since the arena needs all 2064 rows (INFERRED for a rolling-shutter sensor). Plan on 59 fps.
- **Exposure ≤ 2 ms**, set in pylon Viewer and saved as the camera's startup user set; `rig_example.json` leaves `camera.exposure` null so the tracker does not touch it. F1.4 leaves room to stop down to f/2.8–4.
- **Mono.** The tracker detects it and logs the LED patch brightness instead of blue chroma (`led_mode` "luma", `led_r` NaN); the sync flash still shows as a brightness rise. Check that rise in `tracker.csv`.
- **940 nm filter.** Basler's mono models have no IR-cut filter (INFERRED), so the blaster's reflections can flicker in the images. Fit a 940 nm-blocking filter to the lens's M46 × 0.5 thread.
- **Lens resolution.** "-2M" marks a 2-megapixel lens (INFERRED from the name), so the optics blur over more than one 2.4 µm pixel; harmless for markers of 30 px and more.
- **Rolling shutter** (INFERRED, UNTESTED). Rows are exposed up to one readout apart, at most about 1/59 s ≈ 17 ms top to bottom, while the tracker stamps a frame with one time. The bot's flash and its marker lie in the same rows, so the offset is common to both; a twist is fitted over a few centimetres of path, under 1 ms of row offset. `cam_offset` scatters by up to 17 ms with the bot's row, and the PC's marker-loss and geofence stops see up to 17 ms more latency, within `cam_latency_s` (0.05 s).
- **Live capture.** `open_camera` opens a UVC camera through OpenCV's DirectShow backend. A Basler USB3 camera is a USB3 Vision device, not UVC, and needs Basler's pylon software. OpenCV may open it through pylon's DirectShow driver (UNTESTED; exposure units are then unknown, which is why exposure is set in pylon Viewer). There is no pypylon backend here yet. `track --video` on a pylon Viewer recording works either way.

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

## What is untested

The offline test suite was removed on 2026-11-12 (git history has it, last in commit 6845bd1); the firmware's protocol vectors are still checked at compile time by `static_assert`. **Untested on hardware:** the firmware (compiled only; the C code is covered only by compiling it), the blaster link rate and range, the tracker with a real camera, the key reader, everything.

Known gaps: the AVR power monitor is not read, so a flat battery is the operator's job (the body cuts off near 2.8 V, after which only USB revives it); the stall check for a bot that flashes but does not move is gone with it; a segment the bot abandoned early is detected only from the video.

## Removed on 2026-10-30

Cut because they stood between the first measurement and us, without a measurement asking for them; all of it is in git history (`calibration/` as of commit `6dde690`): the Nano/QT Py `ir_blaster` and its LED array (a `bot_blaster` body does the job), the firmware's RAM log and `dump`, battery and magnetometer reads with the low-battery latch and red flash, the I²C bus recovery, the hardware watchdog, SysTick deadman and bootloader quick-boot flag, the 40 kHz PWM option and the bench-hang option; the bang-bang re-centring with its five stop conditions; step, PRBS, circle and validation schedules; the lag and dead-time fit, heading-diffusion and circle-jitter noise models, the GP comparator, the battery model variant, the C header export; ChArUco camera calibration; the mono-camera stall detection. HARDWARE.md §9.1 still records the earlier builds.
