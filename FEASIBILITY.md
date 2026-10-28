# Running the tilemat brains on GRASPion: feasibility study

**Question.** Can the figure-8 controllers trained in `tilemat_control_stack` run on the real GRASPion brainbot? The controllers are the acados MPC, MPC + residual PPO, and standalone PPO on the low, dense, bio, FlyWire-DN and MANC-VNC actors. The brain could be external (a remote "brain") or on the robot itself. What must "learning to walk" (physical calibration) look like first, and what does the literature say?

**Status.** Compiled 2026-10-15 from:
- the controller code in `tilemat_control_stack/robots/elliptical_bot/controllers/mpc/acados_simple/` (PR #1, `rl_arch_bench`) and its `runs/full` results;
- [HARDWARE.md](HARDWARE.md);
- the lab's digital-twin repo (github.com/GRASP-LAB/brainbot-digital-twin);
- datasheets and papers.

An adversarial cross-check pass followed, and every cited paper was verified against its primary page. Updated the same day with findings from building the Phase 0 tooling in [calibration/](calibration/README.md): CoR identifiability (§4) and the implementation pointers in §5.2–5.4. Updated 2026-10-28 with the lab's two control repos (§5.1), a stage-by-stage comparison with the literature (§8) and a VLA section (§10; its model facts were checked against the papers' abstracts and the OpenVLA paper's §3.5). Nothing has been measured on a real bot yet. Labels follow HARDWARE.md: **ESTIMATE** = my arithmetic, **INFERRED** = reasoned, **UNKNOWN** = must be measured.

---

## 0. Verdict

1. **Compute is not the problem.** Every trained actor is a tiny tanh MLP: 390–1,535 parameters, ≤ 6,776 MACs per step, at 10 Hz.
   - All of them fit on the body's SAMD21 (Cortex-M0+, no FPU), in float32, at about 2–20 ms per step when stored sparse. The budget is 100 ms (ESTIMATE).
   - The acados MPC (double precision, SQP-RTI + HPIPM) belongs on a PC (0.2 ms solve).
   - The onboard MPC option is **TinyMPC**: easy on an STM32/M4F-class chip, marginal on the SAMD21.
2. **Blocker 1: pose.** Every controller observes e_long, e_lat and e_θ against a time-indexed reference, so it needs **absolute (x, y, θ)**. GRASPion has no odometry, so **an overhead camera is mandatory wherever the brain runs.** The onboard magnetometer can give heading, and dead reckoning drifts about 6–8 cm per figure-8 lap (ESTIMATE). That is already above your lap-2 ≤ 5 cm success criterion.
3. **Blocker 2: actuation.** All controllers output (v, ω), but the robot takes left/right motor power. **No (v, ω) → (L, R) map exists in your stack.** The only published calibration (one bot, digital twin) is coupled, asymmetric and biased:
   - equal powers turn the bot clockwise at −0.69 to −1.15 rad/s;
   - speed has a floor of about 4.7 cm/s;
   - counter-clockwise turns only exist above about 7.3 cm/s, with a tightest radius of about 6.5 cm.
   - **Your benchmark figure-8 (A 0.15, B 0.20) is not trackable on that envelope as specified** (§4).
4. **Blocker 3: the simulator is too ideal.** It applies (v, ω) instantly with only a linear gain/bias mismatch. It has no delay, no lag, no noise, a centre-of-rotation (CoR) offset of 0–4 mm, and a box action set of v ∈ [0, 0.15] m/s, ω ∈ ±5.3 rad/s. The real loop has:
   - 14.5–16.1 mm CoR offset in every turn (the twin's values; steady-state calibration can only identify their mean, about 15.3 mm, §4);
   - a motor that reaches 50 % in 50 ms;
   - ≈ 90–260 ms from camera to motor (ESTIMATE).
   - **The trained checkpoints cannot be deployed as-is. Retrain on an identified plant.**
5. **RL from scratch on the robot is impractical.** Your reported results used the `full` preset: 640 k steps (residual) and 3.2 M steps (standalone). At 10 Hz that is **17.8 h and 88.9 h** of robot time, or about 12 and 59 battery charges. Use calibrate → refit the sim → retrain → deploy → adapt online.
6. **Your "learning to walk = physical calibration" idea is exactly the right first step.** It is the hard prerequisite for everything else, and it needs no new robot hardware: a camera, an IR LED blaster and firmware (§5).
7. **Scientific opportunity.** Within about 40 web searches (not a systematic review), no paper was found that runs deep-RL/NN tracking policies, residual RL on MPC, or FlyWire/MANC-derived controllers on a physical bristlebot or brainbot. The only learned policies on vibration robots were a central light-field policy over Kilobots and onboard evolutionary RL of a phototaxis threshold. The lab's own digital-twin paper names NN controllers and multi-bot interaction as its future work, so the opening is shared with them (§8).
8. **Safety first on hardware.** Every closed-loop phase needs a **command watchdog**: the motors must stop when no valid frame arrives within ~300 ms. Without it, a lost IR frame or a stalled PC leaves the bot driving into walls or down to the 2.8 V cut-off, after which only USB revives it (§5.2).
9. **Connectome vs. shuffle (simulation).** No measurable advantage: all contrasts of flywire_dn and manc_vnc against their shuffles or dense have 95 % CIs that include 0 (`runs/full/report/RESULTS.md`). For hardware, pick the cheapest actor. Keep the connectome actors as the scientific comparison: they are just as deployable.
10. **A VLA without explicit calibration is feasible as a hierarchy, not end to end.** The VLA (PC GPU, 1–6 Hz) sets goals or target motions; a small adaptive low-level policy, trained in simulation on randomised plants, maps them to (L, R) and infers each bot's quirks from its recent history. That replaces the per-bot calibration session, not its data: the play data come from the existing calibration rig. Bench checks, the PWM mode and the safety layer stay mandatory (§10).

---

## 1. What has to run

All learned actors share one contract:
- **Network:** masked tanh MLP with a linear readout, deployed as `a = tanh(y)`. Stateless. The critic and `log_std` are training-only.
- **Observation** (9-D, clipped to ±1): `[e_long/0.3, e_lat/0.3, e_θ/π, v_ref/0.15, ω_ref/5.3, obs5, obs6, sin φ, cos φ]`.
  - obs5/6 is u_mpc in residual mode, and the previous command (efference copy) in standalone mode.
  - φ = 2πt/T is the trajectory clock.
- **Output:**
  - standalone: v = 0.075(a₀+1), ω = 5.3·a₁;
  - residual: u = clip(u_mpc + a ⊙ [0.04 m/s, 1.0 rad/s]).
- **Rate:** dt = 0.1 s, zero-order hold.

| actor | layers | trainable params | MACs sparse / dense | fig-8 RMSE, sim (mean T16/20/28, 5 seeds) |
|---|---|---|---|---|
| low | 9-16-12-2 | 390 | 360 / 360 | standalone 4.07 mm |
| bio / bio_steer | 9-32-16-2 masked | 410 / 426 | 360–376 / 832 | standalone bio 7.48 (ω cannot see e_θ); **residual_preview bio 4.25 mm**; bio_steer 3.60 |
| dense | 9-32-16-2 | 882 | 832 / 832 | standalone 3.42 |
| flywire_dn (+ shuffled) | 9-77-77-2, real FAFB DN→IN mask | 1,241 | 1,085 / 6,776 | **standalone 3.16 (shuffled 3.10)**, the best non-privileged result on the nominal figure-8 mean |
| manc_vnc (+ shuffled) | 9-32-54-32-2, MANC DN→IN→leg-MN | 1,535 | 1,415 / 3,808 | standalone 3.90 |
| MPC baselines | acados unicycle, N 15 | — | — | legacy 18.64; preview+warm 7.02; oracle 2.82 |

**Do not deploy:**
- the legacy `flywire_vnc*.pkl`, whose IN→MN mask is empty so the output is constant;
- the legacy `flywire_vnc_hybrid.pkl` (52 k parameters, partly synthetic edges).

**The MPC:**
- unicycle model; cost 60‖e_p‖² + 80 e_θ² + 1.5(Δv² + Δω²) + 0.01‖u‖²;
- SQP_RTI + FULL_CONDENSING_HPIPM, exact Hessian, ERK4, **double precision**;
- no terminal cost, and a position box of ±0.2 m.

`runs/` exists only in the `.claude/worktrees/rl-training-trajectory-viz-03947b` worktree of tilemat, not in the main checkout.

---

## 2. Compute feasibility (ESTIMATES from published per-operation cycle costs; measure with `micros()` on a bot)

| controller | SAMD21 body (M0+ 48 MHz, no FPU, 32 KB RAM) | STM32U575, 2026 head (M33 + FPU 160 MHz) | nRF52832 (M4F 64 MHz) | PC |
|---|---|---|---|---|
| low / bio / bio_steer / dense | **yes**: float32 2–10 ms, ≤ 4.5 KB flash, < 0.3 KB RAM | **yes**: ≤ 0.12 ms | **yes**: ≤ 0.3 ms | yes |
| flywire_dn | **yes if stored sparse (CSR)**: 9–21 ms, tanh dominates, so use a LUT tanh. Avoid dense storage (23–63 ms) | yes: 0.3–0.5 ms | yes: ≤ 1.3 ms | yes |
| manc_vnc | **yes (sparse)**: 6–20 ms, 7.8 KB | yes: ≤ 0.33 ms | yes: ≤ 0.83 ms | yes |
| acados MPC (all variants) | **no**: double precision, a library stack of hundreds of KB, no port | not recommended: single-precision FPU, no documented port | no | **yes**: 0.18–0.24 ms solve, ≤ 1.15 ms call |
| TinyMPC (linearised, nx 3–4, nu 2, N 10–15) | **marginal**: ~4–21 ms per ADMM iteration, so cap at ~3–7 iterations (≈ 50 ms budget) with warm start, or use the N 10 double integrator | **yes**: 0.05–0.1 ms per iteration | yes: 0.1–0.3 ms per iteration | yes |
| MPC + residual | marginal (TinyMPC dominates) | **yes** | yes | **yes** (acados + actor ≈ 1 ms) |
| MPC distilled into an MLP | **yes** (same cost as an actor) | yes | yes | n/a |
| on-robot PPO training | **no**: a 53–218 KB rollout buffer exceeds 32 KB RAM | compute OK; bottleneck is robot hours | marginal (RAM) | compute OK; same bottleneck |

**Notes:**
- **int8 is unnecessary on the SAMD21**, because float32 already fits. Per-tensor int8 caused up to 0.093 normalised action error, i.e. 0.49 rad/s on standalone ω. If you quantise, use per-channel scales or QAT, and validate closed-loop.
- **TinyMPC caches one LTI Riccati solution.** For a unicycle, either use the lab's double-integrator + feedback-linearisation trick or gain-schedule the cache.
- **Residuals must be retrained against whichever base MPC they sit on**, because u_mpc is an input.
- **Export path:**
  - dump `state_dict` weights and masks to a C header as `const` CSR arrays;
  - float32 on M33/M4F; float with Qfplib/libgcc or a LUT tanh on the M0+;
  - hand-written C is smallest at these sizes (CMSIS-NN, TFLite-Micro and emlearn are all optional);
  - precedent: "Learning to Fly in Seconds" exports the actor as a C header into Crazyflie firmware.

---

## 3. Where the brain can live

### 3.1 Options

| | **A. External brain** | **B. Onboard brain** | **C. Hybrid** (the architecture that scales) |
|---|---|---|---|
| Sensing | Overhead camera + ArUco → PC computes pose | Magnetometer heading (MLX90393, fast settings 1.3–7.6 ms, but disturbed by the ERM magnets and needs per-(L, R) offset calibration), calibrated open-loop speed, light sensor, APDS | Camera pose fixes broadcast at 1–10 Hz, fused onboard with dead reckoning and magnetometer |
| Compute | PC runs acados MPC / residual / actor, then (v, ω) → (L, R) | SAMD21 or 2026 head runs the actor, distilled MPC or TinyMPC | Onboard actor at a steady 10 Hz |
| Link | Every command: 5–9 Hz over IR NEC (8-bit command, address = bot ID), or ≥ 10 Hz over BLE | none | ~32 bits per fix (ID 5 + x 9 + y 9 + θ 9); reserve bits for a check and a phase/age field |
| Good for | **Phase 1**: one bot, fastest path to closed-loop figure-8, all controllers incl. acados | Behaviours that need no absolute pose: run-and-tumble, phototaxis, Kuramoto colour sync (`colors.ino`, which does not compile as published and uses SERCOM3, so not together with the SPI1 flash), compass alignment, one-lap open-loop shapes | Several bots; delay can be compensated **if the fix age is known** (fixed TDMA slots or an age field), plus a periodic trajectory phase-sync word against the ±2 % clock drift |
| Limit | Link rate and latency; a swarm divides the link | Drift: about 6–8 cm per lap from speed-model error alone | Needs onboard estimator firmware |

### 3.2 Downlinks to the robot

| link | rate | payload / addressing | bot hardware | verdict |
|---|---|---|---|---|
| **IR NEC into the bot's TSOP75438** from an overhead IR LED array | 68 ms per frame, repeat 108–110 ms → **≤ 9 frames/s for the whole channel**. The TSOP AGC4 long-burst gap rule may cut this to ~5/s, so bench-test it. Decode latency ≈ 76 ms (the frame plus IRremote's 8 ms end-of-frame gap). The calibration frames replace the inverse bytes with data and a CRC, so they take 53–82 ms (calibration/README.md §8) | NEC: 8-bit address (= bot ID) + 8-bit command, with inverses. That is 4-bit L + 4-bit R, or an index into a 256-entry per-bot codebook. Onkyo: 16 + 16 bits, no check. **Firmware must filter the address**, which the examples ignore | **none**: lens-up per the V1.3 schematic (TSOP75438TT), which suits an overhead blaster. **Verify on your bots**, especially V1.1, which has no published schematic | **Best Phase-0/1 link**, with zero hardware change. N bots get 9/N Hz each |
| IR custom pulse-distance coding within the TSOP rules | ~20 frames/s (ESTIMATE) | 32+ bits | none | Worth a bench test |
| **BLE via the digital-twin STM32 add-on** | The paper reports 30 Hz; the published firmware is consistent only with ~10 Hz control (§5.1), so verify | 13-byte pose packets; body command `"M<l>,<r>\n"` at 115200 baud | Borrow it. Three caveats: (1) the body-side parser is unpublished, and a new one must map `M0,0` to stop (128 / EN LOW), **not** analogWrite 0, which is full reverse; (2) it must ignore the add-on's non-`M` lines ("Connected"/"Disconnected"); (3) using it as a pure link for a PC brain needs new add-on firmware, since it currently runs TinyMPC itself | **Best Phase-1 link if the GRASP/FAU group lends it.** Its real advantage over IR is lower latency and a reliable ≥ 10 Hz |
| BLE via the 2026 head (STM32U575 + ANNA-B112) | same class | head↔body UART4 = body D9/D10 | **no STM32 firmware exists**; the nRF needs an SWD first flash; it shares the body's 100 mA LDO | Phase 2–3; also enables onboard TinyMPC |
| BLE connectionless advertising (broadcast) | ≈ 40 adv/s → **≈ 10 Hz per bot at N = 20** (6 bots per 27 B packet; ESTIMATE) | no connection limit | an nRF per bot | **Best swarm downlink** |
| BLE-UART module on HEAD X3 (TX D10, RX D9, +3V1) | as BLE | | 3.3 V module; **displaces the APDS front module** (same X3 connector; D10 is also the APDS LED enable) | Stopgap |
| Projected light field onto the up-facing TEMT6202 | 60 Hz, spatially addressed | a few bits per bot per frame (untested) | none (saturates at 2–4 klx) | Research option. Gebhardt et al. 2018 steered real Kilobots *collectively* with one projected moving light spot via onboard phototaxis, and needed a higher-dynamic-range light sensor than the stock part. Per-bot addressed bits by projection is untested |
| USB tether | | | | Bench debugging only; the cable dominates a 17 g bot |

BLE connections: one nRF central (S132/S140) supports ≤ 20 links, and Windows' built-in BLE manages about 3–4 in practice. Use an nRF52840 USB dongle as central.

### 3.3 Latency, camera photon → 50 % motor response (ESTIMATE; measure on your rig)

| setup | total | in control periods |
|---|---|---|
| USB webcam (~90–112 ms) + IR NEC (≈ 76 ms decode) + USB-serial bridge (1–16 ms) | **≈ 245–260 ms** (lower bound) | ~2.5 dt |
| webcam + BLE | ≈ 185 ms | ~2 dt |
| global-shutter ≥ 120 fps camera + IR NEC + bridge | ≈ 150–165 ms | ~1.5 dt |
| global-shutter camera + BLE | **≈ 90 ms** | < 1 dt |

The webcam figure was measured at 320×180, photon-to-display, on a different camera; 1080p is probably slower. Prefer an RP2040 with native USB for the IR bridge, or set the FTDI latency timer to 1 ms.

**Control period over IR.** IR cannot deliver the trained 10 Hz: the period is ≈ 0.11 s at 9 frames/s or ≈ 0.2 s at 5 frames/s. Run the PC loop on the IR frame clock (otherwise an unsynchronised 10 Hz loop adds 0–110 ms of queueing), retrain with dt = link period plus dt jitter, and regenerate the acados solver with the matching Ts. Only BLE supports 10 Hz as trained.

During 240 ms at ω = 1.56 rad/s the bot turns 0.37 rad (all latencies are ESTIMATES; measure them with an LED round-trip test). **Compensate by forward-predicting the measured pose through the known delay with the commands already sent, and add delay and lag randomisation to training.**

**Camera notes:**
- **Fixed short exposure.** Use ≤ 2–5 ms with flicker-free DC lighting. 33 ms auto-exposure gives about 4 mm of blur at 11.5 cm/s.
- **Parallax.** The marker sits 15–20 mm above the floor. Fit the homography at marker height, using elevated reference markers. With the camera at 1 m, a floor-plane fit is off by 3–4 mm inside the current figure-8, 6–10 mm on the enlarged A 0.40–0.50 m path, and 8–14 mm at the arena edge: the same size as your simulated RMSE.
- **Marker placement.**
  - Keep the TSOP and the IR LED (IR downlink) and the top NeoPixel (time sync) uncovered. The TEMT6202 matters only for the light-field option.
  - Part positions are INFERRED from the V1.1 photo, so check them on your board.
  - Use one marker size throughout (20–22 mm).
  - Measure the marker's offset and heading alignment relative to the body axis, and define the tracked reference point explicitly. Moving the reference point by 10 mm changes point speed by up to about 3 cm/s at 3 rad/s.
- **Mono / NoIR cameras** (for example OV9281) need an IR-cut or 940 nm-blocking filter, otherwise the blaster's reflections flicker in the tracking images.

---

## 4. Sim-to-robot gap, quantified

| item | simulator (`pure_rl_env.py`, `residual_env.py`, `presets.py`) | GRASPion (one digital-twin bot; your bots must be measured) |
|---|---|---|
| action | (v, ω), box v ∈ [0, 0.15], ω ∈ ±5.3 | motor powers (L, R), coupled, nonlinear and battery-dependent |
| plant mismatch | g_v U(0.55, 0.95), g_ω U(0.70, 1.05), bias U(−0.15, 0.20) rad/s | at L = R: ω = −0.69 … −1.15 rad/s. Polynomial-model residual: v 0.38–0.52 cm/s, ω 0.19–0.22 rad/s |
| reachable set | box | v 4.7–11.5 cm/s. ω > 0 only at v 7.3–10.6 cm/s; ω ≥ 0.8 only at v 8.5–9.3 cm/s; max +1.43 rad/s at (L60, R120). Single-motor pivots give ~3.2–3.7 rad/s **both ways** (INFERRED from paper figure data) but were not in the twin's grid |
| CoR offset | U(0, 4 mm), 0 nominal | **14.5 mm (CW) / 16.1 mm (CCW)**, applied in every turn (twin MPC and your own det_10 DMPC). At ω = 1.4 rad/s that adds 2.0–2.3 cm/s of point speed. Only the mean, 15.3 mm, is identifiable from steady-state data (below) |
| dynamics | instantaneous | motor 50 % / 90 % in 50 / 100 ms; legs and body UNKNOWN |
| delay | 0 | ≈ 90–260 ms (§3.3) |
| noise | process noise off in training; perfect pose | heading diffusion UNKNOWN; tracker noise ≈ 0.05–0.3 mm (ESTIMATE) |
| clock | exact | crystalless SAMD21, ±2 % on battery |

**CoR offset: only the mean is identifiable.** In the twin's kinematics (and `EllipticalBot2D`), the forward speed of the reference point is u₁ = v + d_ccw·ω⁺ + d_cw·ω⁻, with ω⁺ = max(ω, 0) and ω⁻ = max(−ω, 0). Since |ω| = ω⁺ + ω⁻ and ω = ω⁺ − ω⁻:

  d_ccw·ω⁺ + d_cw·ω⁻ = d·|ω| + ((d_ccw − d_cw)/2)·ω,  with d = (d_ccw + d_cw)/2.

- In the calibration model, ω is linear in the motor activations (f_L, f_R), and so is v(f). The asymmetric term is therefore absorbed exactly by v(f). Any (d_ccw, d_cw) with the same mean fits steady-state (L, R) → twist data equally well, with a correspondingly shifted v(f), unless v(f) is restricted by assumption.
- **The twin's 14.5 / 16.1 mm split is therefore a modelling choice**, not something this data can confirm. Use `dist_rotPoint_to_CoM = [d, d]` in `EllipticalBot2D` together with the fitted v(f); at steady state that is exactly equivalent.
- d itself is identified only through the kink of |ω| at ω = 0. The calibration grid must therefore contain both CW and CCW turns; the §5.2 grid does.
- The calibration's `cor` forward-map variant fits u₁ = v(f) + d·|ω̂| and exports d as `cor_offset_m` (§5.3). On a simulated rig with a 14.5 / 16.1 mm split it recovered 15.3 mm, the mean. That validates the code path, not the robot.

**Figure-8 feasibility.** Your test figure-8 has a 1.072 m path per lap and a 25.4 mm minimum radius of curvature at its slowest point.
- **Mean lap speed:** 6.70 / 5.36 / **3.83** cm/s at T = 16 / 20 / 28 s. T = 28 is below the 4.7 cm/s floor even on average; on average, T ≤ 22.8 s is needed.
- **CCW turns:** they need a radius of at least about 6.5–8.4 cm on the twin's envelope, so the figure-8 must be scaled about **2.6–3.3×**. Pointwise reachability is method-dependent: 62–72 % at T16, 18–19 % at T20, 0 % at T28.
- **Options:**
  - (a) enlarge ≥ 2.6–3.3× (A ≈ 0.40–0.50 m, B ≈ 0.52–0.66 m) **and re-time to a near-constant speed of ≈ 8 cm/s** (T ≈ 35–44 s; ESTIMATE).
    - Scaling alone is not enough: the tightest point is also the slowest, and tight CCW turns (ω ≥ 0.8 rad/s) only exist at 8.5–9.3 cm/s.
    - With the d·|ω| CoR term the commanded point speed rises at the lobe apexes. That leaves a usable band of only about 7.5–8.5 cm/s; 9 cm/s works only if the CoR offset is neglected.
    - This needs a ~1.2 m arena and enlarging the solver's ±0.2 m position box (`generate_solver_via_acados.py` / yaml).
  - (b) keep the size but extend the drive set with single-motor pivots, sub-60 drive and duty-cycled "gaits";
  - (c) switch to circles of R ≥ 0.15 m at 8–9 cm/s, which are feasible both ways.
- **Recompute all of this on your own bots' measured envelope.**

---

## 5. "Learning to walk": calibration and system identification

### 5.1 What the lab did, and the gaps to fix (github.com/GRASP-LAB/brainbot-digital-twin @ 9ee0aa8)

**Method:**
- one bot, a 7×7 grid of powers 60–120 (buzCw scale, INFERRED), one value per cell;
- overhead camera + ArUco;
- cubic v / quadratic ω polynomial fit, inverted by bounded least squares into two 68×490 uint8 LUTs (66.6 KB, about a quarter of the SAMD21's flash);
- run on an STM32 BLE add-on (a Nucleo-WB55, HARDWARE.md §6.2) with a double-integrator TinyMPC (N 10) and feedback linearisation v_cmd = v + d·|ω|.

**Provenance of the 7×7 tables** (checked 2026-10-28):
- The original fit is in github.com/prajolshrestha/mpc_and_imitation_learning_deployment_src @ 1c2d6a0 (March 2026), `imitation_learning_mcu_deployment_src/task/circle/acados_simple/fitting/`: quadratic v and quadratic ω. The twin's ω coefficients in `gen_inverse_map.py` are the same fit.
- The ω tables are identical in both repos. The twin's `vCOM_table_latest.npy` is re-processed: every cell is 0.01–0.10 cm/s (0.1–0.9 %) lower.
- **Layouts differ.** The repo files have rows = right power *decreasing* (120 at the top) and columns = left power increasing; both fitting scripts assume that (`L.T`, `np.rot90(R)`), and their "rows = left" comment is wrong. A third copy of the same tables (`power.npy`, `velocity_table.npy`, `omega_table.npy`, held locally, not in the repo) is the same data with the rows flipped (`np.flipud`): `[i, j]` = (R = p[i], L = p[j]). Feeding that copy to the scripts unflipped mirrors the fit in R.
- Older deployment sketches did not use the fit: they used a linear map with half-track widths 1.61 / 1.45 cm and powers 56–75 (HARDWARE.md §6.2).

**Gaps:**
- no replicates, durations, fps or battery state published;
- the cubic v-fit overfits: leave-one-out RMSE 0.52 cm/s, worse than a plane at 0.47;
- the coefficients pasted into the inverse step are truncated (error 0.38 → 0.66 cm/s);
- only about 44 % of the LUT's (v, ω) box is reachable, and the rest is silently projected. Rechecked 2026-10-28 (the 49 % stated here earlier could not be reproduced): the lab's own VL/VR tables hit 43–45 % of their targets within 0.01–0.1 cm/s and 0.001–0.01 rad/s, the hard-coded model's image covers 43.6 % of the box, and a full-precision refit 39 % (`lab_fit/`);
- the grid omits single-motor and sub-60 drive;
- the controllers' limits exceed the measured envelope (ω −3.46 to +1.43 rad/s): the learned circle policy clamps ω at ±4.5 rad/s and the acados figure-8 model bounds ω at ±5.3 rad/s and v at 0–15 cm/s, so counter-clockwise turns saturate;
- left vs right is not established: the sketches disagree on the `M<a>,<b>` order (HARDWARE.md §6.2), and the row labelling and ω sign convention are not recorded. The data fit §4.4's kinematics under one consistent reading only.

**Before reusing their firmware, check two things:**
1. `fig8.ino` advances the reference clock by 0.1 s **per received packet**. At the paper's 30 Hz that runs the T = 28 s lemniscate 3× fast, so the timing used in the experiment is unclear.
2. `figure8_trajectory.h` reference velocity signs look opposite to the derivative of its position (INFERRED from code).

### 5.2 Protocol (UNTESTED design)

**Implementation:** [calibration/README.md](calibration/README.md) implements this protocol and §5.3–5.4: calibration firmware, IR blaster, tracker, scheduler and fitting. It passes its offline suite on a simulated rig (124 tests; 123 run by default, and the EllipticalBot2D cross-check is skipped unless `GRASPION_TILEMAT_SIM` is set), but it is UNTESTED on hardware. Follow its run procedure; the steps below are the design it follows.

**Recommended design: PC-driven segments over IR.** The PC starts each segment with an addressed IR frame, the bot flashes on receipt (this also measures link latency), and the PC re-centres the bot between segments. The alternative, an open-loop onboard schedule, loses more than 50 % of segments to walls in a ~1 m arena (ESTIMATE): 3 s segments travel 17–28 cm.

0. **Rig.**
   - Deployment surface; arena ≥ 1 m with foam walls.
   - Rigid overhead camera. Global shutter at ≥ 60–120 fps with ≤ 2 ms exposure is needed for steps 4–5 and the latency test; a 30 fps rolling-shutter webcam is enough for the static map (step 3) only.
   - Flicker-free DC light; ChArUco intrinsics; 4 reference markers on spacers at marker height.
   - Right-handed world frame (image y points down, so watch the sign of ω).
1. **Per bot.**
   - Check for the Ø0.8 mm leg-plate centre pin protruding about 1.1 mm beyond the leg tips. If present, remove it (INFERRED print aid).
   - Fit the 20–22 mm ArUco on an identical cap (§3.3 placement rules); record ID ↔ chip serial; measure the marker offset from the body axis.
   - **Choose the PWM mode first.** The examples default to 732 Hz, out of DRV2603 spec; the in-spec option is 40 kHz. The twin's mode is UNKNOWN because its body firmware is unpublished. A map measured in one mode is invalid in the other, so A/B-test 5 points on bot #1.
2. **Firmware (calibration and all later phases).**
   - Build with `graspion:samd:graspionBody:opt=fastest,usbstack=tinyusb` and `#include <Adafruit_TinyUSB.h>` (HARDWARE.md §9.1), or with `opt=small` for any sketch that pulls in a BusIO-based library such as Adafruit APDS9960 or MLX90393 (they fail at `opt=fastest`); `pinMode(BUZ_ENABLE, OUTPUT)` first; 128 = stop; EN LOW when stopped.
   - **Command watchdog:** stop (128, EN LOW) when no valid, address-matched frame arrives within 2–3 frame periods (~300 ms; ESTIMATE). Optionally add a hardware watchdog (Adafruit SleepyDog) against firmware hangs. `calib_bot` adds a SysTick deadman and a hardware watchdog that cuts EN and resets the MCU at its early warning (calibration/README.md §1). Mind the UF2 bootloader: a QT Py bootloader ≥ 3.11 holds EN HIGH while it runs, including after every watchdog reset unless the firmware sets its quick-boot flag (HARDWARE.md §9.2).
   - IR receive with **address filtering** (bot ID from the chip serial). Keep D6 (IR TX) driven HIGH and the IR LED off during runs, otherwise a floating line can key ~100 mA of IR and jam every receiver.
   - Sync flash on one channel of the top NeoPixel at ~15 % only: a full-white flash plus motor start current can approach the 400 mA overcurrent trip.
   - Log mVbat from the AVR at 0x36 (verify the bytes against a multimeter). Abort the run when mVbat under load falls below ~3.4 V. Keep a USB cable at hand, because trips need USB to recover.
   - Log the MLX90393 (address 0x10, fast non-blocking settings) in every segment and at rest. The magnetometer offset map b(L, R) for Phase 2 then comes for free. Drive it over raw `Wire`, as `calib_bot` does: the Adafruit library (via BusIO) does not build at `opt=fastest` (HARDWARE.md §9.1).
3. **Bench checks.**
   - 60 s static tracking → measurement-noise floor.
   - LED-sync visibility.
   - **IR link test:** frame rate, packet loss at the arena corners and under the marker cap, and address filtering.
   - A bang-bang re-centring law (pivot, then straight) that works before any map exists.
   - The PC-side stop: motors off on marker loss, geofence violation, or solver/estimator failure.
4. **Static map.**
   - Signed grid L, R ∈ {0, 55, 60, 70, 80, 90, 100, 110, 120}: 80 cells, plus 8 reverse-polarity checks (V1.3's right motor is wired reversed).
   - 5 randomised blocks per battery, each segment 3 s drive + 0.8 s rest, with re-centring in between. With 3–5 s of re-centring, the 440 drive segments take about 50–65 min, i.e. more than one battery; re-budget per bot.
   - Cap the grid at 120 like the twin (≈ 3.07 V on the motor: above the 2.7 V rating, inside the 3.3 V maximum). Treat cells above 104 as battery-dependent.
   - Re-sync the clock at every segment (the IR start frame and the NeoPixel flash). The ±2 % clock drifts about 1 min over such a run.
5. **Dynamics.** Steps and a PRBS switching every 0.1–0.5 s, split into chunks between re-centrings → first-order lag τ and dead time per bot.
6. **Noise.** 60 s steady CW and CCW circle runs (R ≈ 5–15 cm fits the arena) → heading diffusion D_r, speed jitter, lateral slip. Straight-motion D_r comes from pooled 3 s segments: a 60 s straight run needs 3–6 m.
7. **Validation.** Command (v, ω) targets through the fitted inverse map and regress achieved on commanded. This is the dataset your domain-randomisation (DR) ranges come from.
8. **Scale.** ≥ 5 bots (8–10 better), plus one bot on 3 different days. Budget about 1 lab day per bot (ESTIMATE).

### 5.3 Models to fit

- **Per-segment body twist (u₁, u₂, ω).** Fitted by linear least squares on the tracked path. The instantaneous centre of rotation is (−u₂/ω, u₁/ω). Self-tested on your `EllipticalBot2D`: u₁ error ≤ 0.3 mm/s.
- **Forward map (L, R, V_bat) → (u₁, u₂, ω).**
  - Use a physical parametric model, f(p) = max(0, |p| − p₀)^γ per motor with twist linear in (f_L, f_R, 1), or a GP.
  - Select by grouped cross-validation. Not a 10-term cubic on 49 noisy points.
  - The calibration tooling compares four variants: `lin` (twist on [1, f_L, f_R]), `int` (+ f_L·f_R), `cor` and `bat` (+ battery terms). `cor` uses the twin's kinematics, u₁ = v(f) + d·|ω̂| with ω̂ the model's ω, and exports the mean CoR offset d as `cor_offset_m`. Only that mean is identifiable (§4).
- **Inverse map + feasible set.**
  - A closed-form 2×2 solve needs no LUT and is MCU-friendly.
  - Export the feasible set as a polytope a·v + b·ω ≤ c for the MPC constraints and the RL action projection.
- **Dynamics and noise.** Lag τ, dead time, D_r (OU process if ω is correlated), plus measurement noise.
- **Variation.** Hierarchical robot / day / segment model, whose variance decomposition gives the DR ranges.

### 5.4 Feeding it back into `rl_arch_bench` (your code; nothing was changed)

Add a calibrated-plant env mixin that:
1. projects (v, ω) onto the feasible polytope;
2. computes (L, R) = G(v, ω), quantised;
3. computes the twist from F_θ(L, R, V);
4. applies the lag and a k-step delay, with control dt = the real link period plus jitter (≈ 0.11–0.2 s over IR);
5. adds measured noise;
6. integrates at the tracked point, with tracker noise.

The calibration tooling's `calib.json` holds the polytope, inverse map, forward map, lag, dead time, latency and noise; the bootstrap draws are in `forward_map_bootstrap.json` next to it (calibration/README.md §6).

Then:
- Replace the gain/bias DR with posterior-predictive draws of F_θ.
- Replace CoR U(0, 4 mm) with the fitted mean offset d (`cor_offset_m`), as `dist_rotPoint_to_CoM = [d, d]` together with the fitted v(f). Do not import the twin's 14.5/16.1 mm split: steady-state data cannot distinguish it from [d, d] (§4).
- Enable process noise.
- Replace the ±5.3 rad/s box with the feasible set.
- Regenerate references inside the envelope.

**Option worth trying:** train with **(L, R) as the action** directly, with no inverse map. The `manc_vnc` chain DN → IN → leg MN → 2 maps naturally onto "motor neurons → two motors".

**Sim-to-real gate:**
- replay logged PRBS commands in the calibrated sim and compare error distributions at 1 s and 3 s;
- the nominal MPC's RMSE on the real bot and in the sim must agree within CIs;
- hold out one whole robot.

### 5.5 Online "keep learning to walk"

- **RLS on the 6–8 map parameters per bot**, with a forgetting factor of about 0.99–0.999 per segment. This tracks battery drift and leg wear. Store the parameters in flash keyed by chip serial (the SAMD21 has no EEPROM).
- **Active learning / contextual BO** to choose the next (L, R). Robot #1 gets the full grid; later robots use the population prior and maybe 60–100 segments instead of 440 (ESTIMATE).
- **Hindsight supervised residual targets** instead of on-robot PPO, then fine-tune briefly. Two changes to your code are needed:
  - `compute_target_residual` (`residual_env.py`) currently inverts the privileged gain/bias plant; rewrite it to use the calibrated or RLS map and the achieved twist;
  - the supervised trainer fits MSE on the raw actor output while deployment applies tanh. Train through tanh, or regress on atanh of the targets clipped below 1.
- **Duty-cycled gaits** (on/off bursts ≥ 0.2 s) to reach speeds below the dead zone.

---

## 6. Recommended roadmap

| phase | goal | brain / link | new hardware |
|---|---|---|---|
| **0: learn to walk** (weeks) | First: IR link bench test and watchdog firmware. Then calibrate and identify ≥ 5 bots (8–10 better; §5). Measure loop latency with an LED round-trip | PC + camera; IR NEC downlink for segment starts and re-centring | overhead camera, IR blaster, rig, lights |
| **1: external brain, one bot** | Closed-loop figure-8 on a *feasible* geometry, in this order: preview_warm MPC + delay compensation → residual (retrained on the calibrated plant) → standalone actors incl. flywire_dn / manc_vnc vs. their shuffles. Report real-robot RMSE with `test_v1`-style metrics | PC (acados) → IR NEC at ~5–9 Hz (retrain and regenerate at that dt), or the **twin's STM32 BLE add-on** (≥ 10 Hz; new add-on firmware and body parser needed) | optionally an nRF52840 dongle |
| **2: brain on the body** | Actor or distilled MPC on the SAMD21; hybrid pose broadcast + magnetometer (offset map from Phase 0) + dead reckoning, with fix age known (TDMA slots), a phase-sync word and a per-bot clock-rate correction | onboard + IR/BLE fixes | none, or a BLE-UART module on X3 |
| **3: swarm (5–20)** | **Hierarchical:** your `det_10` Dt-HOCBF DMPC (already 20 brainbots in MuJoCo via ROS 2 `cmd_vel`) with a per-bot `cmd_vel` → (L, R) map; or two-level goals at 0.5–2 Hz. **Emergent:** onboard rules (Kuramoto colour sync, phototaxis under a modulated light field, compass alignment), with the camera only measuring | PC + onboard | an nRF per bot (2026 head or module), 1–4 nRF52840 dongles, more cameras beyond ~1.4 m |

**Parts for Phase 0 and 1:**

| item | details |
|---|---|
| Camera | A USB3 global-shutter camera at 100–200 fps (machine-vision class, or a budget OV9281 UVC module, which needs an IR-cut filter). Prefer a **colour** sensor: the OV9281 is mono, and on a mono camera the calibration tracker can log only LED brightness, so the red low-battery flash looks like a sync flash and the PC-side low-battery stop does not work (calibration/README.md §2). A 1080p webcam with manual exposure is enough only for the static map, not for dynamics, noise or latency |
| Rig and light | Aluminium-extrusion overhead rig; flicker-free DC LED panels |
| Markers | ArUco 4x4_50 caps, 20–25 mm |
| IR blaster | 4–8× TSAL6200 940 nm LEDs + logic-level MOSFET, driven by an Arduino Nano / RP2040 running IRremote as a USB-serial bridge; optionally one overhead TSOP75438 for uplink |
| Bench tools | Scope (DRV2603 at 732 Hz vs 40 kHz); 0.01 g scale |

---

## 7. Swarm outlook (per-bot update rate, ESTIMATE)

| link | N = 1 | N = 5 | N = 10 | N = 20 |
|---|---|---|---|---|
| IR NEC/Onkyo, time-shared | 5–9 Hz | 1–1.8 Hz | 0.5–0.9 Hz | 0.25–0.45 Hz |
| BLE advertising broadcast | ~40 Hz | ~40 Hz | ~20 Hz | ~10 Hz |
| BLE connections, one nRF52840 central | ≥ 10–30 Hz (the twin add-on reported 30 Hz for its own link) | ≥ 10 Hz | ~10 Hz | needs 2–4 centrals |
| one overhead camera | 30–200 fps for all bots, about a 1–1.4 m arena per camera | | | |

**Conclusions:**
- IR suits hierarchical goal commands, not tight per-bot tracking, beyond 1–2 bots.
- Hybrid (onboard control + low-rate pose fixes) or BLE broadcast is the scalable route.
- GRASPion moves about 10× faster than a Kilobot, so ARK-style message bundling leaves bigger open-loop gaps. The onboard estimator therefore matters more.
- **Calibration does not scale by hand.** Use the population prior + per-bot RLS and a 1-minute check per session. Rubenstein et al. (Kilobot, *Robotics and Autonomous Systems* 2014) and Pinciroli et al. 2018 describe manual per-robot calibration as tedious or cumbersome, and Ben Zion et al. remark (citing other work) that it drifts.

---

## 8. Papers (verified on primary pages)

**The five most relevant:**

1. **Mammadli, Shrestha, Pande, Novkoski, Mohapatra, Noirhomme, Maier, Vandewalle, Smith.** "Physics-informed digital twin and onboard control of a brainbot for intelligent active matter." *Phys. Rev. Research* 8, 033164 (2026). doi:10.1103/rd1t-h2v6, arXiv:2511.00384; code: github.com/GRASP-LAB/brainbot-digital-twin.
   - Same lab and robot family, same figure-8 task, same pipeline you propose: camera-based (v, ω) → (L, R) calibration, then MPC with camera pose sent over BLE (30 Hz reported) to onboard TinyMPC.
   - No learned policy, no quantitative tracking error (only a "small error" and an asymmetry are reported), and no multi-robot work. The paper itself names NN controllers trained on twin trajectories and multi-bot interactions as future work, so this opening is shared with the lab: coordinate with them.
2. **Pinciroli, Talamali, Reina, Marshall, Trianni.** "Simulating Kilobots within ARGoS: models and experimental validation." ANTS 2018, LNCS 11172, pp. 176–187. doi:10.1007/978-3-030-00533-7_14.
   - Template for *calibration → simulator*: 120 pre-calibrated vibration robots were tracked by an overhead camera in 60 s straight-motion runs.
   - A per-robot Gaussian bias (SD 1.86 mm/s) was estimated and injected, with ARGoS default per-step noise, and simulated and real mean-square displacement matched.
   - These were straight runs on a robot with 3 discrete motion modes, so your continuous (v, ω) map extends this recipe.
   - Shows that calibrated Kilobots still rarely move straight or at nominal speed. That argues for calibrated DR and online residuals.
3. **Reina, Cope, Nikolaidis, Marshall, Sabo.** "ARK: Augmented Reality for Kilobots." *IEEE RA-L* 2(3):1755–1761 (2017). doi:10.1109/LRA.2017.2700059.
   - Blueprint for an **external brain over a vibration-driven swarm**: overhead cameras plus an overhead IR broadcaster, ID assignment, positioning and virtual sensors.
   - Demos with 50–100 robots.
   - Caveat: automatic motor calibration is "enabled but not included" in the paper.
4. **Gebhardt, Daun, Schnaubelt, Neumann.** "Learning Robust Policies for Object Manipulation with Robot Swarms." ICRA 2018, pp. 7688–7695. doi:10.1109/ICRA.2018.8463215.
   - The closest precedent for a **learned policy as an external brain over real vibration robots**, transferred from sim to real Kilobots.
   - Hierarchical: a planner plus a learned low-level policy. The swarm representation is permutation- and size-invariant.
   - Control is through a projected light field. GRASPion's TEMT6202 makes that channel available too.
5. **Ben Zion, Fersula, Bredeche, Dauchot.** "Morphological computation and decentralized learning in a swarm of sterically interacting robots." *Science Robotics* 8(75), eabo6140 (2023). doi:10.1126/scirobotics.abo6140.
   - The **brain-on-body, emergent route**: 64 physical vibration robots (Kilobots on resonant tripod exoskeletons) run distributed, online evolutionary RL of a phototaxis perceptron.
   - Also remarks, without measuring it (citing other work), that per-robot vibrator calibration is lengthy and drifts. The authors avoid calibration mechanically.

**Also worth citing:**
- Becker et al., *IJRR* 33(13):1626–1644 (2014): steer many differently-calibrated robots with one broadcast (v, ω) plus online gain calibration. Wheeled robots.
- Lechner, Hasani, Grosu, "Neuronal Circuit Policies", arXiv:1803.08554 (2018, preprint, not peer-reviewed): the fixed C. elegans tap-withdrawal circuit, with parameters learned by search-based RL, parked a real rover. The closest precedent for "connectome-masked policy on hardware".
- Lechner, Hasani, Zimmer, Henzinger, Grosu, "Designing worm-inspired neural networks for interpretable robotic control", ICRA 2019, pp. 87–94, doi:10.1109/ICRA.2019.8793840: the peer-reviewed follow-on; compact networks designed from worm-inspired neuron-pair motifs (not a fixed connectome), trained supervised.
- Robinson et al., *Sci. Rep.* 12:3210 (2022): a Drosophila central-complex ring attractor on a TurtleBot.
- Eschmann, Albani, Loianno, "Learning to Fly in Seconds", *RA-L* 9(7) (2024): tiny actor exported to C on a Crazyflie MCU.
- Liu et al., arXiv:2512.24740 (2025): an RL policy under Cortex-M0 compute limits, with int8 analysis.
- Alavilli et al., "TinyMPC", ICRA 2024 (arXiv:2310.16985).
- Jeon et al., "Residual MPC", arXiv:2510.12717 (2025): residual RL on MPC deployed on hardware.
- Smith, Kostrikov, Levine, "A Walk in the Park", arXiv:2208.07860 (2022); demo version Kostrikov, Smith, Levine, "Demonstrating A Walk in the Park", RSS 2023, doi:10.15607/RSS.2023.XIX.056: a real Unitree A1 learned to walk from under 20 k samples (about 20 min).

**Calibration ("learning to walk", §5); added 2026-10-21.** No published paper gives a complete per-bot calibration protocol for GRASPion; §5 and [calibration/](calibration/README.md) assemble one from the pieces below. Entries marked *(abstract only)* were checked on an abstract or index page, not the full text.
- **The lab's own papers:**
  - Noirhomme et al., "Brainbots as smart autonomous active particles with programmable motion", *Phys. Rev. Applied* 23, 064008 (2025), arXiv:2411.01943. The most detailed motion-characterisation method, on the one-motor predecessor: overhead camera with an OpenCV body-centre and long-axis tracker, Savitzky–Golay smoothing and differentiation, instantaneous centre of rotation from v = ω × (r − r_c), and a spin-vs-walk index η = |ω|·l_leg/|v| swept over motor voltage and leg angle (5–25°). Frame rate and filter settings are not given.
  - The GRASPion paper (HARDWARE.md header) has **no calibration procedure**: only the single-motor rotation (Fig. 4, ~5 mm drift) and the diffusive MSD (Fig. 5), with no tracking details.
  - Mammadli et al. (paper 1 above): the (v, ω) → (L, R) lookup-table step is only in **arXiv v2 (2026-05-21) / the journal version**, §IV, and only at a high level (measure over (L, R) combinations, nonlinear regression, least-squares inversion; no grid, fps or error metrics). arXiv v1 has none of it. The details are only in `brainbot-digital-twin` (§5.1).
- **Bristlebot physics (forward-map form, dead zone, lag):**
  - Cicconofri & DeSimone, "Motility of a model bristle-bot: a theoretical analysis", *Int. J. Non-Linear Mech.* (2015), doi:10.1016/j.ijnonlinmec.2014.12.010, arXiv:1410.8153 *(abstract only)*: closed-form mean speed vs vibration frequency, and the frequency at which the direction reverses.
  - Giomi, Hawley-Weld, Mahadevan, "Swarming, swirling and stasis in sequestered bristle-bots", *Proc. R. Soc. A* 469, 20120637 (2013), arXiv:1302.5952 *(abstract only)*: one per-bot parameter, the spin-to-walk ratio, sets the collective state, so it is worth calibrating per bot.
  - Scholz, Jahanshahi, Ldov, Löwen, "Inertial delay of self-propelled particles", *Nat. Commun.* 9, 5156 (2018), doi:10.1038/s41467-018-07596-x: vibrated particles reach steady speed in ~0.1 s, and an underdamped active-Brownian model explains the orientation–velocity lag. Supports fitting τ (§5.2 step 5) instead of assuming an instant response.
- **Calibration practice on vibration-robot swarms:**
  - Rubenstein, Ahler, Hoff, Cabrera, Nagpal, "Kilobot: A low cost robot with scalable operations designed for collective behaviors", *Robotics and Autonomous Systems* (2014). "Lessons learned" (§5): manual sensor and motor calibration is tedious and hard to automate; future collectives should calibrate automatically. The hands-on per-robot procedure (left / right / straight values tested in KiloGUI, stored in EEPROM) is in github.com/acornejo/kilobot-labs.
  - Pickem et al., "Safe, Remote-Access Swarm Robotics Research on the Robotarium", arXiv:1604.00640 (2016) *(abstract only)*: automatic registration with an overhead tracker and automatic sensor calibration on a remote swarm testbed. Wheeled robots.
- **Reusable methods for single steps:**
  - Antonelli, Chiaverini, Fusco, "A systematic calibration method for odometry of mobile robots based on the least-squares technique: theory and experimental validation", *IEEE Trans. Robotics* 21(5):994–1004 (2005) *(abstract only)*: differential-drive kinematic parameters by linear least squares from camera measurements, on any path, using continuous-time kinematics. The closest general template for fitting (L, R) → twist, with motor commands in place of wheel encoders.
  - Vasconcelos, Elkaim, Silvestre, Oliveira, Cardeira, "Geometric approach to strapdown magnetometer calibration in sensor frame", *IEEE TAES* 47:1293–1306 (2011), doi:10.1109/TAES.2011.5751259 *(abstract only)*: hard- and soft-iron correction by ellipsoid fitting. A planar bot only spins about z, so on the MLX90393 this reduces to an ellipse fit during in-place turns (INFERRED), plus the motor-induced offset map b(L, R) from §5.2 step 2.
  - Garrido-Jurado, Muñoz-Salinas, Madrid-Cuevas, Marín-Jiménez, "Automatic generation and detection of highly reliable fiducial markers under occlusion", *Pattern Recognition* 47(6):2280–2292 (2014), doi:10.1016/j.patcog.2014.01.005 *(abstract only)*: the ArUco marker system used by the tracker.
- **Data-driven map fitting: the minimal reading list for the calibration experiments** (added 2026-10-27; found and checked on primary or bibliographic pages by a literature search, and the arXiv entries re-checked by hand; venues for the arXiv entries are from the literature search, as their arXiv pages name none):
  - Nagabandi, Yang, Asmar, Pandya, Kahn, Levine, Fearing, "Learning Image-Conditioned Dynamics Models for Control of Under-actuated Legged Millirobots", IROS 2018, arXiv:1711.05253. The closest published experiment: a legged millirobot without odometry, tracked externally; a neural-network forward model learned from under 17 min of random-command data drives MPC. Use it as the main comparison: add a small NN forward map to the §5.3 cross-validation and compare data budgets (theirs < 17 min, the §5.2 grid ≈ 50–65 min per bot).
  - Seegmiller, Rogers-Marcovitz, Miller, Kelly, "Vehicle Model Identification by Integrated Prediction Error Minimization", *IJRR* 32(8):912–931 (2013) *(abstract only)*. Fits the motion model and its lag jointly from low-rate pose data by minimising integrated, multi-step prediction error. The criterion behind the §5.4 sim-to-real gate (replayed PRBS, errors at 1 s and 3 s), and an alternative fit for lag and dead time in §5.2 step 5.
  - Martínez, Mandow, Morales, Pedraza, García-Cerezo, "Approximating kinematics for tracked mobile robots", *IJRR* 24(10):867–878 (2005), doi:10.1177/0278364905058239 *(abstract only)*. Slip-dominated two-track vehicles modelled by instantaneous centres of rotation that stay in a bounded region, fixed at experimentally identified, terrain-specific values. The same model family as the `cor` variant and the single identifiable mean CoR offset (§4).
  - Censi, Franchi, Marchionni, Oriolo, "Simultaneous Calibration of Odometry and Sensor Parameters for Mobile Robots", *IEEE T-RO* 29(2):475–492 (2013), doi:10.1109/TRO.2012.2226380. Closed-form maximum-likelihood calibration with an observability analysis and a Cramér–Rao bound. Tools for the identifiability argument in §4, for choosing the (L, R) grid, and for parameter error bars.
  - Xiao, Biswas, Stone, "Learning Inverse Kinodynamics for Accurate High-Speed Off-Road Navigation on Unstructured Terrain", *IEEE RA-L* 6(3) (2021), arXiv:2102.12667. Learns the inverse map (desired motion plus context → command) directly from driving data. Template for Phase 1: a learned residual on top of the closed-form (v, ω) → (L, R) inverse, trained on closed-loop logs.

**Gap.** Within about 40 web searches (not a systematic review), no paper was found that runs deep-RL/NN tracking or locomotion policies, residual RL on MPC, or FlyWire/MANC-derived controllers on a physical bristlebot or brainbot. Learned policies on vibration robots appeared only as a central light-field policy over Kilobots (Gebhardt 2018) and as onboard evolutionary RL of a phototaxis threshold (Ben Zion 2023). A real-robot version of your architecture comparison therefore appears to be new. Some candidates could not be opened (e.g. Ratchair; Mousavi & Fakhari 2025 on vibration-robot tracking); check them by hand.

**How this work compares, stage by stage** (added 2026-10-28). The methodology is the standard one, and the lab's: measure the motion → identify a model → invert it for actuation → build a calibrated simulator → control or learn → adapt online. Most stages here use a different method, and a few add something the lab and the papers above do not. The lab has real-robot results; everything here is a design or runs on a simulated rig.

| stage | lab code and literature | this work | relation |
|---|---|---|---|
| Tracking | Lab v1: ellipse fit on the bot outline at ~35 fps; lab v2: ArUco, pose over BLE (HARDWARE.md §8). Noirhomme 2025: OpenCV body axis, Savitzky–Golay. None report latency or filter settings | ArUco, ChArUco intrinsics, homography at marker height (parallax), LED-flash time sync, measured link latency (§3.3, calibration/README.md) | same method family, timing made explicit |
| Excitation | Lab: 7×7 grid, one bot, one value per cell, no replicates (§5.1). Nagabandi 2018: < 17 min of random commands. Pinciroli 2018: 60 s straight runs on 120 Kilobots | 9×9 signed grid including single-motor and dead-zone cells, randomised blocks with replicates, steps, PRBS and circles; ≥ 5 bots plus one bot on 3 days (§5.2) | same methodology, designed and replicated; about 3–4× Nagabandi's data per bot (50–65 min) |
| Forward model | Lab: black-box cubic v / quadratic ω polynomials. Twin: geometric CoR model. Antonelli 2005, Martínez 2005: grey-box least squares with an ICR. Nagabandi: neural network | Grey-box dead zone + power law per motor, twist linear in activations, `cor` and `bat` variants, grouped cross-validation, optional GP; an NN is the planned comparison (§5.3) | same as Antonelli / Martínez; the lab's cubic overfits (§5.1) |
| Identifiability | Twin fits separate CW / CCW CoR offsets (14.5 / 16.1 mm) | Only their mean is identifiable from steady-state data (§4; observability as in Censi 2013) | goes beyond the lab |
| Inverse map | Lab: bounded least squares into a 68×490 LUT; older sketches a hand-tuned linear map (HARDWARE.md §6.2). Xiao 2021: learned inverse | Closed-form 2×2 solve and an exported feasible polytope for the MPC and RL (§5.3) | same goal, different method |
| Lag and dead time | Lab: none, or set by hand (acados τ_v 0.1 s, τ_ω 0.05 s). Scholz 2018: vibrated particles settle in ~0.1 s | Fitted per bot from steps and PRBS (§5.2 step 5) | a stage the lab lacks |
| Calibrated simulator | Twin: kinematic model reproducing trajectory statistics. Pinciroli 2018: per-robot bias injected, checked on MSD | Calibrated-plant env, posterior-predictive DR, gate on replayed PRBS errors at 1 s and 3 s (Seegmiller 2013) and a held-out robot (§5.4) | same as Pinciroli, predictive instead of statistical validation |
| Control | Lab: TinyMPC onboard a Nucleo-WB55, acados MPC, and an MPC-distilled student network on the WB55 (below) | acados MPC, residual RL on MPC, standalone actors incl. connectome-masked ones, an MPC-distilled MLP option (§1, §2) | same family; residual RL and connectome actors on a real bristlebot have no precedent found (Gap above) |
| Online adaptation | Becker 2014: online gain calibration (wheeled). Kilobots: manual and drifting (Rubenstein 2014). Ben Zion 2023: avoids calibration mechanically | Per-bot RLS, active learning of the next (L, R) (§5.5), an RMA-style adaptive low-level policy (§10.3) | same ideas, new on bristlebots |
| VLA | VLAs are shown on manipulators (§10.6). Gebhardt 2018: planner + learned low-level policy over Kilobots | VLA over an adaptive low-level policy, play data relabelled as goals (§10) | standard robot-learning methodology on a new embodiment. Prior VLA work on vibration robots was **not searched** |

**Prior art in the lab's own code: an MPC-distilled student policy.** `prajolshrestha/mpc_and_imitation_learning_deployment_src` @ 1c2d6a0 deploys small networks on the Nucleo-WB55 with TFLite Micro: `imitation_learning_mcu_deployment_src/task/circle/acados_simple/models/pinn_student_policy_{32h,256h}_lambda_*.tflite` (float32 and int8, 5–266 KB). `old_stuff/.../circlenn.ino` feeds it [x, y, cos θ, sin θ] and turns its (v, ω) into motor powers with the linear map (HARDWARE.md §6.2). Every `platformio.ini` there targets `nucleo_wb55rg_p`. `old_stuff/imitation_learning_mcu_deployment_src/task/` also has figure-8 and helix variants whose model folders are named `acados`, `osqp` and `tinympc`. The folder names point to those MPCs as teachers, and the `pinn_` and `lambda` names suggest a physics-informed loss term (INFERRED from names; the training code is not in the repo). Credit it, and treat it as the baseline for this stack's distilled-MPC and actor options.

---

## 9. Decisions and information needed from you

1. **Hardware on hand.** How many bots, and V1.1 or V1.3? This also decides whether the lens-up IR receiver can be assumed. Is there any working 2026 head? Is there an overhead camera, and how large is the arena?
2. **BLE add-on.** Can you borrow the digital-twin BLE add-on (a Nucleo-WB55) and its body-side `M<a>,<b>` parser (Mammadli / Shrestha)? That decides Phase 1: IR at 5–9 Hz or BLE at ≥ 10 Hz with lower latency. Ask them also:
   - the actual packet rate (30 Hz reported, code consistent with 10 Hz) and their PWM mode;
   - the parser's motor order (their sketches send both `M<right>,<left>` and `M<left>,<right>`);
   - how the 7×7 table rows were labelled, and which way ω was signed (camera y points down);
   - what their "power" values are in `analogWrite` units.
3. **Reference geometry.** Enlarge the figure-8 (and the solver box), or extend the drive set to keep it small?
4. **Action space for retraining.** Keep (v, ω) + inverse map, or train directly in (L, R)?
5. **PWM mode.** 732 Hz (the examples' default; the twin's mode is UNKNOWN) or in-spec 40 kHz. This must be fixed before calibrating.
6. **VLA (§10).** Is a GPU of at least the RTX 4090 class available next to the rig? Which instruction set should the model follow (goal positions, shapes, other bots or objects)? Is "no per-bot calibration session" the claim to test, against a calibrated baseline?

---

## 10. A vision-language-action model that learns to drive, without explicit calibration

Added 2026-10-28. Nothing here has been built or run; numbers are ESTIMATES unless sourced.

### 10.1 Verdict

- **Yes, the explicit calibration step can go:** no grid fit, polynomial or lookup table per bot. **The data it collects cannot go.** A policy that outputs motor powers must learn the same (L, R) → motion map, from demonstrations, from random driving, or from a short history of its own commands and their effects.
- **The realistic architecture is a hierarchy** (§10.3): a VLA on the PC sets goals or target motions at a few Hz, and a small adaptive low-level policy turns them into (L, R) at the link rate, inferring each bot's quirks online from the last few seconds. That last part replaces the per-bot calibration session.
- **End-to-end VLA → (L, R) is possible but weak.** Most VLAs see one image or a short window. Per-bot differences are large (one bot drives straight at L 60, R 80; equal powers turn it at −0.7 to −1.2 rad/s, §4), so the model must see its own recent commands and motion to tell bots apart.
- **What remains mandatory** takes minutes per bot, not a calibration session: bench check 11 (motor identity and turn direction, calibration/README.md §5), the PWM mode (§9 item 5; a policy trained in one mode is invalid in the other), the dead zone (about 55–60, HARDWARE.md §4.4), and the safety layer (watchdog, deadman, geofence, re-centring; §5.2).

### 10.2 Constraints from this robot

| item | value | consequence |
|---|---|---|
| Where the VLA runs | PC GPU. OpenVLA (7 B) runs at about 6 Hz on one RTX 4090 and needs 15 GB in bfloat16 (Kim et al. 2024, §3.5). Octo has 27 M / 93 M parameters, SmolVLA 450 M (§10.6) | Nothing onboard: the SAMD21 has 32 KB RAM, and the 2026 head has 786 KB SRAM (HARDWARE.md §6.1) |
| Downlink | IR: ≤ 9 frames/s for the whole channel, maybe ~5/s; one (L, R) per frame (§3.2). BLE add-on ≥ 10 Hz | The bot executes one action per frame. Action chunks are unrolled on the PC at the frame clock, not sent as a chunk |
| Loop latency | camera + IR ≈ 245–260 ms (§3.3), plus VLA inference ≈ 170 ms at 6 Hz → ≈ 0.4 s (ESTIMATE) | At 11.5 cm/s and 3.46 rad/s the bot moves ≈ 4.6 cm and turns ≈ 1.4 rad before an action acts on the image it came from. Goal reaching is fine; ≤ 5 cm figure-8 tracking end to end is not |
| Vision | Overhead camera only; the bot has no camera | Third-person VLAs fit (Octo, OpenVLA). At 224–256 px over a 1.2 m arena (INFERRED input sizes), a bot spans about 11 × 6 px, so heading is barely visible: also feed the tracker's pose (x, y, θ, recent velocity) as state, or crop around the bot |
| Robot time | > 90 min per battery; bots differ; walls end segments (§5.2) | Pure RL on hardware is impractical (§0 item 5). Use imitation, relabelled random driving and simulation |
| Action space | 2 signed bytes (L, R), clamped to ±120 by `calib_bot`'s `DRIVE_MAX`; 0 = stop (`analogWrite` 128, EN LOW when both are 0) | Fits OpenVLA-style 256-bin action tokens. A policy must never emit raw `analogWrite` values: `analogWrite` 0 is full reverse |

### 10.3 Architecture

1. **Top: the VLA** (PC GPU, 1–6 Hz). Input: the overhead image, an instruction ("drive to the top-left corner", "circle clockwise", "follow bot 3"), and the tracker state. Output: a waypoint or a short chunk of target (v, ω). Fine-tune a small model first (Octo or SmolVLA; SmolVLA's asynchronous inference keeps the queue fed while the next chunk is computed), OpenVLA or π0-class later.
2. **Middle: an adaptive low-level policy** (PC CPU, at the IR frame clock). Input: the target, the last 2–5 s of tracker poses and of (L, R) sent. Output: (L, R). This is RMA's idea (Kumar et al. 2021): a base policy plus an adaptation module that infers the hidden plant parameters from recent history, trained only in simulation with randomised plants and deployed without real-world fine-tuning.
   - Train it on `calibration/pc/graspion_calib/sim.py`'s randomised bots, widened to cover §4: dead zone, coupling, CW bias, CoR offset, motor lag, 0.1–0.4 s delay, link loss, battery sag.
   - Fine-tune on recorded random driving (10.4). It is a tiny MLP or GRU, so it also fits the 2026 head later (§2).
3. **Bottom: `calib_bot` firmware**, unchanged: per-frame (L, R), address filter, CRC, command watchdog and deadman (calibration/README.md §1).

The VLA never outputs motor powers, so its actions mean the same on every bot. Only the middle layer deals with per-bot quirks, and it adapts within seconds instead of needing a calibration session (UNTESTED claim; measuring it is the experiment).

### 10.4 Data without a calibration fit

- **Play data** is what the calibration rig already records: random (L, R) segments, steps, PRBS and circles (`schedule grid / steps / prbs / circles`), with tracker poses, frame logs and battery logs (calibration/README.md §5 and §7). Skip `analyze`'s fit and keep the raw segments.
- **Hindsight relabelling:** every recorded stretch from pose A to pose B becomes a goal-conditioned example "go to B", and templated language turns it into an instruction (Lynch & Sermanet 2021 train language-conditioned policies on unstructured play with under 1 % of it annotated).
- **Teleoperation:** a person driving with a keyboard or gamepad acts as the inverse map and supplies demonstrations for instructions that random driving rarely produces (circles around an object, following another bot). The frames exist (a SEG, then DRIVE keep-alives at the blaster period; a DRIVE without a SEG stops the bot), but a teleop front end and its logging are **not implemented**.
- **Scale (ESTIMATE):** one battery gives about 60 min of segments; ≥ 5 bots (8–10 better) for the low-level policy to see real spread; SmolVLA was pretrained on under 30 k episodes, so plan on hundreds to a few thousand relabelled episodes per instruction family for fine-tuning, and measure.
- **Record the same metadata as calibration:** bot ID ↔ chip serial, battery voltage, PWM mode and firmware version, so the data stay usable if a fit is wanted later.

### 10.5 Plan and success criteria

1. **Bench:** bench checks 1–11 on every bot, PWM mode fixed.
2. **Simulation:** train the adaptive low-level policy on randomised plants; check it adapts to unseen plants, including the twin's measured envelope (§4).
3. **Play data:** record random driving on ≥ 5 bots with the existing schedules.
4. **Low level on the robot:** goal reaching against two baselines, the same task with a per-bot calibrated map (§5.3) and with a single fleet-average map. **"No calibration" only stands if the adaptive policy matches the calibrated baseline.** Report time-to-adapt on a new bot (seconds of driving until goal error < 3 cm) and the final error.
5. **VLA:** fine-tune on relabelled play data plus teleop demos, then evaluate language-conditioned goal reaching per instruction family, on bots held out of training.
6. **Swarm:** several bots, one camera, the IR channel divided between them (§3.2).

§5.5's online "keep learning to walk" is the same idea applied to the calibrated stack; the adaptive low-level policy makes it implicit.

### 10.6 Papers (checked on their arXiv abstract pages; OpenVLA's speed and memory from its paper, §3.5)

- Kim, Pertsch, Karamcheti et al., "OpenVLA: An Open-Source Vision-Language-Action Model", arXiv:2406.09246 (2024). 7 B parameters, 970 k real-robot demonstrations, fine-tunable on consumer GPUs with low-rank adaptation; about 6 Hz on one RTX 4090, 15 GB in bfloat16; each action dimension in 256 bins; 5 and 15 Hz controllers in its experiments.
- Kim, Finn, Liang, "Fine-Tuning Vision-Language-Action Models: Optimizing Speed and Success", arXiv:2502.19645 (2025). OFT recipe (parallel decoding, action chunking, continuous actions, L1 loss): 26× action-generation throughput.
- Octo Model Team (Ghosh, Walke, Pertsch et al.), "Octo: An Open-Source Generalist Robot Policy", arXiv:2405.12213 (2024). Octo-Small 27 M and Octo-Base 93 M parameters (project page), 800 k Open X-Embodiment trajectories, fine-tunable to new sensory inputs and action spaces.
- Shukor, Aubakirova et al., "SmolVLA: A Vision-Language-Action Model for Affordable and Efficient Robotics", arXiv:2506.01844 (2025). 450 M parameters (Hugging Face blog), trained on a single GPU, deployable on consumer GPUs or CPUs; asynchronous inference decouples prediction from execution; pretrained on 487 community datasets, under 30 k episodes.
- Black, Brown, Driess et al., "π0: A Vision-Language-Action Flow Model for General Robot Control", arXiv:2410.24164 (2024). Flow-matching action generation on a pretrained vision-language model.
- Kumar, Fu, Pathak, Malik, "RMA: Rapid Motor Adaptation for Legged Robots", RSS 2021, arXiv:2107.04034. A base policy plus an adaptation module; adapts to novel situations "in fractions of a second" and was deployed on the A1 robot without fine-tuning. The template for §10.3's middle layer.
- Lynch, Sermanet, "Language Conditioned Imitation Learning over Unstructured Data", RSS 2021, arXiv:2005.07648. Language-conditioned policies from unstructured play data, with under 1 % of it language-annotated. The template for §10.4.

---

## Sources

- **Code:**
  - `tilemat_control_stack/robots/elliptical_bot/controllers/mpc/acados_simple/`: `rl_arch_bench/{README.md, archs.py, presets.py, envs.py, mpc.py, suites.py, topologies/}`; `pure_rl_env.py`, `residual_env.py`, `Ellipse2dObj.py`, `symbolic_model_defination.py`, `generate_solver_via_acados.py`, `controller.py`, `config/solver_ellipse_mpc.yaml`.
  - The worktree `runs/full/report/{RESULTS.md, tables/}`.
  - `robots/brainbot/controllers/dmpc/acados_simple/det_10/`.
- **Lab code:**
  - github.com/GRASP-LAB/brainbot-digital-twin @ 9ee0aa8 (`generate_LUT/*`, `tinympc_deployment_code/*`, `bot_simulation/*`);
  - github.com/prajolshrestha/mpc_and_imitation_learning_deployment_src @ 1c2d6a0 (`imitation_learning_mcu_deployment_src/task/circle/acados_simple/fitting/*`, `mpc_mcu_deployment_src/task/figure8/acados/*`, `old_stuff/**/*.ino`).
- **Calibration tooling:** [calibration/](calibration/README.md) (commit fa28d91), in particular `pc/graspion_calib/forward_map.py` (CoR identifiability) and `sim.py` (the simulated rig).
- **Hardware:** [HARDWARE.md](HARDWARE.md).
- **Per-operation costs and protocol facts:**
  - Qfplib-M0 cycle table;
  - Karg et al., arXiv:1910.13906 (NN on a Cortex-M3);
  - IRremote `ir_NEC.hpp`;
  - Vishay TSOP752/754 doc 82494;
  - LigerBots camera-latency whitepaper;
  - Nordic DevZone (S132 link limits).
