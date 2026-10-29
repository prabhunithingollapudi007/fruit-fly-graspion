# Standalone calibration paper: verdict, plan and literature

Written 2026-10-29 from a design discussion and three literature searches. No controller, no fruit fly, no MPC, no RL: calibration only. Nothing below has run on hardware; every result is a hypothesis until robot #1 drives.

## 1. Verdict

**Realistic, currently over-scoped, and at risk of being empty prose until there is data.**

- **Realistic.** The venue class is reachable: Pinciroli 2018 (ANTS, 120 Kilobots, one variance number), Klingner 2014 (IROS, a black-box calibration loop), the GRASP lab's digital twin (Phys. Rev. Research, one robot, a 7×7 grid). A measured two-motor map on 6–8 robots, a variance decomposition and an error-vs-data curve is a proper RA-L / IROS / ANTS paper, or Phys. Rev. E if it leads with heterogeneity.
- **Over-scoped.** The 16 research questions (§4) are a menu. The paper is three figures (§2). The rig (IR, camera, re-centring, watchdog) is infrastructure, not the contribution: Klingner 2014 already did that loop.
- **Slop risk.** The repo has ~1,800 lines of docs, 125 offline tests, three agent-assembled literature surveys and zero measurements. Written from that, the paper is slop. The next output must be data, not documents.

**The one change to the original plan ("deep data on one robot, transfer to the rest"):** one reference robot fixes the model form, the noise level and the prior mean, but not the between-robot spread, and the spread is what lets a few segments pin down a new robot. The prior must come from several fully calibrated robots (Sorocky 2020: source choice swung target error by 62 % and could give negative transfer; PACOH; population-based SHM).

**Coordinate with the GRASP lab early** (robot, 7×7 tables and nearby papers are theirs; authorship).

## 2. The paper: three figures

1. **The map on robot #1.** (L, R, V_bat) → (u₁, u₂, ω) with replicates; model comparison by grouped cross-validation (dead zone + power law `lin` / `int` / `cor` / `bat`, polynomial, GP, small NN); the identifiability result (only the mean CoR offset is identifiable from steady-state data; parameter SDs against the Cramér–Rao bound).
2. **Variance decomposition** over 5–8 robots: robot / day / battery / segment.
3. **Error vs number of target segments** k = 0, 1, 2, 4 … 64 on held-out robots, for priors built from 1, 2, 4 (8) robots, against (i) per-robot fit from scratch, (ii) the reference model used as-is, (iii) reference model + per-robot gain / standardisation map, (iv) a Klingner-style black-box search. Random selection first; active (Bayesian D-optimal) selection only if random already works.

Everything else is optional or a second paper: active selection, magnetometer-only ω recalibration, physical causes, MSD / ISF matching, heterogeneity as a tunable knob.

## 3. Steps

**Phase A: robot #1, deep** (answers questions 1–5)
- Rig bring-up first (budget 3–6 weeks; each known unknown can eat a week): UF2 bootloader holding EN HIGH, IR link rate under the TSOP AGC, 732 Hz vs 40 kHz PWM, AVR battery byte meaning, tracker noise floor. Procedure in [calibration/README.md](../calibration/README.md) §5.
- Full 9×9 signed grid with replicates, steps + PRBS (lag, dead time), CW/CCW circles (heading diffusion), three days (drift).
- **Check the top of the grid for non-monotonic speed or direction reversal** before fixing the model form (Cicconofri 2016, Kim 2020; Swarmodroid shows vibration frequency rising roughly linearly with PWM).
- Check whether step data break the CW/CCW CoR degeneracy.

**Phase B: 5–8 robots, full grids** (answers 6–7; the critical path, ~1 lab day per robot)
- Leave-one-robot-out ground truth, variance decomposition, population prior (hierarchical Bayes: once p₀ and γ are fixed the model is linear in the twist coefficients, so the posterior after k segments is closed-form).

**Phase C: held-out robots, few segments** (answers 9–12, 15)
- The figure-3 curve. Target: ~75 segments (~10 min) reaching ~90 % of full-grid accuracy; that number is the experiment, not a fact.
- Validation without a controller: twist error on held-out random cells; integrated open-loop pose error vs horizon (Seegmiller 2013); predictive-interval coverage; "same commanded (v, ω), same motion" across robots on the intersection of their feasible sets; MSD fitted on log-MSD with bootstrap CIs (Bailey 2022).

**Pre-registered kill criteria**
- Between-robot spread within tracker noise → figure 3 is flat; the paper becomes "one map fits all" (smaller).
- Spread huge and unstructured → the prior does not help; the paper becomes a drift-and-heterogeneity measurement (smaller).
Decide now what gets written in each case.

## 4. Research questions and their status

| # | Question | Status |
|---|---|---|
| 1 | Simplest model predicting motion from (L, R)? | Open. Nearest work is single-motor (Swarmodroid). No measured two-motor map exists; the GRASPion platform paper gives none. |
| 2 | Do the motors add linearly, or interact? | Open. Notomista 2019 derives v = (v_L+v_R)/2, ω = (v_R−v_L)/W for a differential brushbot with no data on that variant. The `int` variant tests it. Beat/resonance is a hypothesis. |
| 3 | What is identifiable? | Partly ours (mean CoR, FEASIBILITY.md §4; holds for steady-state data with v(f) in the same basis as ω). ICR offsets known to be partly observable (Martínez 2005, Yi 2009). Open: transient data. |
| 4 | Active Brownian with drive-dependent parameters? | Partly answered (Kiechl 2026 hexbug vs ABP; Scholz 2018 inertial delay; Siebers 2023 per-unit fits). Open: per-robot, drive-dependent D_r and lag. |
| 5 | Drift with battery, warm-up, wear, surface? | Qualitative only (Raoufi 2023: Kilobot calibration is "only temporary"; Kilombo, Ellis 2026). Numbers open. |
| 6 | Robot vs day vs segment variance? | Open. Pinciroli 2018 gives one number (bias SD 1.86 mm/s over 120 Kilobots); Siemensma 2024 hand-tuned distributions for 5 wheeled robots. Mandatory: it is the prior's covariance. |
| 7 | Is the variability low-dimensional? | Open, underpowered at 5–10 robots. Report spreads and correlations only. |
| 8 | Predict parameters from physical measurements? | Exploratory; dropped from the core. |
| 9 | Which model parts are shared vs per-robot? | Open. Template: Neural-Fly (fixed basis, per-unit linear weights), ALPaCA. |
| 10 | Segments needed for ~90 % of full accuracy? | Open, headline. No data-budget curve found for swarm robots. |
| 11 | One reference robot or several? | Answered: several. Becomes the prior-from-N-robots curve. |
| 12 | Which segments are most informative? | Open. No D-optimal / Bayesian design found for mobile-robot motor maps. |
| 13 | Calibrate from the magnetometer? | Narrowed: heading only, motor-induced field b(L, R) must be corrected; at best the ω part. Low priority. |
| 14 | Open-loop prediction horizon, MSD? | Methods exist (Seegmiller 2013, Bailey 2022); applying them here is open. |
| 15 | Same command → same motion after calibration? | Open; standard validation. Compare on the intersection of feasible sets. |
| 16 | Heterogeneity as a tunable knob? | Open. Siebers 2023 exploit disorder; Giomi 2013: one per-bot ratio sets the collective state. Active-matter positioning. |
| + | Does PWM mode (732 Hz vs 40 kHz) change the map's shape or only its gain? | Open; A/B on robot #1. If gain only, maps transfer across modes. |

Corrections made during re-verification: Q13 was overstated (magnetometer gives heading only); Q7–8 are underpowered at 5–10 robots; Q4 is partly answered; Q14 must use the log-MSD fit, not OLS.

## 5. Literature (three searches, 2026-10-29; not systematic)

Entries were checked on a primary page (arXiv, publisher, DOI). "(abstract only)" = not read beyond the abstract. **Re-read anything before citing it.** IEEE Xplore full texts and 2026 arXiv were thinly covered. Entries already listed in [FEASIBILITY.md](../FEASIBILITY.md) §8 (Mammadli 2026, Noirhomme 2025, Pinciroli 2018, Reina 2017, Rubenstein 2014, Ben Zion 2023, Giomi 2013, Cicconofri & DeSimone 2015, Scholz 2018, Nagabandi IROS 2018, Censi 2013, Martínez 2005, Seegmiller 2013) are not repeated.

**Pre-empting / closest prior art**
- Klingner, Kanakia, Farrow, Reishus, Correll, "A stick-slip omnidirectional powertrain for low-cost swarm robotics: mechanism, calibration, and control", IROS 2014, doi:10.1109/IROS.2014.6942658 (abstract only). Per-robot Nelder–Mead calibration of vibration-motor Droplets with IR commands and a webcam. **Pre-empts the automated rig as a contribution; baseline for figure 3.**
- Dmitriev et al., "Swarmodroid & AMPy", arXiv:2305.13510. Single-motor bristle-bots, ArUco tracking, speed monotone in PWM, dispersion grows with PWM, seven robots per type, vibration frequency ≈180 Hz at 10 % to ≈385 Hz at 50 % PWM. No two-motor map, inverse or population model.
- Novkoski et al., "GRASPion: an open-source, programmable brainbot for active matter research", Rev. Sci. Instrum. 97, 014704 (2026), doi:10.1063/5.0303152, arXiv:2509.07437 (full text). No speed/yaw vs PWM, no variability, no battery dependence.

**Vibration-robot physics (model form warnings)**
- Notomista, Mayya, Mazumdar, Hutchinson, Egerstedt, "A study of a class of vibration-driven robots: … the Brushbot", IROS 2019, doi:10.1109/IROS40897.2019.8968490, arXiv:1902.10830. Differential-brushbot kinematics from first principles; only the fully actuated robot was validated.
- Vartholomeos & Papadopoulos, ICRA 2006, doi:10.1109/ROBOT.2006.1641784 (abstract only). Earliest two-vibration-motor differential analysis.
- Cicconofri, Becker, Noselli, DeSimone, Zimmermann, "The inversion of motion of bristle bots", ROMANSY 21 (2016), doi:10.1007/978-3-319-33714-2_25, arXiv:1702.00343 (abstract only). Direction reverses with frequency.
- Kim, Hao, Mohazab, Ansari, "On the forward and backward motion of milli-bristle-bots", Int. J. Non-Linear Mech. (2020), doi:10.1016/j.ijnonlinmec.2020.103551, arXiv:2002.10344 (abstract only). Dry-friction model: thresholds, separate forward/backward bands, resonances.
- Kiechl, Altshuler, Lüders, Roichman, Franosch, "Free chiral self-propelled robots compared to active Brownian circle swimmers", Phys. Rev. E 113, 045409 (2026), arXiv:2604.05723. Hexbug matches ABP circle-swimmer MSD and ISF.
- Bailey, Sprenger, Grillo, Löwen, Isa, "Log it: how to fit an active Brownian particle's MSD", Phys. Rev. E 106, L052602 (2022), doi:10.1103/PhysRevE.106.L052602. OLS on the MSD is biased; use log-MSD and bootstrap CIs.

**Heterogeneity and drift in swarms**
- Raoufi, Romanczuk, Hamann, "Individuality in swarm robots with the case study of Kilobots", ALIFE 2023, doi:10.1162/isal_a_00623, arXiv:2305.16063. Persistent per-robot heading bias; calibration "only temporary". No speed numbers.
- Siebers, Jayaram, Blümler, Speck, "Exploiting compositional disorder in collectives of light-driven circle walkers", Sci. Adv. 9 (2023), doi:10.1126/sciadv.adf5443. 20 walkers fitted individually as ABPs; hard drive threshold.
- Siemensma & Haghighat, arXiv:2412.14646 (2024). Wheeled; hand-tuned per-robot parameter distributions from 5 robots.
- Ellis et al., Front. Robot. AI 13 (2026), doi:10.3389/frobt.2026.1828495; Jansson et al., Kilombo, arXiv:1511.04285: Kilobots still hand-calibrated and frequently recalibrated. "KiloCali" (Sheffield wiki) exists only as a search snippet: UNVERIFIED.

**Transfer across a fleet (method family: hierarchical / empirical Bayes with a fixed physical basis)**
- Bull et al., "Foundations of population-based SHM, Part I", MSSP 148, 107141 (2021), doi:10.1016/j.ymssp.2020.107141.
- Bull et al., "Hierarchical Bayesian modeling for knowledge transfer across engineering fleets via multitask learning", CACAIE (2022), doi:10.1111/mice.12901. Sharing cut parameter SD by 70 % (truck fleet) and 32 % (wind farm).
- Dardeno et al., "On the hierarchical Bayesian modelling of frequency response functions", MSSP 208, 111072 (2024), doi:10.1016/j.ymssp.2023.111072 (abstract only). Small population of nominally identical blades.
- O'Connell et al., "Neural-Fly", Sci. Robotics 7, eabm6597 (2022), arXiv:2205.06908. Fixed learned basis + per-unit linear weights; basis from one drone transferred to another (3.7 vs 3.2 cm RMS at 0 wind).
- Harrison, Sharma, Pavone, "ALPaCA", WAFR 2018, doi:10.1007/978-3-030-44051-0_19, arXiv:1807.08912 (abstract only). Meta-learned Gaussian prior on last-layer weights, closed-form online update.
- Rothfuss, Fortuin, Josifoski, Krause, "PACOH", ICML 2021, arXiv:2002.05551 (abstract only). Few meta-training tasks = prior overfitting.
- Helwa & Schoellig, "Multi-robot transfer learning: a dynamical system perspective", IROS 2017, arXiv:1707.08689. One source drone → one target with 6 min of source data; 68–70 % lower error than direct transfer.
- Sorocky, Zhou, Schoellig, "Experience selection using dynamics similarity …", ICRA 2020, arXiv:2003.13150. Five sources; best vs worst source differ by 62 %; negative transfer possible. **Why one reference robot is not enough.**
- Williams, Klanke, Vijayakumar, Chai, "Multi-task GP learning of robot inverse dynamics", NIPS 2008 (abstract only).
- Wang, Veltkamp, Kowalski, "Multivariate instrument standardization", Anal. Chem. 63, 2750 (1991), doi:10.1021/ac00023a016 (metadata only); reviews Feudale 2002, Workman 2018. Master/slave calibration transfer with transfer samples: the one-reference-robot fallback.
- Nagabandi et al., "Learning to adapt …", ICLR 2019, arXiv:1803.11347; Belkhale et al., RA-L 2021, doi:10.1109/LRA.2021.3057046 (abstract only); Kumar et al., RMA, RSS 2021, arXiv:2107.04034 (abstract only); Chakrabarty et al., IFAC WC 2023, arXiv:2211.07768 (abstract only); Forgione, Pura, Piga, IEEE L-CSS 7 (2023), doi:10.1109/LCSYS.2023.3335036; Xiao et al., AnyCar, arXiv:2409.15783 (abstract only). Neural meta-learning: overkill for ~10 parameters.

**Experiment design, identifiability, validation**
- Chaloner & Verdinelli, "Bayesian experimental design: a review", Stat. Sci. 10, 273 (1995), doi:10.1214/ss/1177009939.
- Rainforth, Foster, Ivanova, Bickford Smith, "Modern Bayesian experimental design", Stat. Sci. 39, 100 (2024), doi:10.1214/23-STS915, arXiv:2302.14545.
- Sun & Hollerbach, "Observability index selection for robot calibration", ICRA 2008, doi:10.1109/ROBOT.2008.4543308. D-optimal for parameters, E/G/I-type for prediction error.
- Buisson-Fenet, Solowjow, Trimpe, "Actively learning Gaussian process dynamics", L4DC 2020, arXiv:1911.09946.
- Schultheis, Belousov, Abdulsamad, Peters, "Receding horizon curiosity", CoRL 2019, arXiv:1910.03620.
- Preiss, Hausman, Sukhatme, Weiss, "Trajectory optimization for self-calibration and navigation", RSS 2017, doi:10.15607/RSS.2017.XIII.054.
- Das, Touma, Burdick, "Bayesian optimal experimental design for robot kinematic calibration", arXiv:2409.10802 (manipulator).
- Mandow et al., "Experimental kinematics for wheeled skid-steer mobile robots", IROS 2007, doi:10.1109/IROS.2007.4399139; Yi et al., IEEE T-RO 25(5), 1087 (2009), doi:10.1109/TRO.2009.2026506. ICR offsets standard but only partly observable.
- Kelly, "Fast and easy systematic and stochastic odometry calibration", IROS 2004 (DOI not verified). Joint systematic + diffusion calibration.
- Martinelli, Tomatis, Siegwart, Autonomous Robots 22, 75 (2007), doi:10.1007/s10514-006-9006-7; Roy & Thrun, ICRA 1999, doi:10.1109/ROBOT.1999.770447. Kalman / incremental ML tracking of drifting kinematic parameters.

**Gap search (2026-10-29, folded into the draft and `paper/refs.bib`):** Pogobot (Loi et al., arXiv:2504.08686: 5 robots × 10 trials per PWM, wheels "most consistent across individuals", vibration units explicitly uncalibrated, no variance numbers); Patterson & Parisi 2023; hierarchical-Bayes system identification on simulated sparse data (Brugnara et al. 2026) and PAC-Bayesian few-shot LTI identification (Huang & Michailidis 2026); battery sample-size analogues (Dechent et al. 2021: 9–13 cells for 1–3-parameter models; Zhou & Howey 2023); RAPTOR; Hammerstein / dead-zone motor identification (Kara & Eker 2004, Zhang et al. 2017, Risuleo et al. 2017); UMBmark track design (Jung & Chung 2011) and automatic Duckiebot calibration (Chaika et al. 2021). Still nothing found: a dead-zone + power-law fit to a vibration motor, a hardware error-vs-samples curve, a peer-reviewed automatic Kilobot motor calibrator, Hexbug across-unit speed statistics.

**Searches that found nothing:** a peer-reviewed automatic Kilobot motor calibrator; speed vs voltage with a dead zone measured on bristle-bots; hierarchical Bayesian calibration of a robot fleet; transfer of a vibration-robot model to other robots with few trials; D-optimal design for mobile-robot odometry or motor maps; a data-budget-vs-accuracy curve for swarm robots.

## 6. Next step

Not another document. A 60 s static-tracking log and a first `calib_bot` dump from robot #1. When segment tables exist, run the figure-2 and figure-3 analysis of §7 on them instead of on the synthetic population.

## 8. Paper draft

[paper/](paper/) is a LaTeX project (IEEEtran journal layout; `main.tex` inputs `sections/00-abstract.tex` … `10-conclusion.tex`, bibliography `refs.bib` with `IEEEtran.bst`, figures in `figures/`). It was converted once from a Pandoc Markdown draft with pandoc; the `.tex` files are the source from now on. Builds with `pdflatex main; bibtex main; pdflatex main; pdflatex main` (10 pages, no errors or overfull boxes on 2026-10-29, TinyTeX 2026 with `ieeetran` and `courier` installed; the toolchain path is in the agent memory, not on PATH). Draft v0.1 covers: abstract, introduction, a related-work section in six themes from the three searches plus a gap search, robot / rig / problem statement, the method (twist, forward map, CoR identifiability, inverse map, lag and noise, population prior and MAP few-shot fit, D-optimal segments, variance decomposition, baselines), the pre-registered protocol, the simulation study of §7, placeholders marked **[HW]** for every hardware result, discussion, limitations. Bib entries marked `abstract only` or `verify` must be re-read before submission. Revised the same day after the user objected that the abstract, discussion and conclusion stated findings with no hardware behind them: those sections now claim only the derivation, the lab-table reanalysis and that the pipeline works on a synthetic population; every simulation reading is conditional on real robots following the model family; `\hw{…}` slots name what each missing measurement is. An independent proofread (fresh agent, 2026-10-29) then found 24 overclaims, 22 consistency slips, 17 citation issues and 14 prose tics; all factual and wording items were applied (prior size renamed `S` vs fleet `N`, the lab-table reanalysis moved into a "Results without hardware" section, a pgfplots figure of the few-shot curve added, Table 3 completed with N = 6, unverifiable Ellis 2026 citation and the Bull 2022 percentages removed). Left open: condensing §2.2 and §2.5 from catalogues into an argument, reading the abstract-only references, and the author block.

## 7. Pre-hardware work (2026-10-29): the figure-2/3 analysis on a synthetic population

[popsim.py](popsim.py) is the analysis behind figures 2 and 3, run on a simulated population so that it is tested before data exist. Tests: `python calibration_paper/popsim.py selftest` (14 tests, ~6 s, repo `.venv`). Pieces: a synthetic population around `sim.default_truth()` (per-robot dead zones, exponents, motor gains, CW bias; per-day drift; segment noise), a population prior from N fully calibrated robots (shrunk covariance plus a floor that shrinks as 1/√N; N = 1 is the "one reference robot, inflated variance" fallback), a MAP few-shot fit, the baselines `scratch` / `ref` / `ref_gain` (affine standardisation on the reference), `random` and greedy Bayesian D-optimal (`dopt`) segment choice, leave-one-robot-out scoring, nested robot/day/segment variance components, and a power sweep. Results in [results/](results/). **Everything below is synthetic**: the spreads are guesses (spread 1: robot SD ≈ 0.6 cm/s in u₁ and 0.30 rad/s in ω, day SD a quarter of that, segment noise 0.3 cm/s / 0.05 rad/s), the population is drawn from the fitted model family (no misspecification, no battery term, no high-PWM reversal, no lag), noise is constant over the grid, and scores are against the noise-free truth over the whole grid.

**Variance components** (`variance`, 8 robots × 2 days × 5 replicates): recovered within about 3 % of the truth for u₁ and ω at spreads 0.3, 1 and 3; u₂ is at the noise floor in this population, so its robot component is poorly determined. Fractions at spread 1: u₁ 0.76 / 0.04 / 0.19 and ω 0.88 / 0.10 / 0.02 (robot / day / segment).

**Figure-3 curve** (`loro`, 8 robots, 3 random subsets each, 155 s). Score = RMSE over the grid in noise units (1 = 0.5 cm/s, 0.5 cm/s, 0.1 rad/s). Full 405-segment fit: 0.050. Prior mean with no data: ≈ 2.0.

| k | scratch | hier N=1 | hier N=4 | ref_gain N=4 | scratch, dopt | hier N=1, dopt | hier N=4, dopt |
|---|---|---|---|---|---|---|---|
| 2 | – | 0.98 | 0.81 | 2.17 | – | 0.91 | 0.69 |
| 4 | – | 0.67 | 0.56 | 0.96 | – | 0.39 | 0.33 |
| 8 | 1.01 | 0.45 | 0.35 | 0.84 | 0.40 | 0.31 | 0.24 |
| 16 | 0.35 | 0.29 | 0.23 | 0.75 | 0.25 | 0.22 | 0.19 |
| 32 | 0.19 | 0.19 | 0.16 | 0.68 | 0.16 | 0.16 | 0.15 |
| 64 | 0.12 | 0.14 | 0.12 | 0.67 | 0.12 | 0.12 | 0.11 |

Readings, valid for this synthetic population only:
- The prior pays below k ≈ 32: at k = 8 it is 2–3× better than fitting from scratch, and from scratch is not even possible below k = 8.
- D-optimal cells roughly halve the segments needed at small k (hier N=4: 8 designed segments ≈ 16 random ones).
- Affine standardisation on the reference (`ref_gain`) plateaus at 0.6–0.9: a per-robot gain and offset cannot absorb dead-zone and exponent differences. That is the argument against the master/slave fallback, to be checked on real robots.
- Prior from 1 vs 4 robots matters at k ≤ 8 (0.45 vs 0.35 random, 0.31 vs 0.24 designed) and little beyond. In this population the N = 1 inflated prior already has the right shape; real robots may differ in ways the model family does not capture, which is exactly what several full grids would reveal.
- 16 designed segments (≈ 2 min of driving) leave an error of about 10 % of the between-robot spread and below one segment's noise. The "~75 segments in 10 min" budget looks generous here; the real question is whether the model family holds.
- 95 % intervals of the hier fit cover the true mean twist 0.85–0.95 of the time at k ≥ 4 (the floor heuristics are slightly narrow); 1.0 at k = 0 (prior too wide).

**Power** (`power --sims 8`: 3 spreads × fleets of 4, 6, 8 robots × 8 simulated populations, 2 days × 5 replicates each). "SD ratio" is the estimated between-robot SD of ω over the population's true value (from a 2,000-robot draw), median and 10–90 % band over populations; the figure-3 scores are medians, random design, 1 subset per robot, `scratch` with the quick 4-start fit.

| spread | n | SD ratio [q10, q90] | hier N=1, k=4 / 16 | hier N=n−1, k=4 / 16 | scratch k=16 | full |
|---|---|---|---|---|---|---|
| 0.3 | 4 | 0.87 [0.61, 1.09] | 0.58 / 0.26 | 0.53 / 0.22 | 0.48 | 0.06 |
| 0.3 | 8 | 0.84 [0.66, 1.10] | 0.57 / 0.25 | 0.42 / 0.20 | 0.33 | 0.05 |
| 1.0 | 4 | 0.91 [0.61, 1.12] | 0.74 / 0.28 | 0.74 / 0.25 | 0.41 | 0.06 |
| 1.0 | 8 | 0.83 [0.68, 1.18] | 0.66 / 0.27 | 0.55 / 0.24 | 0.34 | 0.05 |
| 3.0 | 4 | 0.97 [0.58, 1.26] | 1.20 / 0.41 | 1.33 / 0.32 | 0.51 | 0.05 |
| 3.0 | 8 | 0.81 [0.70, 1.28] | 1.32 / 0.36 | 0.91 / 0.29 | 0.37 | 0.05 |

Readings (synthetic; 8 populations per cell, so the bands themselves are rough):
- The number of full-grid robots mainly sets how well the *spread* is known: about ±40 % at 4 robots and ±30 % at 8 (the sampling error of a variance from n units, ≈ 1/√(2(n−1)), not measurement noise). Measurement noise is not the limit at 5 replicates.
- It barely changes how well a *new* robot is calibrated: at k = 16 the few-shot error is 0.20–0.29 at every spread and fleet size; N beyond 2–4 adds little, except at large spread and k ≤ 4.
- Planning consequence: 5–6 full grids are a defensible phase B; 8–12 buys a tighter spread estimate, not a better figure 3. The real risk that no simulation can show is model misspecification across robots, which only the full grids reveal.
- The first version of this sweep compared the estimate against the sampled robots' own spread and reported a ratio of 0.98 at every n; that measured ANOVA recovery, not sampling, and was replaced.
