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

**Searches that found nothing:** a peer-reviewed automatic Kilobot motor calibrator; speed vs voltage with a dead zone measured on bristle-bots; hierarchical Bayesian calibration of a robot fleet; transfer of a vibration-robot model to other robots with few trials; D-optimal design for mobile-robot odometry or motor maps; a data-budget-vs-accuracy curve for swarm robots.

## 6. Next step

Not another document. A 60 s static-tracking log and a first `calib_bot` dump from robot #1.
