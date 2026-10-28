# fruit-fly-graspion

Programming and using [GRASPion](https://github.com/GRASP-LAB/GRASPion) brainbots, vibration-driven robots on a SAMD21 body, as a testbed for learned and connectome-derived controllers.

| Path | What it is |
|---|---|
| [HARDWARE.md](HARDWARE.md) | Verified hardware reference: pins, motors, sensors, IR, power, toolchain, gotchas. Every claim is labelled sourced, INFERRED, UNKNOWN, UNTESTED or VERIFIED |
| [FEASIBILITY.md](FEASIBILITY.md) | Running the tilemat controllers (MPC, residual RL, connectome actors) and a VLA on the robot: compute, links, sim-to-robot gap, calibration protocol, literature |
| [calibration/](calibration/README.md) | Phase 0 "learning to walk": calibration firmware, IR blaster, tracker, scheduler and map fitting |
| [bot_blaster/](bot_blaster/bot_blaster.ino) | One GRASPion body used as a USB IR remote for the others, with a keyboard teleop and a browser remote |
| [calib_plot/](calib_plot/plot_paths.py) | HTML figure of a grid and a PRBS calibration session |
| [lab_fit/](lab_fit/fit_lab_tables.py) | Our forward-map models fitted to the GRASP lab's 7×7 calibration tables, against the lab's own fits |
| [AGENTS.md](AGENTS.md) | Rules for coding agents (`CLAUDE.md` includes it) |

The Python tools share one environment (numpy, scipy, opencv-contrib-python, pyserial; `calibration/pc/requirements.txt`):

```bash
cd calibration/pc && python -m graspion_calib selftest
```

Nothing here has been run on a bot unless HARDWARE.md marks it VERIFIED.
