# Agent guide

This project programs and uses **GRASPion** brainbots: vibration-driven robots built on a SAMD21 Arduino body (upstream: https://github.com/GRASP-LAB/GRASPion).

**Before any hardware-related work, read [HARDWARE.md](HARDWARE.md).** That includes firmware, pins, motors, sensors, IR, power, flashing, front or head modules, and experiment modelling. It is the verified reference for the robot. Every claim in it is labelled as sourced, INFERRED, UNKNOWN or UNTESTED, so you do not need to re-read the upstream schematics, datasheets or paper.

**For Phase 0 calibration ("learning to walk": firmware, IR blaster, tracker, scheduler, map fitting), read [calibration/README.md](calibration/README.md).** Its offline tests run with `python -m graspion_calib selftest` from `calibration/pc`.

**Before running controllers or learned policies on the robot, read [FEASIBILITY.md](FEASIBILITY.md).** This covers the figure-8 MPC / residual / PPO / connectome brains from `tilemat_control_stack`, the external vs onboard brain, calibration ("learning to walk") and swarms. It quantifies what fits on which chip, the sim-to-robot gap, the calibration protocol and the roadmap.

Rules that prevent the most common mistakes (details in HARDWARE.md §11):
- Build for `graspion:samd:graspionBody:usbstack=tinyusb`, not the Adafruit QT Py M0 board.
  - Use `opt=fastest` (what upstream asks for) only for sketches that pull in no Adafruit BusIO-based library. The Adafruit MLX90393 and APDS9960 libraries use BusIO, and so do all three upstream examples (each includes `Adafruit_MLX90393.h`). Those fail to assemble at `opt=fastest`, `opt=faster` and `opt=dragons` (`lo register required`).
  - Build them with `opt=small` (verified), or drive the sensor over raw `Wire` as `calibration/firmware/calib_bot` does (HARDWARE.md §9.1). `opt=fast` (-O2) is UNTESTED for a whole sketch: only the failing BusIO file was shown to assemble at -O2.
- Begin `setup()` with `pinMode(BUZ_ENABLE, OUTPUT); digitalWrite(BUZ_ENABLE, LOW);`.
- Motor pins: D2 = right, D3 = left, D12 = shared enable. `analogWrite` 128 = stop.
- I2C addresses: MLX90393 at **0x10** (pass it explicitly, because the library default is 0x0C); AVR power monitor at 0x36; APDS-9960 at 0x39.
- With TinyUSB, `#include <Adafruit_TinyUSB.h>` in every sketch (guard it with `#if defined(USE_TINYUSB)` if it must also build with `usbstack=arduino`). `Serial` links only when that header reaches the build, from the sketch or from a library header that includes it. Make `loop()` return or yield, and never call `Serial.end()`.
- Mark anything you have not verified on real hardware as untested.
