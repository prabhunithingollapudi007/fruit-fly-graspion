Interim calibration v4 of the lab's overhead camera, 2026-11-12, IN PROGRESS on the teammate's side. Made with
graspion_tracking (github.com/prajolshrestha/brainbot_ws, branch camera_calibration, commit d7814a5,
src/graspion_tracking/calibration/results/calib_2026-11-12_interim_v4/). Copied unchanged on 2026-11-12:

  intrinsics.json       lens v3 of 2026-11-11 (same focus and iris) in the pixels of the fixed camera view
                        2700 x 2064 at offset (192, 0): cx 1295.52, cy 971.61, f 2624.27 px. No new lens shots:
                        outside the old 2400 x 2000 window (sensor columns < 344 or > 2743, rows < 32 or > 2031)
                        the distortion is extrapolated.
  arena.json            the 8 arena markers re-stuck at the view's edges (0, 7, 3 left; 1, 5, 2 right; 4 bottom
                        middle; 6 top middle), flat on the plate (z = 0), 953 x 729 mm marked area. Camera 942.4 mm
                        above the plate, tilt 3.9 deg.
  camera_settings.json  the view, 20 ms, 5 dB and Mono8, saved in the camera as UserSet1 (loaded at power-on);
                        the camera runs at ~49.7 fps.

Quality so far (teammate's README): layout fit corner rms 1.24 px, max 2.93 px, side ratio 1.05 %, so the 1 mm
gates MISS narrowly. Refitting with marker subsets moves robot positions by 0.8-1.2 mm median (up to 3.3 mm), and
marker 0 -> 1 reads 891.9 / 893.0 / 894.5 mm with 8 / 6 / 4 markers (about 0.3 % scale uncertainty). Their fix:
tile shots at the view's edges, or a steel tape on 0 -> 1. Not written yet on their side: calibration.json,
robots.json.

Robot centre (arena_extent_v4.py, 31 mm clear of the marker papers): x 86..860 mm, y 87..633 mm (774 x 546 mm).
Moving markers 4 and 6 to the sides would give 774 x 680 mm.

Used by ../rig_interim_v4.json. Copy the files here again when v4 is finished (calib v2), and after any focus,
iris, view or marker change.
