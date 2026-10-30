"""graspion_calib command line. Run from calibration/pc:  python -m graspion_calib <command> --help

  register        read a USB-connected bot's ID / serial / watchdog ('i') into bots/<id>.json (refuses a bot
                  built with another IR protocol version)
  fleet-check     check bots/*.json for IR-ID collisions
  schedule        write a schedule JSON (grid, cells, ping, static)
  run             run a schedule on the rig (camera + blaster) or in the simulator (--sim); the operator
                  re-centres the bot before every segment; --only picks segments
  track           tracker only: live camera or a recorded video -> tracker CSV
  analyze         session folder(s) -> twists, forward map, inverse map (report.py)
  selftest        offline test suite (no hardware)
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

from . import protocol as P
from . import schedules as S


def _rig(path):
    from .geometry import RigConfig
    return RigConfig.load(path)


def _make_tracker(rig, bot_cfgs):
    """Tracker for the rig's references and the given rig-config bot entries (bots.<id>): LED offset and height
    per bot, camera intrinsics (an OpenCV .npz, optional) and camera_height_m (both used for the LED-height
    correction)."""
    from .tracker import BotMarker, Tracker, Undistorter
    cam_cfg = rig.camera
    und = Undistorter.from_npz(cam_cfg["intrinsics"]) if cam_cfg.get("intrinsics") else Undistorter()
    bots = {int(b["marker_id"]): BotMarker(int(b["marker_id"]), tuple(b.get("led_offset", (0.012, 0.0))),
                                           b.get("marker_size_m", 0.02), float(b.get("led_height") or 0.0))
            for b in bot_cfgs}
    return Tracker(rig.reference_markers, bots, und, ref_size_m=cam_cfg.get("reference_size_m"),
                   camera_height_m=cam_cfg.get("camera_height_m"))


def _wait_for_fit(tracker, cam, timeout_s=10.0):
    """Wait until the camera thread has fitted the homography. Tracker._fit_h publishes H last, after its
    reference checks, warnings and the led_mode decision."""
    t0 = time.time()
    while tracker.H is None:
        if time.time() - t0 > timeout_s:
            raise SystemExit("reference markers not all visible: homography not fitted")
        if cam.error:
            raise SystemExit(f"camera: {cam.error}")
        time.sleep(0.1)


def _print_tracker_warnings(tracker, start=0, last=5):
    """Print tracker.warnings[start:]: every one-off warning (t None: reference checks, mono camera, LED height)
    and the last `last` per-frame ones with their count (a bumped reference warns on every frame). Returns the
    index to continue from."""
    n = len(tracker.warnings)                      # the camera thread may still append
    ws = tracker.warnings[start:n]
    for t, w in ws:
        if t is None:
            print("warning:", w)
    timed = [w for t, w in ws if t is not None]
    if len(timed) > last:
        print(f"{len(timed)} per-frame tracker warnings, the last {last}:")
    for w in timed[max(0, len(timed) - last):]:
        print("warning:", w)
    return n


def _print_fit(tracker):
    print(f"homography fitted: {tracker.reference_summary()}; LED signal: {tracker.led_mode}")
    return _print_tracker_warnings(tracker)


def read_info_serial(port, baud=115200):
    """The bot's 'i' line over USB as a dict of its key=value fields."""
    import serial
    with serial.Serial(port, baud, timeout=1.0) as s:
        time.sleep(0.3)
        s.reset_input_buffer()
        s.write(b"i")
        ln = s.readline().decode("ascii", "replace").strip()
    if not ln.startswith("I "):
        raise RuntimeError(f"unexpected reply {ln!r}")
    return P.info_fields(ln)


def cmd_register(a):
    info = read_info_serial(a.port)
    err = P.proto_error(info, f"the bot on {a.port}", "calib_bot")
    if err:                                       # nothing is written: run would reject the file anyway
        raise SystemExit(f"{err}, then run register again")
    bid = int(info["id"])
    if info.get("serial") and int(info.get("id_override", "0")) == 0:
        assert P.bot_id_from_usb_serial(info["serial"]) == bid, "firmware and Python ID hashes disagree"
    out = Path(a.bots_dir)
    out.mkdir(parents=True, exist_ok=True)
    info["registered"] = time.strftime("%Y-%m-%d %H:%M:%S")
    (out / f"{bid}.json").write_text(json.dumps(info, indent=2))
    print(json.dumps(info, indent=2))


def cmd_fleet_check(a):
    seen = {}
    for p in sorted(Path(a.bots_dir).glob("*.json")):
        d = json.loads(p.read_text())
        seen.setdefault(int(d["id"]), []).append(d.get("serial"))
    dup = {k: v for k, v in seen.items() if len(v) > 1}
    print(f"{len(seen)} IDs; collisions: {dup or 'none'}")
    if dup:
        print("Fix: rebuild the colliding bots with -DBOT_ID_OVERRIDE=<unused id>.")
        sys.exit(1)


def cmd_schedule(a):
    seed = S.new_seed() if a.seed is None else a.seed
    blocks = a.blocks if a.blocks is not None else (1 if a.kind == "cells" else 5)
    if a.kind == "grid":
        specs = S.static_grid(blocks=blocks, drive_s=a.drive_s, rest_s=a.rest_s, seed=seed)
    elif a.kind == "cells":
        if not a.cells:
            raise SystemExit('schedule cells needs --cells "[[L, R], ...]"')
        pairs = json.loads(a.cells)
        if not pairs or any(len(c) != 2 or max(abs(c[0]), abs(c[1])) > S.DRIVE_MAX for c in pairs):
            raise SystemExit(f"--cells: a non-empty list of [L, R] pairs with |drive| <= {S.DRIVE_MAX}")
        specs = S.cells(pairs, blocks=blocks, drive_s=a.drive_s, rest_s=a.rest_s)
    elif a.kind == "ping":
        specs = S.pings(a.long_s)
    elif a.kind == "static":
        specs = S.static(a.long_s)
    else:
        raise SystemExit(f"unknown schedule kind {a.kind}")
    meta = {"kind": a.kind, "seed": seed, "created": time.strftime("%Y-%m-%d %H:%M:%S"),
            "args": {k: v for k, v in vars(a).items() if k not in ("fn", "seed")}}
    S.save(specs, a.out, meta)
    print(f"wrote {a.out} (seed {seed}): {S.summary(specs)}")


def _check_proto(bot, bot_info, bot_file):
    """Refuse a bot registered with another IR protocol version: every frame would be a CRC error on it, so each
    segment would end 'missed', with no hint at the cause."""
    if bot_info.get("proto") is None:             # not from register: nothing to compare
        return
    err = P.proto_error(bot_info, f"bot {bot} ({bot_file})", "calib_bot")
    if err:
        raise SystemExit(f"{err}, then run register again")


def _check_watchdog(bot, bot_info, period_ms, specs):
    """Refuse a driving schedule when one lost frame would trip the bot's command watchdog (at least 2.5 blaster
    periods). Link tests (ping) and static runs do not drive, so they may run at any period."""
    wd = bot_info.get("watchdog_ms")
    if wd is None or all(s.kind in ("ping", "static") for s in specs):
        return
    if float(wd) < 2.5 * period_ms:
        raise SystemExit(f"bot {bot}: watchdog_ms={wd} is under 2.5 blaster periods ({2.5 * period_ms:.0f} ms at "
                         f"bridge.period_ms={period_ms}), so single lost frames would stop it mid-segment. Rebuild "
                         "calib_bot with a larger WATCHDOG_MS and register it again, or shorten the period.")


def cmd_run(a):
    from .geometry import Arena
    from .poses import PoseStore, TrackerCSV
    from .scheduler import Session, SessionConfig, console_getkey, console_pause
    specs, schedule_meta = S.load_with_meta(a.schedule)
    extra = {"schedule_file": str(a.schedule), "schedule_meta": schedule_meta, "argv": sys.argv}
    if a.only:                                    # the session's schedule.json and seg numbers are the selection
        try:
            idx = S.parse_selection(a.only, len(specs))
        except ValueError as e:
            raise SystemExit(f"--only: {e}")
        specs = [specs[k] for k in idx]
        extra["only"] = {"selection": a.only, "indices": idx}
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    S.save(specs, out / "schedule.json", schedule_meta)
    tracker, n_warn = None, 0
    if a.sim:
        from . import sim
        from .clock import SimClock
        p = sim.SimParams()
        world = sim.SimWorld({a.bot: (p, (0.35, 0.45, 2.0))}, seed=a.seed, tracker_csv=TrackerCSV(out / "tracker.csv"))
        bridge, poses, clock = sim.SimBridge(world), world.poses, SimClock(world)
        geom, arena = p.geometry, Arena(0.0, 1.0, 0.0, 1.0, 0.08)
        extra.update({"simulated": True, "pwm_hz": "sim"})
        on_pause = (lambda msg: (print("PAUSE (sim, aborting):", msg), "abort")[1])

        def getkey():                             # the simulated operator: bot back at the centre, then Enter
            plant = world.plants[a.bot]
            plant.x, plant.y = 0.5, 0.5
            return "\r"
        cam = None
    else:
        from .bridge import PERIOD_MAX_MS, PERIOD_MIN_MS, BridgeError, SerialBridge
        from .clock import RealClock
        from .tracker import CameraThread, open_camera
        rig = _rig(a.rig)
        geom, arena = rig.geometry(a.bot), rig.arena
        bcfg = rig.bots[str(a.bot)]
        period_ms = int(rig.bridge.get("period_ms", 110))
        if not PERIOD_MIN_MS <= period_ms <= PERIOD_MAX_MS:
            raise SystemExit(f"bridge.period_ms={period_ms} outside {PERIOD_MIN_MS}..{PERIOD_MAX_MS} (frames take up "
                             "to 86 ms and the bot's receiver needs 8 ms of silence after each)")
        bot_file = Path(a.bots_dir) / f"{a.bot}.json"
        if bot_file.exists():
            extra["bot_info"] = json.loads(bot_file.read_text())
            extra["pwm_hz"] = extra["bot_info"].get("pwm_hz")
            _check_proto(a.bot, extra["bot_info"], bot_file)
            _check_watchdog(a.bot, extra["bot_info"], period_ms, specs)
        else:
            print(f"warning: {bot_file} not found (run 'register'): the bot's watchdog is not checked against "
                  f"the blaster period")
        cam_cfg = rig.camera
        tracker = _make_tracker(rig, [bcfg])
        poses = PoseStore()
        cap = open_camera(cam_cfg.get("index", 0), cam_cfg.get("width", 1280), cam_cfg.get("height", 720),
                          cam_cfg.get("fps", 60), cam_cfg.get("exposure"))
        cam = CameraThread(cap, tracker, poses, out / "tracker.csv",
                           out / "video.avi" if a.video else None, cam_cfg.get("fps", 60))
        cam.start()
        _wait_for_fit(tracker, cam)
        n_warn = _print_fit(tracker)
        try:                                      # e.g. a blaster on another protocol version
            bridge = SerialBridge(a.port or rig.bridge["port"], period_ms=period_ms)
        except BridgeError as e:
            cam.stop()
            cam.join(2.0)
            raise SystemExit(f"blaster: {e}")
        clock = RealClock()
        extra.update({"rig": rig.to_dict(), "homography_rms_m": tracker.reproj_rms_m,
                      "ref_check": tracker.ref_check, "reference_summary": tracker.reference_summary(),
                      "tracker_warnings": [w for _, w in tracker.warnings[:n_warn]], "led_mode": tracker.led_mode})
        on_pause = console_pause
        getkey = console_getkey()
    cfg = SessionConfig(bot_id=a.bot, geometry=geom, arena=arena, out_dir=str(out), notes=a.notes or "")
    sess = Session(cfg, bridge, poses, clock, getkey, on_pause=on_pause, extra_meta=extra)
    try:
        counts = sess.run(specs)
        print("done:", counts)
    finally:
        if cam is not None:
            cam.stop()
            cam.join(2.0)
        if a.sim:
            world.csv.close()
        bridge.close()
        if tracker is not None:
            _print_tracker_warnings(tracker, start=n_warn)


def cmd_track(a):
    from .poses import PoseStore
    from .tracker import CameraThread, open_camera, track_video
    rig = _rig(a.rig)
    cam_cfg = rig.camera
    tracker = _make_tracker(rig, rig.bots.values())
    if a.video:
        n = track_video(a.video, tracker, a.out, fps=a.fps)
        print(f"{n} frames -> {a.out}")
        if tracker.H is None:
            print("homography not fitted: the reference markers were not all visible in enough frames")
        else:
            print(f"homography: {tracker.reference_summary()}; LED signal: {tracker.led_mode}")
        _print_tracker_warnings(tracker)
        return
    cap = open_camera(cam_cfg.get("index", 0), cam_cfg.get("width", 1280), cam_cfg.get("height", 720),
                      cam_cfg.get("fps", 60), cam_cfg.get("exposure"))
    poses = PoseStore()
    cam = CameraThread(cap, tracker, poses, a.out, a.record, cam_cfg.get("fps", 60))
    cam.start()
    t_end = time.time() + a.seconds
    n_warn = None
    try:
        while time.time() < t_end and cam.is_alive():
            time.sleep(1.0)
            if n_warn is None and tracker.H is not None:
                n_warn = _print_fit(tracker)
            print(f"frames {poses.frames}  H {'ok' if tracker.H is not None else 'waiting'}  "
                  + "  ".join(f"{i}:({d.x:.3f},{d.y:.3f})" for i in tracker.bots if (d := poses.latest(i))))
    finally:
        cam.stop()
        cam.join(2.0)
    if cam.error:
        print("camera:", cam.error)
    _print_tracker_warnings(tracker, start=n_warn or 0)


def cmd_analyze(a):
    from .report import analyze
    s = analyze(a.sessions, a.out, bootstrap_n=a.bootstrap)
    fm = s.get("forward_map", {})
    print(json.dumps({"n_segments_fit": s.get("n_segments_fit"), "selected": fm.get("meta", {}).get("selected"),
                      "p0": fm.get("p0"), "gamma": fm.get("gamma"), "latency": s.get("latency"),
                      "replicate_scatter": s.get("replicate_scatter"), "error": s.get("error")},
                     indent=2, default=float))
    print(f"full results in {a.out}/calib.json")


def cmd_selftest(a):
    import unittest
    here = Path(__file__).resolve().parents[1] / "tests"
    suite = unittest.defaultTestLoader.discover(str(here), top_level_dir=str(here.parent))
    r = unittest.TextTestRunner(verbosity=2 if a.verbose else 1).run(suite)
    sys.exit(0 if r.wasSuccessful() else 1)


def main(argv=None):
    ap = argparse.ArgumentParser(prog="graspion_calib", description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("register"); p.add_argument("--port", required=True); p.add_argument("--bots-dir", default="bots")
    p.set_defaults(fn=cmd_register)
    p = sub.add_parser("fleet-check"); p.add_argument("--bots-dir", default="bots"); p.set_defaults(fn=cmd_fleet_check)

    p = sub.add_parser("schedule")
    p.add_argument("kind", choices=["grid", "cells", "ping", "static"])
    p.add_argument("--out", required=True)
    p.add_argument("--blocks", type=int, help="randomised blocks of the grid, or how many times each cell runs "
                                              "(cells). Default 5, cells 1")
    p.add_argument("--drive-s", type=float, default=3.0)
    p.add_argument("--rest-s", type=float, default=0.8)
    p.add_argument("--long-s", type=float, default=60.0, help="length of a ping or static run")
    p.add_argument("--cells", help='cells: JSON list of [L, R] drive pairs, run in this order, e.g. "[[80, 80], [100, 60]]"')
    p.add_argument("--seed", type=int, help="order seed for the grid (default: a fresh random one; it is recorded "
                                            "in the schedule JSON)")
    p.set_defaults(fn=cmd_schedule)

    p = sub.add_parser("run")
    p.add_argument("--bot", type=int, required=True, help="IR bot ID (from register)")
    p.add_argument("--schedule", required=True)
    p.add_argument("--out", required=True, help="new session folder")
    p.add_argument("--rig", help="rig config JSON (configs/rig_example.json)")
    p.add_argument("--port", help="blaster serial port (overrides the rig config)")
    p.add_argument("--bots-dir", default="bots")
    p.add_argument("--video", action="store_true", help="also record raw video (large)")
    p.add_argument("--sim", action="store_true", help="simulated bot, rig and blaster (dry run)")
    p.add_argument("--seed", type=int, default=0)
    p.add_argument("--notes")
    p.add_argument("--only", metavar="SEL", help='run these segments of the schedule only, e.g. "0-3,7,12-" '
                                                 "(indices as in the schedule file, in the order given)")
    p.set_defaults(fn=cmd_run)

    p = sub.add_parser("track")
    p.add_argument("--rig", required=True); p.add_argument("--out", required=True)
    p.add_argument("--video", help="re-track a recorded video instead of the live camera")
    p.add_argument("--fps", type=float); p.add_argument("--seconds", type=float, default=60.0)
    p.add_argument("--record", help="also save raw video here (live mode)")
    p.set_defaults(fn=cmd_track)

    p = sub.add_parser("analyze")
    p.add_argument("sessions", nargs="+"); p.add_argument("--out", required=True)
    p.add_argument("--bootstrap", type=int, default=50)
    p.set_defaults(fn=cmd_analyze)

    p = sub.add_parser("selftest"); p.add_argument("-v", "--verbose", action="store_true"); p.set_defaults(fn=cmd_selftest)

    a = ap.parse_args(argv)
    if a.cmd == "run" and not a.sim and not a.rig:
        ap.error("run needs --rig (or --sim)")
    a.fn(a)


if __name__ == "__main__":
    main()
