"""graspion_calib command line. Run from calibration/pc:  python -m graspion_calib <command> --help

  register        read a USB-connected bot's ID / serial / PWM mode ('i') into bots/<id>.json
  dump            download a bot's log ('d') to CSV (copy it into the session folder as bot_dump.csv)
  fleet-check     check bots/*.json for IR-ID collisions
  schedule        write a schedule JSON (grid, steps, prbs, circles, ping, static, validation)
  run             run a schedule on the rig (camera + blaster) or in the simulator (--sim)
  track           tracker only: live camera or a recorded video -> tracker CSV
  analyze         session folder(s) -> maps, polytope, lag, noise (report.py)
  camera-calib    ChArUco intrinsics -> .npz (or --make-board to print the board)
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


def cmd_register(a):
    from .botlog import read_info_serial
    info = read_info_serial(a.port)
    bid = int(info["id"])
    if info.get("serial") and int(info.get("id_override", "0")) == 0:
        assert P.bot_id_from_usb_serial(info["serial"]) == bid, "firmware and Python ID hashes disagree"
    out = Path(a.bots_dir)
    out.mkdir(parents=True, exist_ok=True)
    info["registered"] = time.strftime("%Y-%m-%d %H:%M:%S")
    (out / f"{bid}.json").write_text(json.dumps(info, indent=2))
    print(json.dumps(info, indent=2))


def cmd_dump(a):
    from .botlog import parse_dump, read_dump_serial
    text = read_dump_serial(a.port)
    Path(a.out).write_text(text + "\n")
    info, stats, recs = parse_dump(text)
    print(f"{len(recs)} records from bot {info.get('id')}; stats {stats}")


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
    if a.kind == "grid":
        specs = S.static_grid(blocks=a.blocks, drive_s=a.drive_s, rest_s=a.rest_s, seed=a.seed)
    elif a.kind == "steps":
        specs = S.steps([((60, 60), (100, 100)), ((80, 0), (80, 80)), ((0, 80), (80, 80)), ((70, 90), (90, 70))],
                        hold_s=a.drive_s / 2)
    elif a.kind == "prbs":
        specs = S.prbs([(70, 70), (100, 100), (90, 60), (60, 90), (0, 90), (90, 0)], duration_s=a.drive_s,
                       n_segments=a.blocks * 4, seed=a.seed)
    elif a.kind == "circles":
        specs = S.circles(json.loads(a.pairs or "[[100, 60], [60, 100]]"), duration_s=a.long_s)
    elif a.kind == "ping":
        specs = S.pings(a.long_s)
    elif a.kind == "static":
        specs = S.static(a.long_s)
    elif a.kind == "validation":
        from .forward_map import PowerLawMap
        from .inverse_map import InverseMap
        inv = InverseMap(PowerLawMap.load(a.forward_map))
        targets = json.loads(a.targets) if a.targets else [
            (float(v), float(w)) for v in (0.06, 0.08, 0.10) for w in (-1.5, -0.8, 0.0, 0.8)]
        specs = S.validation(targets, inv, duration_s=a.drive_s)
    else:
        raise SystemExit(f"unknown schedule kind {a.kind}")
    S.save(specs, a.out)
    print(f"wrote {a.out}: {S.summary(specs)}")


def cmd_run(a):
    from .geometry import Arena
    from .poses import PoseStore, TrackerCSV
    from .scheduler import RecentreConfig, Session, SessionConfig, SafetyConfig, console_pause
    specs = S.load(a.schedule)
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    S.save(specs, out / "schedule.json")
    extra = {"schedule_file": str(a.schedule), "argv": sys.argv}
    if a.sim:
        from . import sim
        from .clock import SimClock
        p = sim.SimParams()
        world = sim.SimWorld({a.bot: (p, (0.35, 0.45, 2.0))}, seed=a.seed, tracker_csv=TrackerCSV(out / "tracker.csv"))
        bridge, poses, clock = sim.SimBridge(world), world.poses, SimClock(world)
        geom, arena = p.geometry, Arena(0.0, 1.0, 0.0, 1.0, 0.08)
        extra.update({"simulated": True, "pwm_hz": "sim"})
        on_pause = (lambda msg: (print("PAUSE (sim, aborting):", msg), "abort")[1])
        cam = None
    else:
        from .bridge import SerialBridge
        from .clock import RealClock
        from .tracker import BotMarker, CameraThread, Tracker, Undistorter, open_camera
        rig = _rig(a.rig)
        geom, arena = rig.geometry(a.bot), rig.arena
        bcfg = rig.bots[str(a.bot)]
        cam_cfg = rig.camera
        und = Undistorter.from_npz(cam_cfg["intrinsics"]) if cam_cfg.get("intrinsics") else Undistorter()
        tracker = Tracker(rig.reference_markers,
                          {geom.marker_id: BotMarker(geom.marker_id, tuple(bcfg.get("led_offset", (0.012, 0.0))),
                                                     bcfg.get("marker_size_m", 0.02))},
                          und, ref_size_m=cam_cfg.get("reference_size_m"))
        poses = PoseStore()
        cap = open_camera(cam_cfg.get("index", 0), cam_cfg.get("width", 1280), cam_cfg.get("height", 720),
                          cam_cfg.get("fps", 60), cam_cfg.get("exposure"))
        cam = CameraThread(cap, tracker, poses, out / "tracker.csv",
                           out / "video.avi" if a.video else None, cam_cfg.get("fps", 60))
        cam.start()
        t0 = time.time()
        while tracker.H is None:
            if time.time() - t0 > 10:
                raise SystemExit("reference markers not all visible: homography not fitted")
            if cam.error:
                raise SystemExit(f"camera: {cam.error}")
            time.sleep(0.1)
        print(f"homography fitted: reference reprojection RMS {tracker.reproj_rms_m * 1000:.2f} mm")
        bridge = SerialBridge(a.port or rig.bridge["port"], period_ms=rig.bridge.get("period_ms", 110))
        clock = RealClock()
        bot_file = Path(a.bots_dir) / f"{a.bot}.json"
        if bot_file.exists():
            extra["bot_info"] = json.loads(bot_file.read_text())
            extra["pwm_hz"] = extra["bot_info"].get("pwm_hz")
        extra.update({"rig": rig.to_dict(), "homography_rms_m": tracker.reproj_rms_m})
        on_pause = console_pause
    cfg = SessionConfig(bot_id=a.bot, geometry=geom, arena=arena, out_dir=str(out), notes=a.notes or "",
                        safety=SafetyConfig(), recentre=RecentreConfig())
    sess = Session(cfg, bridge, poses, clock, on_pause=on_pause, extra_meta=extra)
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


def cmd_track(a):
    from .poses import PoseStore
    from .tracker import BotMarker, CameraThread, Tracker, Undistorter, open_camera, track_video
    rig = _rig(a.rig)
    cam_cfg = rig.camera
    und = Undistorter.from_npz(cam_cfg["intrinsics"]) if cam_cfg.get("intrinsics") else Undistorter()
    bots = {int(b["marker_id"]): BotMarker(int(b["marker_id"]), tuple(b.get("led_offset", (0.012, 0.0))),
                                           b.get("marker_size_m", 0.02)) for b in rig.bots.values()}
    tracker = Tracker(rig.reference_markers, bots, und, ref_size_m=cam_cfg.get("reference_size_m"))
    if a.video:
        n = track_video(a.video, tracker, a.out, fps=a.fps)
        print(f"{n} frames -> {a.out}; reference RMS {tracker.reproj_rms_m * 1000:.2f} mm")
        return
    cap = open_camera(cam_cfg.get("index", 0), cam_cfg.get("width", 1280), cam_cfg.get("height", 720),
                      cam_cfg.get("fps", 60), cam_cfg.get("exposure"))
    poses = PoseStore()
    cam = CameraThread(cap, tracker, poses, a.out, a.record, cam_cfg.get("fps", 60))
    cam.start()
    t_end = time.time() + a.seconds
    try:
        while time.time() < t_end and cam.is_alive():
            time.sleep(1.0)
            print(f"frames {poses.frames}  H {'ok' if tracker.H is not None else 'waiting'}  "
                  + "  ".join(f"{i}:({d.x:.3f},{d.y:.3f})" for i in bots if (d := poses.latest(i))))
    finally:
        cam.stop()
        cam.join(2.0)
    for t, w in tracker.warnings[-5:]:
        print("warning:", w)


def cmd_analyze(a):
    from .report import analyze
    s = analyze(a.sessions, a.out, gp=a.gp, bootstrap_n=a.bootstrap)
    fm = s.get("forward_map", {})
    print(json.dumps({"n_segments_fit": s.get("n_segments_fit"), "selected": fm.get("meta", {}).get("selected"),
                      "p0": fm.get("p0"), "gamma": fm.get("gamma"), "latency": s.get("latency"),
                      "dynamics": {k: {kk: v[kk] for kk in ("dead_time", "tau")} for k, v in s.get("dynamics", {}).items()},
                      "error": s.get("error")}, indent=2, default=float))
    print(f"full results in {a.out}/calib.json")


def cmd_camera_calib(a):
    import cv2
    from . import intrinsics
    board = intrinsics.make_board(tuple(a.squares), a.square_m, a.marker_m)
    if a.make_board:
        cv2.imwrite(a.make_board, intrinsics.board_image(board))
        print(f"wrote {a.make_board}; print at 100 % and measure the square size")
        return
    res = intrinsics.calibrate_folder(a.images, a.out, squares=tuple(a.squares), square_m=a.square_m,
                                      marker_m=a.marker_m)
    print(f"views {res['views']}, RMS {res['rms_px']:.3f} px -> {a.out}")


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
    p = sub.add_parser("dump"); p.add_argument("--port", required=True); p.add_argument("--out", required=True)
    p.set_defaults(fn=cmd_dump)
    p = sub.add_parser("fleet-check"); p.add_argument("--bots-dir", default="bots"); p.set_defaults(fn=cmd_fleet_check)

    p = sub.add_parser("schedule")
    p.add_argument("kind", choices=["grid", "steps", "prbs", "circles", "ping", "static", "validation"])
    p.add_argument("--out", required=True)
    p.add_argument("--blocks", type=int, default=5)
    p.add_argument("--drive-s", type=float, default=3.0)
    p.add_argument("--rest-s", type=float, default=0.8)
    p.add_argument("--long-s", type=float, default=60.0)
    p.add_argument("--pairs", help="JSON list of [L, R] for circles")
    p.add_argument("--forward-map", help="forward_map.json (validation)")
    p.add_argument("--targets", help="JSON list of [v, omega] (validation)")
    p.add_argument("--seed", type=int, default=0)
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
    p.set_defaults(fn=cmd_run)

    p = sub.add_parser("track")
    p.add_argument("--rig", required=True); p.add_argument("--out", required=True)
    p.add_argument("--video", help="re-track a recorded video instead of the live camera")
    p.add_argument("--fps", type=float); p.add_argument("--seconds", type=float, default=60.0)
    p.add_argument("--record", help="also save raw video here (live mode)")
    p.set_defaults(fn=cmd_track)

    p = sub.add_parser("analyze")
    p.add_argument("sessions", nargs="+"); p.add_argument("--out", required=True)
    p.add_argument("--gp", action="store_true"); p.add_argument("--bootstrap", type=int, default=50)
    p.set_defaults(fn=cmd_analyze)

    p = sub.add_parser("camera-calib")
    p.add_argument("--images"); p.add_argument("--out", default="camera.npz"); p.add_argument("--make-board")
    p.add_argument("--squares", type=int, nargs=2, default=(7, 5))
    p.add_argument("--square-m", type=float, default=0.03); p.add_argument("--marker-m", type=float, default=0.022)
    p.set_defaults(fn=cmd_camera_calib)

    p = sub.add_parser("selftest"); p.add_argument("-v", "--verbose", action="store_true"); p.set_defaults(fn=cmd_selftest)

    a = ap.parse_args(argv)
    if a.cmd == "run" and not a.sim and not a.rig:
        ap.error("run needs --rig (or --sim)")
    a.fn(a)


if __name__ == "__main__":
    main()
