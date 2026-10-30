"""Segment scheduler: drives one bot through a schedule over the IR blaster, with the operator as the re-centring
loop and PC-side stops.

Per drive segment:
  1. Pause: STOP, then wait for the operator. The status line shows the bot's position; the operator places the
     bot by hand or nudges it with the keys (short RECENTRE bursts through the same bridge), then presses Enter
     to run the segment from where the bot stands, x to skip it or q to end the session. Nothing waits for the
     marker during the pause (a hand over the bot hides it); the segment itself does.
  2. STOP and rest rest_s.
  3. SEG(L, R): the bot starts and flashes. Then DRIVE keep-alives at the blaster period, so the bot's ~300 ms
     command watchdog never trips while the PC is healthy.
  4. STOP. A segment with no flash and no motion is retried once ("missed": the SEG frame was lost).
PC-side stops (STOP burst, segment marked): marker lost for marker_loss_s, predicted pose outside the geofence,
or any exception. A nudge stops on marker loss or at the arena fence while moving away from home.
On exit the scheduler sends STOP to the bot and a broadcast STOP, each in its own try (resyncing the bridge
after a failure, e.g. a Ctrl-C that left half a reply in the serial buffer); the original exception propagates.

UNTESTED on the rig: exercised only against sim.SimWorld (tests/test_scheduler.py).
"""
from __future__ import annotations

import csv
import json
import math
import sys
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np

from . import protocol as P
from .bridge import BridgeError, FrameLog, SendRecord
from .geometry import Arena, BotGeometry, wrap
from .poses import PoseStore
from .schedules import SegmentSpec

SEGMENT_COLUMNS = ["seg", "attempt", "kind", "tag", "block", "L", "R", "levels", "duration_s", "rest_s",
                   "t_seg_req", "t_seg_ack", "t_end_ack", "status", "flash_live", "moved_m", "x0", "y0", "th0",
                   "n_frames"]
ANCHOR_PINGS = 3                       # a ping run starts with this many isolated PINGs (run_ping) ...
ANCHOR_GAP_S = 0.8                     # ... this far apart: more than extract.match_flashes' max_lag (0.5 s) plus
                                       # half a period, so each anchor's flash fits only its own PING at the true lag
PAUSE_KEYS = "w/a/d (or arrows): nudge forward / pivot left / pivot right, space: stop, Enter: run, x: skip, q: quit"


@dataclass
class SafetyConfig:
    marker_loss_s: float = 0.25        # no detection for this long -> stop (camera ~40-60 fps: 10+ frames)
    lookahead_s: float | None = None   # geofence check on the pose predicted this far ahead; None: stop_latency_s
    cam_latency_s: float = 0.05        # exposure -> frame read (pose times are read times); INFERRED, measure it
    stop_tail_s: float = 0.1           # after the STOP frame: decode gap (8 ms), dead time and motor lag
    reacquire_timeout_s: float = 5.0
    stop_repeats: int = 2
    flash_window_s: float = 0.5        # flash expected within this time after the SEG acknowledgement
    flash_rise: float = 40.0           # LED level rise over the pre-segment baseline that counts as a flash
    missed_motion_m: float = 0.003     # less displacement than this + no flash -> the SEG was missed
    max_retries: int = 1

    def stop_latency_s(self, period_s):
        """Pose exposure -> bot at rest after the PC reacts to it: camera latency, up to one period until the next
        check (the loop is paced by the blocking send), up to one period until the STOP has left the blaster,
        then decode and spin-down. Lengthening the blaster period therefore needs a longer geofence lookahead."""
        if self.lookahead_s is not None:
            return self.lookahead_s
        return self.cam_latency_s + 2 * period_s + self.stop_tail_s


@dataclass
class NudgeConfig:
    straight: tuple = (85, 85)         # the operator's w key; equal drive drifts CW on the twin's bot
    pivot_ccw: tuple = (0, 90)         # a key: right motor only -> turns left (CCW), paper / HARDWARE.md 4.4
    pivot_cw: tuple = (90, 0)          # d key
    nudge_s: float = 0.6               # one key press drives this long (a few cm), then STOP
    fence_m: float = 0.03              # a nudge stops when the predicted pose gets this close to the arena edge
                                       # while moving away from home (about the body half-length)


@dataclass
class SessionConfig:
    bot_id: int
    geometry: BotGeometry
    arena: Arena
    out_dir: str
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    nudge: NudgeConfig = field(default_factory=NudgeConfig)
    notes: str = ""


class SessionAbort(RuntimeError):
    pass


def console_pause(msg: str) -> str:
    """Operator prompt for the marker-loss wait: returns 'continue' or 'abort'."""
    print(f"\n*** PAUSED: {msg}")
    ans = input("    [Enter] continue, 'a' abort: ").strip().lower()
    return "abort" if ans.startswith("a") else "continue"


def console_getkey():
    """A non-blocking key reader for the operator pause: the next key, or None when none is pressed. Arrow keys
    map to w/a/d/s. Windows reads the console through msvcrt; elsewhere stdin is put into cbreak mode for each
    poll only, so input() (console_pause) still works between polls."""
    try:
        import msvcrt
    except ImportError:
        import select
        import termios
        import tty
        arrows = {"A": "w", "D": "a", "C": "d", "B": "s"}          # ESC [ A/B/C/D

        def getkey():
            fd = sys.stdin.fileno()
            old = termios.tcgetattr(fd)
            try:
                tty.setcbreak(fd)
                if not select.select([sys.stdin], [], [], 0)[0]:
                    return None
                k = sys.stdin.read(1)
                if k == "\x1b" and select.select([sys.stdin], [], [], 0.01)[0] and sys.stdin.read(1) == "[":
                    return arrows.get(sys.stdin.read(1))
                return k
            finally:
                termios.tcsetattr(fd, termios.TCSADRAIN, old)
        return getkey
    arrows = {"H": "w", "K": "a", "M": "d", "P": "s"}              # a 0x00 / 0xE0 prefix, then the scan code

    def getkey():
        if not msvcrt.kbhit():
            return None
        k = msvcrt.getwch()
        return arrows.get(msvcrt.getwch()) if k in ("\x00", "\xe0") else k
    return getkey


class Session:
    def __init__(self, cfg: SessionConfig, bridge, poses: PoseStore, clock, getkey, on_pause=console_pause,
                 extra_meta=None):
        """getkey: the operator's key reader (console_getkey()); in a dry run one that returns Enter at once."""
        self.cfg, self.bridge, self.poses, self.clock = cfg, bridge, poses, clock
        self.getkey, self.on_pause = getkey, on_pause
        self.out = Path(cfg.out_dir)
        self.out.mkdir(parents=True, exist_ok=True)
        self.frames = FrameLog(self.out / "frames.csv")
        self._seg_f = open(self.out / "segments.csv", "w", newline="")
        self._seg_w = csv.writer(self._seg_f)
        self._seg_w.writerow(SEGMENT_COLUMNS)
        self._ev_f = open(self.out / "events.csv", "w", newline="")
        self._ev_w = csv.writer(self._ev_f)
        self._ev_w.writerow(["t", "event", "detail"])
        self.meta = {"bot_id": cfg.bot_id, "geometry": asdict(cfg.geometry), "arena": asdict(cfg.arena),
                     "safety": asdict(cfg.safety), "nudge": asdict(cfg.nudge), "notes": cfg.notes,
                     "started_wall": time.strftime("%Y-%m-%d %H:%M:%S"), "proto": P.PROTO_VERSION,
                     "bridge_info": getattr(bridge, "info", ""), "period_s": self.period_s(),
                     "geofence_lookahead_s": self.lookahead_s()}
        if extra_meta:
            self.meta.update(extra_meta)
        self._write_meta()
        self.counts = {}

    def period_s(self):
        return float(getattr(self.bridge, "period_s", 0.11))

    def lookahead_s(self):
        return self.cfg.safety.stop_latency_s(self.period_s())

    # ------------------------------------------------------------------ logging
    def _write_meta(self):
        with open(self.out / "session.json", "w") as f:
            json.dump(self.meta, f, indent=2, default=str)

    def event(self, kind, detail=""):
        self._ev_w.writerow([f"{self.clock.now():.6f}", kind, detail])
        self._ev_f.flush()
        self.counts[kind] = self.counts.get(kind, 0) + 1

    def warn(self, msg):
        """Operator warning: printed, logged as a 'warning' event and kept in session.json ('warnings')."""
        print(f"warning: {msg}")
        self.event("warning", msg)
        self.meta.setdefault("warnings", []).append(msg)

    def send(self, op, L=0, R=0, seg=-1, purpose="", bot=None):
        raw = P.pack(self.cfg.bot_id if bot is None else bot, op, L, R)
        rec = self.bridge.send(raw)
        self.frames.write(rec, seg, purpose)
        return rec

    def stop(self, seg=-1, purpose="stop"):
        first = None
        for _ in range(max(1, self.cfg.safety.stop_repeats)):
            rec = self.send(P.OP_STOP, seg=seg, purpose=purpose)
            first = first or rec
        return first

    # ------------------------------------------------------------------ pose helpers
    def body_pose(self):
        """(t, x, y, th) of the body reference point from the latest detection, or None."""
        d = self.poses.latest(self.cfg.geometry.marker_id)
        if d is None:
            return None
        x, y, th = self.cfg.geometry.body_from_marker(d.x, d.y, d.theta)
        return d.t, float(x), float(y), float(th)

    def pose_age(self):
        d = self.poses.latest(self.cfg.geometry.marker_id)
        return math.inf if d is None else self.clock.now() - d.t

    def recent_rates(self, window_s=0.3):
        """Recent world velocity and turn rate of the body point (for latency prediction)."""
        now = self.clock.now()
        h = self.poses.history(self.cfg.geometry.marker_id, since_t=now - window_s - 0.2)
        if len(h) < 4:
            return 0.0, 0.0, 0.0
        g = self.cfg.geometry
        t = np.array([d.t for d in h])
        x, y, th = g.body_from_marker([d.x for d in h], [d.y for d in h], [d.theta for d in h])
        th = np.unwrap(th)
        A = np.c_[t - t[-1], np.ones_like(t)]
        vx = np.linalg.lstsq(A, x, rcond=None)[0][0]
        vy = np.linalg.lstsq(A, y, rcond=None)[0][0]
        w = np.linalg.lstsq(A, th, rcond=None)[0][0]
        return float(vx), float(vy), float(w)

    def wait_for_marker(self):
        t0 = self.clock.now()
        while self.pose_age() > self.cfg.safety.marker_loss_s:
            if self.clock.now() - t0 > self.cfg.safety.reacquire_timeout_s:
                if self.on_pause(f"marker {self.cfg.geometry.marker_id} not visible") == "abort":
                    raise SessionAbort("marker not visible")
                t0 = self.clock.now()
            self.clock.sleep(0.05)

    def _led_baseline(self, t_before):
        """Median blue LED level over the second before t_before, from finite samples only; NaN when there is
        none (LED patch outside the image)."""
        h = self.poses.history(self.cfg.geometry.marker_id, since_t=t_before - 1.0)
        vals = [d.led_b for d in h if d.t < t_before and math.isfinite(d.led_b)]
        return float(np.median(vals)) if vals else float("nan")

    def _flash_seen(self, t_from, t_to, base):
        """Blue LED level in [t_from, t_to] rose more than flash_rise over base. NaN samples (LED patch outside
        the image) are skipped, so one cannot hide a flash. Without a finite base, the rise is measured over the
        window's own lowest sample (a flash lasts ~70 ms of the window)."""
        h = self.poses.history(self.cfg.geometry.marker_id, since_t=t_from)
        vals = [d.led_b for d in h if d.t <= t_to and math.isfinite(d.led_b)]
        if not vals:
            return False
        if not math.isfinite(base):
            base = min(vals)
        return max(vals) - base > self.cfg.safety.flash_rise

    # ------------------------------------------------------------------ the operator pause
    def _pause(self, seg, spec: SegmentSpec):
        """Motors off, a status line with the bot's position, and the keys of PAUSE_KEYS. Returns "run" or
        "skip"; q (or Ctrl-C as a key) raises SessionAbort. The marker may be hidden meanwhile (a hand over the
        bot): the line says so and nothing prompts."""
        self.stop(seg, "pause")
        hx, hy = self.cfg.arena.centre
        print(f"\n--- segment {seg}: ({spec.L}, {spec.R}) block {spec.block}, {spec.duration_s:g} s.  {PAUSE_KEYS}")
        t_print = -math.inf
        while True:
            k = self.getkey()
            if k in ("\r", "\n", "x", "q", "\x03"):          # msvcrt hands Ctrl-C over as a key
                print()
                if k in ("q", "\x03"):
                    raise SessionAbort("operator quit")
                return "skip" if k == "x" else "run"
            if k in ("w", "a", "d"):
                self._nudge(seg, k)
                continue
            if k in (" ", "s"):
                self.stop(seg, "nudge_stop")
            now = self.clock.now()
            if now - t_print >= 0.5:
                t_print = now
                pose = self.body_pose()
                if pose is None or self.pose_age() > self.cfg.safety.marker_loss_s:
                    where = "marker not visible"
                else:
                    _, x, y, th = pose
                    where = (f"bot at ({x:.2f}, {y:.2f}) m, {100 * math.hypot(hx - x, hy - y):.0f} cm from home, "
                             f"heading {math.degrees(th):.0f} deg")
                print(f"\r    {where:72s}", end="", flush=True)
            self.clock.sleep(0.05)

    def _nudge(self, seg, key):
        """One RECENTRE burst of nudge_s with the drive pair of the key (w: straight, a: pivot CCW, d: pivot CW),
        then STOP. Ends early when the marker is lost or the predicted pose leaves the arena fence while moving
        away from home."""
        nc, saf, arena = self.cfg.nudge, self.cfg.safety, self.cfg.arena
        hx, hy = arena.centre
        L, R = {"w": nc.straight, "a": nc.pivot_ccw, "d": nc.pivot_cw}[key]
        t0 = self.clock.now()
        result = "done"
        while self.clock.now() < t0 + nc.nudge_s:
            now = self.clock.now()
            if self.pose_age() > saf.marker_loss_s:
                result = "marker_loss"
                break
            t_pose, x, y, _ = self.body_pose()
            vx, vy, _ = self.recent_rates()
            h = self.lookahead_s() + max(0.0, now - t_pose)
            px, py = x + vx * h, y + vy * h
            if not arena.inside(px, py, min(nc.fence_m, arena.margin)) and math.hypot(hx - px, hy - py) > \
                    math.hypot(hx - x, hy - y):
                result = "geofence"
                break
            self.send(P.OP_RECENTRE, L, R, seg, f"nudge_{key}")
        self.stop(seg, "nudge_stop" if result == "done" else f"nudge_{result}")
        if result != "done":
            self.event(f"nudge_{result}", f"seg {seg} key {key}")
            print(f"\r    nudge stopped: {result:60s}")

    # ------------------------------------------------------------------ one segment
    def _log_segment(self, i, attempt, spec: SegmentSpec, rec_seg, rec_end, status, flash, moved, pose0, n):
        x0, y0, th0 = pose0 if pose0 else (float("nan"),) * 3
        self._seg_w.writerow([i, attempt, spec.kind, spec.tag, spec.block, spec.L, spec.R, json.dumps(spec.levels),
                              spec.duration_s, spec.rest_s,
                              f"{rec_seg.t_req:.6f}" if rec_seg else "", f"{rec_seg.t_ack:.6f}" if rec_seg else "",
                              f"{rec_end.t_ack:.6f}" if rec_end else "", status, int(flash), f"{moved:.4f}",
                              f"{x0:.5f}", f"{y0:.5f}", f"{th0:.5f}", n])
        self._seg_f.flush()
        self.counts[status] = self.counts.get(status, 0) + 1

    def run_static(self, i, spec):
        r0 = self.stop(i, "static")
        self.clock.sleep(spec.duration_s)
        r1 = SendRecord(self.clock.now(), self.clock.now(), 0, 0, 0)
        self._log_segment(i, 0, spec, r0, r1, "ok", False, 0.0, None, 0)

    def run_ping(self, i, spec):
        """PINGs for spec.duration_s: the first ANCHOR_PINGS each followed by ANCHOR_GAP_S of silence, then back
        to back at the blaster period. In a train of PINGs a lag and the lag plus or minus one period can match
        equally many flashes when the first or last PING is lost (extract.match_flashes), e.g. with a camera offset
        above the period; the isolated anchors' flashes match only the true lag."""
        self.wait_for_marker()
        first = last = None
        n = 0
        t_end = self.clock.now() + spec.duration_s
        while self.clock.now() < t_end:
            if last is not None and n <= ANCHOR_PINGS:
                self.clock.sleep(max(0.0, min(last.t_ack + ANCHOR_GAP_S, t_end) - self.clock.now()))
                if self.clock.now() >= t_end:
                    break
            last = self.send(P.OP_PING, 0, 0, i, "ping")
            first = first or last
            n += 1
        self._log_segment(i, 0, spec, first, last, "ok", False, 0.0, None, n)

    def run_drive(self, i, attempt, spec: SegmentSpec):
        saf, arena = self.cfg.safety, self.cfg.arena
        pose = self.body_pose()
        pose0 = pose[1:] if pose else None
        base_b = self._led_baseline(self.clock.now())
        L0, R0 = spec.level_at(0.0)
        rec_seg = self.send(P.OP_SEG, L0, R0, i, "seg")
        t0 = rec_seg.t_ack
        status, n, flash, checked = "ok", 1, False, False
        la = self.lookahead_s()
        while True:
            now = self.clock.now()
            el = now - t0
            if el >= spec.duration_s:
                break
            if self.pose_age() > saf.marker_loss_s:
                status = "marker_loss"
                break
            t_pose, x, y, th = self.body_pose()
            vx, vy, _ = self.recent_rates()
            h = la + max(0.0, now - t_pose)          # the latest pose is already pose_age old
            if not arena.inside(x + vx * h, y + vy * h):
                status = "geofence"
                break
            if not checked and el > saf.flash_window_s + 0.1:
                checked = True
                flash = self._flash_seen(t0, t0 + saf.flash_window_s + 0.1, base_b)
                moved = math.hypot(x - pose0[0], y - pose0[1]) if pose0 else 0.0
                if not flash and moved < saf.missed_motion_m and (L0 or R0):
                    status = "missed"
                    break
            L, R = spec.level_at(el + 0.5 * self.period_s())
            self.send(P.OP_DRIVE, L, R, i, "drive")
            n += 1
        rec_end = self.stop(i, "end" if status == "ok" else status)
        if not checked:
            flash = self._flash_seen(t0, t0 + saf.flash_window_s + 0.1, base_b)
        pose = self.body_pose()
        moved = math.hypot(pose[1] - pose0[0], pose[2] - pose0[1]) if (pose and pose0) else float("nan")
        self._log_segment(i, attempt, spec, rec_seg, rec_end, status, flash, moved, pose0, n)
        return status

    # ------------------------------------------------------------------ whole schedule
    def run(self, specs):
        self.meta["n_segments"] = len(specs)
        self._write_meta()
        try:
            for i, spec in enumerate(specs):
                if spec.kind == "static":
                    self.run_static(i, spec)
                    continue
                if spec.kind == "ping":
                    self.run_ping(i, spec)
                    continue
                for attempt in range(self.cfg.safety.max_retries + 1):
                    if spec.pause and self._pause(i, spec) == "skip":
                        self.event("skipped", f"seg {i}")
                        break
                    self.wait_for_marker()
                    self.stop(i, "pre_rest")
                    self.clock.sleep(spec.rest_s)
                    status = self.run_drive(i, attempt, spec)
                    if status == "missed":
                        self.event("missed", f"seg {i} attempt {attempt}")
                        continue
                    if status == "marker_loss":
                        self.event("marker_loss", f"seg {i}")
                    break
        except KeyboardInterrupt:
            self.event("abort", "keyboard interrupt")
            raise
        except SessionAbort as e:
            self.event("abort", str(e))
        finally:
            try:
                failed, n_ok = self._final_stops()
            finally:
                self.meta["counts"] = self.counts
                self.meta["finished_wall"] = time.strftime("%Y-%m-%d %H:%M:%S")
                self._write_meta()
                self.frames.close()
                self._seg_f.close()
                self._ev_f.close()
        if failed and not n_ok:                      # reached only when no other exception is propagating
            raise BridgeError(f"no final STOP was acknowledged: {failed[0]!r}") from failed[0]
        return self.counts

    def _final_stops(self):
        """STOP to the bot (stop_repeats times) and a broadcast STOP, each in its own try, so one failure does not
        skip the rest. After a failure the bridge is resynced: a Ctrl-C during the blocking read leaves the
        interrupted frame's reply (or the rest of it) in the serial buffer, which the next read would take as
        the reply to the STOP (that STOP has still been transmitted). Returns (exceptions, number acknowledged)."""
        frames = [None] * max(1, self.cfg.safety.stop_repeats) + [P.ADDR_BROADCAST]
        failed, n_ok = [], 0
        for bot in frames:
            try:
                self.send(P.OP_STOP, seg=-1, purpose="final" if bot is None else "final_broadcast", bot=bot)
                n_ok += 1
            except Exception as e:                   # a second Ctrl-C (KeyboardInterrupt) is not caught
                failed.append(e)
                self.event("final_stop_error", repr(e))
                resync = getattr(self.bridge, "resync", None)
                if resync is not None:
                    try:
                        resync()
                    except Exception as e2:
                        self.event("final_stop_error", f"resync: {e2!r}")
        return failed, n_ok
