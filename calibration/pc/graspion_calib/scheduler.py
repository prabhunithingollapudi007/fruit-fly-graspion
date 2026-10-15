"""Segment scheduler: drives one bot through a schedule over the IR blaster, with re-centring and PC-side stops.

Per segment (FEASIBILITY.md 5.2, "PC-driven segments over IR"):
  1. Wait until the marker is tracked; re-centre to the arena home with a bang-bang law (pivot, then straight)
     if the bot is farther than r_start from home.
  2. STOP and rest rest_s (settling; the firmware reads mVbat at rest when the next SEG arrives).
  3. SEG(L, R): the bot starts and flashes. Then DRIVE keep-alives (and level changes for step/PRBS segments)
     at the blaster period, so the bot's ~300 ms command watchdog never trips while the PC is healthy.
  4. STOP. A segment with no flash and no motion is retried once ("missed": the SEG frame was lost).
PC-side stops (STOP burst, segment marked): marker lost for marker_loss_s, predicted pose outside the
geofence, red low-battery flash, or any exception. On exit the scheduler sends STOP and a broadcast STOP.

UNTESTED on the rig: exercised only against sim.SimWorld (tests/test_scheduler.py).
"""
from __future__ import annotations

import csv
import json
import math
import time
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np

from . import protocol as P
from .bridge import FrameLog, SendRecord
from .geometry import Arena, BotGeometry, wrap
from .poses import PoseStore
from .schedules import SegmentSpec

SEGMENT_COLUMNS = ["seg", "attempt", "kind", "tag", "block", "L", "R", "levels", "duration_s", "rest_s",
                   "t_seg_req", "t_seg_ack", "t_end_ack", "status", "flash_live", "moved_m", "x0", "y0", "th0",
                   "n_frames"]


@dataclass
class SafetyConfig:
    marker_loss_s: float = 0.25        # no detection for this long -> stop (camera ~40-60 fps: 10+ frames)
    lookahead_s: float = 0.3           # geofence check on the pose predicted this far ahead
    recentre_timeout_s: float = 25.0
    reacquire_timeout_s: float = 5.0
    stop_repeats: int = 2
    flash_window_s: float = 0.5        # flash expected within this time after the SEG acknowledgement
    flash_rise: float = 40.0           # LED level rise over the pre-segment baseline that counts as a flash
    missed_motion_m: float = 0.003     # less displacement than this + no flash -> the SEG was missed
    max_retries: int = 1


@dataclass
class RecentreConfig:
    r_done: float = 0.05               # re-centring stops inside this radius of home [m]
    r_start: float = 0.08              # re-centre before a segment if farther than this
    e_enter_deg: float = 35.0          # bang-bang with hysteresis on the heading error to home
    e_exit_deg: float = 12.0
    pivot_ccw: tuple = (0, 90)         # right motor only -> turns left (CCW), paper / HARDWARE.md 4.4
    pivot_cw: tuple = (90, 0)
    straight: tuple = (85, 85)         # equal drive drifts CW on the twin's bot; the loop corrects it
    latency_s: float = 0.2             # heading prediction horizon (camera + IR latency)


@dataclass
class SessionConfig:
    bot_id: int
    geometry: BotGeometry
    arena: Arena
    out_dir: str
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    recentre: RecentreConfig = field(default_factory=RecentreConfig)
    notes: str = ""


class SessionAbort(RuntimeError):
    pass


def console_pause(msg: str) -> str:
    """Default operator prompt: returns 'continue' or 'abort'."""
    print(f"\n*** PAUSED: {msg}")
    ans = input("    [Enter] continue, 'a' abort: ").strip().lower()
    return "abort" if ans.startswith("a") else "continue"


class Session:
    def __init__(self, cfg: SessionConfig, bridge, poses: PoseStore, clock, on_pause=console_pause, extra_meta=None):
        self.cfg, self.bridge, self.poses, self.clock = cfg, bridge, poses, clock
        self.on_pause = on_pause
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
                     "safety": asdict(cfg.safety), "recentre": asdict(cfg.recentre), "notes": cfg.notes,
                     "started_wall": time.strftime("%Y-%m-%d %H:%M:%S"), "proto": P.PROTO_VERSION,
                     "bridge_info": getattr(bridge, "info", "")}
        if extra_meta:
            self.meta.update(extra_meta)
        self._write_meta()
        self.counts = {}

    # ------------------------------------------------------------------ logging
    def _write_meta(self):
        with open(self.out / "session.json", "w") as f:
            json.dump(self.meta, f, indent=2, default=str)

    def event(self, kind, detail=""):
        self._ev_w.writerow([f"{self.clock.now():.6f}", kind, detail])
        self._ev_f.flush()
        self.counts[kind] = self.counts.get(kind, 0) + 1

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
        h = self.poses.history(self.cfg.geometry.marker_id, since_t=t_before - 1.0)
        h = [d for d in h if d.t < t_before]
        if not h:
            return 0.0, 0.0
        return float(np.median([d.led_b for d in h])), float(np.median([d.led_r for d in h]))

    def _flash_seen(self, t_from, t_to, base, channel="b"):
        h = self.poses.history(self.cfg.geometry.marker_id, since_t=t_from)
        vals = [getattr(d, "led_" + channel) for d in h if d.t <= t_to]
        return bool(vals) and max(vals) - base > self.cfg.safety.flash_rise

    # ------------------------------------------------------------------ re-centring
    def recentre(self, seg):
        rc, saf, arena = self.cfg.recentre, self.cfg.safety, self.cfg.arena
        hx, hy = arena.centre
        mode = "straight"
        t0 = self.clock.now()
        n = 0
        while True:
            if self.pose_age() > saf.marker_loss_s:
                self.stop(seg, "recentre_marker_loss")
                self.event("marker_loss", f"recentre seg {seg}")
                self.wait_for_marker()
                continue
            _, x, y, th = self.body_pose()
            dist = math.hypot(hx - x, hy - y)
            if dist < rc.r_done:
                self.stop(seg, "recentre_done")
                self.event("recentred", f"seg {seg} frames {n} in {self.clock.now() - t0:.1f}s")
                return True
            if self.clock.now() - t0 > saf.recentre_timeout_s:
                self.stop(seg, "recentre_timeout")
                self.event("recentre_timeout", f"seg {seg} dist {dist:.3f}")
                return False
            _, _, w = self.recent_rates()
            e = wrap(math.atan2(hy - y, hx - x) - (th + w * rc.latency_s))
            if mode == "pivot" and abs(e) < math.radians(rc.e_exit_deg):
                mode = "straight"
            elif mode == "straight" and abs(e) > math.radians(rc.e_enter_deg):
                mode = "pivot"
            L, R = rc.straight if mode == "straight" else (rc.pivot_ccw if e > 0 else rc.pivot_cw)
            self.send(P.OP_RECENTRE, L, R, seg, f"recentre_{mode}")
            n += 1

    def _ensure_home(self, seg):
        hx, hy = self.cfg.arena.centre
        while True:
            self.wait_for_marker()
            _, x, y, _ = self.body_pose()
            if math.hypot(hx - x, hy - y) <= self.cfg.recentre.r_start or self.recentre(seg):
                return
            if self.on_pause(f"re-centring failed before segment {seg}: place the bot near home") == "abort":
                raise SessionAbort("re-centring failed")

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
        self.wait_for_marker()
        first = last = None
        n = 0
        t_end = self.clock.now() + spec.duration_s
        while self.clock.now() < t_end:
            last = self.send(P.OP_PING, 0, 0, i, "ping")
            first = first or last
            n += 1
        self._log_segment(i, 0, spec, first, last, "ok", False, 0.0, None, n)

    def run_drive(self, i, attempt, spec: SegmentSpec):
        saf, arena, g = self.cfg.safety, self.cfg.arena, self.cfg.geometry
        pose = self.body_pose()
        pose0 = pose[1:] if pose else None
        base_b, base_r = self._led_baseline(self.clock.now())
        L0, R0 = spec.level_at(0.0)
        rec_seg = self.send(P.OP_SEG, L0, R0, i, "seg")
        t0 = rec_seg.t_ack
        status, n, flash, checked = "ok", 1, False, False
        while True:
            now = self.clock.now()
            el = now - t0
            if el >= spec.duration_s:
                break
            if self.pose_age() > saf.marker_loss_s:
                status = "marker_loss"
                break
            _, x, y, _ = self.body_pose()
            vx, vy, _ = self.recent_rates()
            if not arena.inside(x + vx * saf.lookahead_s, y + vy * saf.lookahead_s):
                status = "geofence"
                break
            if self._flash_seen(t0, now, base_r, "r"):
                status = "lowbat"
                break
            if not checked and el > saf.flash_window_s + 0.1:
                checked = True
                flash = self._flash_seen(t0, t0 + saf.flash_window_s + 0.1, base_b, "b")
                moved = math.hypot(x - pose0[0], y - pose0[1]) if pose0 else 0.0
                if not flash and moved < saf.missed_motion_m and (L0 or R0):
                    status = "missed"
                    break
            L, R = spec.level_at(el + 0.5 * getattr(self.bridge, "period_s", 0.11))
            self.send(P.OP_DRIVE, L, R, i, "drive")
            n += 1
        rec_end = self.stop(i, "end" if status == "ok" else status)
        if not checked:
            flash = self._flash_seen(t0, t0 + saf.flash_window_s + 0.1, base_b, "b")
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
                    self.wait_for_marker()
                    if spec.recentre:
                        self._ensure_home(i)
                    self.stop(i, "pre_rest")
                    self.clock.sleep(spec.rest_s)
                    status = self.run_drive(i, attempt, spec)
                    if status == "lowbat":
                        self.event("lowbat", f"seg {i}")
                        if self.on_pause("red flash: low battery latched. Charge or swap, reset the bot "
                                         "(or send 'r' over USB), then continue") == "abort":
                            raise SessionAbort("low battery")
                        continue
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
                self.stop(-1, "final")
                self.send(P.OP_STOP, seg=-1, purpose="final_broadcast", bot=P.ADDR_BROADCAST)
            finally:
                self.meta["counts"] = self.counts
                self.meta["finished_wall"] = time.strftime("%Y-%m-%d %H:%M:%S")
                self._write_meta()
                self.frames.close()
                self._seg_f.close()
                self._ev_f.close()
        return self.counts
