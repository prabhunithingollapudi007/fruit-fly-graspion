"""Segment scheduler: drives one bot through a schedule over the IR blaster, with re-centring and PC-side stops.

Per segment (FEASIBILITY.md 5.2, "PC-driven segments over IR"):
  1. Wait until the marker is tracked; re-centre to the arena home with a bang-bang law (pivot, then straight)
     if the bot is farther than r_start from home.
  2. STOP and rest rest_s (settling; the firmware reads mVbat at rest when the next SEG arrives).
  3. SEG(L, R): the bot starts and flashes. Then DRIVE keep-alives (and level changes for step/PRBS segments)
     at the blaster period, so the bot's ~300 ms command watchdog never trips while the PC is healthy.
  4. STOP. A segment with no flash and no motion is retried once ("missed": the SEG frame was lost).
PC-side stops (STOP burst, segment marked): marker lost for marker_loss_s, predicted pose outside the
geofence, red low-battery flash, or any exception. Re-centring has its own stops: marker loss, red flash,
predicted pose leaving the arena while moving away from home, no progress, diverging, timeout.
On exit the scheduler sends STOP to the bot and a broadcast STOP, each in its own try (resyncing the bridge
after a failure, e.g. a Ctrl-C that left half a reply in the serial buffer); the original exception propagates.
The LED checks skip NaN samples (LED patch outside the image, or led_r on a mono camera).

Mono camera (tracker led_mode 'luma', led_r NaN): a red low-battery flash cannot be told from a sync flash, so the
PC-side low-battery stop cannot fire. The session warns once and relies on the bot's own latch: after 3 low mVbat
readings the bot stops its motors and answers every SEG, RECENTRE or PING with a flash and no motion (and logs no
record for it). The PC then sees re-centring without progress (operator pause with a low-battery hint), or a
segment that flashes but neither moves nor turns although a motor is driven at stall_drive or more (status
'stalled', the low-battery pause, retried like 'lowbat'). Weaker segments between the latch and that point stay
'ok' with no motion: drop them by hand (moved_m in segments.csv).

Manual re-centring (SessionConfig.recentre_mode 'manual', `run --recentre manual`): instead of driving the bot home,
the session stops the motors and pauses before every segment. The operator places the bot by hand or nudges it
with the keys (short RECENTRE bursts through the same bridge, under the fence, marker-loss and red-flash stops),
then presses Enter to run the segment from where the bot stands, x to skip it or q to end the session. Nothing
waits for the marker during that pause (a hand over the bot hides it); the segment itself still does. For trial
runs, single cells and a campaign whose automatic re-centring is not yet tuned.

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


@dataclass
class SafetyConfig:
    marker_loss_s: float = 0.25        # no detection for this long -> stop (camera ~40-60 fps: 10+ frames)
    lookahead_s: float | None = None   # geofence check on the pose predicted this far ahead; None: stop_latency_s
    cam_latency_s: float = 0.05        # exposure -> frame read (pose times are read times); INFERRED, measure it
    stop_tail_s: float = 0.1           # after the STOP frame: decode gap (8 ms), dead time and motor lag
    recentre_timeout_s: float = 25.0
    reacquire_timeout_s: float = 5.0
    stop_repeats: int = 2
    flash_window_s: float = 0.5        # flash expected within this time after the SEG acknowledgement
    flash_rise: float = 40.0           # LED level rise over the pre-segment baseline that counts as a flash
    missed_motion_m: float = 0.003     # less displacement than this + no flash -> the SEG was missed
    max_retries: int = 1
    stall_drive: int = 90              # mono camera only: a full segment with a flash and |drive| >= this on a
    stall_turn_rad: float = 0.1        # motor that never got missed_motion_m from its start nor turned this far is
                                       # "stalled": a latched bot answers SEG with a flash and no motion. 90 is well
                                       # above any dead zone; the turn test keeps spins in place from counting

    def stop_latency_s(self, period_s):
        """Pose exposure -> bot at rest after the PC reacts to it: camera latency, up to one period until the next
        check (the loop is paced by the blocking send), up to one period until the STOP has left the blaster,
        then decode and spin-down. Lengthening the blaster period therefore needs a longer geofence lookahead."""
        if self.lookahead_s is not None:
            return self.lookahead_s
        return self.cam_latency_s + 2 * period_s + self.stop_tail_s


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
    # Stops for wrong guesses above or a wrong marker_yaw (sign errors steer the bot away from home):
    fence_m: float = 0.03              # geofence: the predicted pose must stay this far inside the arena (about the
                                       # body half-length), checked only while it moves away from home. Not the
                                       # segment margin: a segment may end at the margin line, and a one-motor pivot
                                       # moves the CoM on a circle of a few cm (~9 cm across in the simulator)
    progress_m: float = 0.02           # stop if the distance to home has not dropped by progress_m ...
    progress_s: float = 4.0            # ... within progress_s (e.g. swapped pivot pairs: the bot spins facing away)
    diverge_m: float = 0.03            # stop if the distance grows by this during one straight run (e.g. marker_yaw
                                       # off by pi: the bot drives straight away from home)
    nudge_s: float = 0.6               # manual mode: one key press drives this long (a few cm), then STOP


@dataclass
class SessionConfig:
    bot_id: int
    geometry: BotGeometry
    arena: Arena
    out_dir: str
    safety: SafetyConfig = field(default_factory=SafetyConfig)
    recentre: RecentreConfig = field(default_factory=RecentreConfig)
    recentre_mode: str = "auto"        # 'auto': bang-bang re-centring; 'manual': the operator pause (module doc)
    notes: str = ""
    led_mode: str = "auto"             # tracker.led_mode: 'chroma' (colour), 'luma' (mono: no red channel) or
                                       # 'auto' (from the first detection with a finite led_b: led_r NaN = mono)


MONO_WARNING = ("mono camera (tracker led_mode 'luma', led_r = NaN): a red low-battery flash looks like a sync "
                "flash, so the PC-side low-battery stop is off. The bot still stops itself (low-battery latch: "
                "motors off, every SEG / RECENTRE / PING answered with a flash and no motion). The session pauses "
                "when re-centring makes no progress or a strong segment flashes without moving ('stalled'); "
                "segments with no motion before that are logged 'ok': check moved_m in segments.csv")


class SessionAbort(RuntimeError):
    pass


def console_pause(msg: str) -> str:
    """Default operator prompt: returns 'continue' or 'abort'."""
    print(f"\n*** PAUSED: {msg}")
    ans = input("    [Enter] continue, 'a' abort: ").strip().lower()
    return "abort" if ans.startswith("a") else "continue"


MANUAL_KEYS = "w/a/d (or arrows): nudge forward / pivot left / pivot right, space: stop, Enter: run, x: skip, q: quit"


def console_getkey():
    """A non-blocking key reader for the manual pause: the next key, or None when none is pressed. Arrow keys map
    to w/a/d/s. Windows reads the console through msvcrt; elsewhere stdin is put into cbreak mode for each poll
    only, so input() (the other pauses) still works between polls."""
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
    def __init__(self, cfg: SessionConfig, bridge, poses: PoseStore, clock, on_pause=console_pause, extra_meta=None,
                 getkey=None):
        if cfg.recentre_mode not in ("auto", "manual"):
            raise ValueError(f"recentre_mode {cfg.recentre_mode!r}: 'auto' or 'manual'")
        if cfg.recentre_mode == "manual" and getkey is None:
            raise ValueError("recentre_mode 'manual' needs a getkey reader (console_getkey())")
        self.cfg, self.bridge, self.poses, self.clock = cfg, bridge, poses, clock
        self.on_pause, self.getkey = on_pause, getkey
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
                     "safety": asdict(cfg.safety), "recentre": asdict(cfg.recentre),
                     "recentre_mode": cfg.recentre_mode, "notes": cfg.notes,
                     "started_wall": time.strftime("%Y-%m-%d %H:%M:%S"), "proto": P.PROTO_VERSION,
                     "bridge_info": getattr(bridge, "info", ""), "period_s": self.period_s(),
                     "geofence_lookahead_s": self.lookahead_s(), "led_mode": cfg.led_mode}
        if extra_meta:
            self.meta.update(extra_meta)
        self.led_mode = cfg.led_mode
        self._mono_warned = False
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

    def _mono(self):
        """True on a mono camera (led_mode 'luma'), where no red low-battery flash can be seen. 'auto' is resolved
        from the first detection with a finite led_b: led_r is NaN there only in the tracker's luma mode (an LED
        patch outside the image gives NaN in both). Warns once (MONO_WARNING)."""
        if self.led_mode == "auto":
            for d in self.poses.history(self.cfg.geometry.marker_id):
                if math.isfinite(d.led_b):
                    self.led_mode = "chroma" if math.isfinite(d.led_r) else "luma"
                    self.meta["led_mode"] = self.led_mode
                    break
        if self.led_mode == "luma" and not self._mono_warned:
            self._mono_warned = True
            self.warn(MONO_WARNING)
        return self.led_mode == "luma"

    def _led_baseline(self, t_before):
        """Median (blue, red) LED level over the second before t_before, from finite samples only; NaN when there
        is none (e.g. led_r on a mono camera)."""
        h = self.poses.history(self.cfg.geometry.marker_id, since_t=t_before - 1.0)
        h = [d for d in h if d.t < t_before]
        out = []
        for c in ("b", "r"):
            vals = [v for v in (getattr(d, "led_" + c) for d in h) if math.isfinite(v)]
            out.append(float(np.median(vals)) if vals else float("nan"))
        return tuple(out)

    def _flash_seen(self, t_from, t_to, base, channel="b"):
        """LED level in [t_from, t_to] rose more than flash_rise over base. NaN samples (LED patch outside the
        image) are skipped, so one cannot hide a flash. Without a finite base, the rise is measured over the
        window's own lowest sample (a flash lasts ~70 ms of the window). That fallback needs a window with no
        flash of the other colour: a blue sync flash pulls led_r = R - (G+B)/2 down by about half its rise, so
        the dark level over that dip would read as a red flash. Re-centring windows have none (RECENTRE and STOP
        never flash blue); run_drive checks red only against a finite baseline."""
        h = self.poses.history(self.cfg.geometry.marker_id, since_t=t_from)
        vals = [v for v in (getattr(d, "led_" + channel) for d in h if d.t <= t_to) if math.isfinite(v)]
        if not vals:
            return False
        if not math.isfinite(base):
            base = min(vals)
        return max(vals) - base > self.cfg.safety.flash_rise

    # ------------------------------------------------------------------ re-centring
    def recentre(self, seg):
        """Bang-bang re-centring to home. Returns "done", or why it stopped the bot: "lowbat" (red flash),
        "geofence", "diverging", "no_progress" or "timeout" (see RecentreConfig)."""
        rc, saf, arena = self.cfg.recentre, self.cfg.safety, self.cfg.arena
        hx, hy = arena.centre
        mode = "straight"
        t0 = self.clock.now()
        _, base_r = self._led_baseline(t0)
        n = 0
        best = t_best = d_straight = None
        while True:
            if self.pose_age() > saf.marker_loss_s:
                self.stop(seg, "recentre_marker_loss")
                self.event("marker_loss", f"recentre seg {seg}")
                self.wait_for_marker()
                best = d_straight = None             # restart the progress checks after the wait
                continue
            now = self.clock.now()
            t_pose, x, y, th = self.body_pose()
            dist = math.hypot(hx - x, hy - y)
            if dist < rc.r_done:
                result = "done"
                break
            if not self._mono() and self._flash_seen(t0, now, base_r, "r"):  # a latched bot answers RECENTRE
                result = "lowbat"                                               # with a red flash only
                break
            if now - t0 > saf.recentre_timeout_s:
                result = "timeout"
                break
            if best is None or dist < best - rc.progress_m:
                best, t_best = dist, now
            elif now - t_best > rc.progress_s:
                result = "no_progress"
                break
            vx, vy, w = self.recent_rates()
            h = self.lookahead_s() + max(0.0, now - t_pose)
            px, py = x + vx * h, y + vy * h
            if not arena.inside(px, py, min(rc.fence_m, arena.margin)) and math.hypot(hx - px, hy - py) > dist:
                result = "geofence"
                break
            e = wrap(math.atan2(hy - y, hx - x) - (th + w * rc.latency_s))
            if mode == "pivot" and abs(e) < math.radians(rc.e_exit_deg):
                mode = "straight"
            elif mode == "straight" and abs(e) > math.radians(rc.e_enter_deg):
                mode = "pivot"
            if mode == "straight":                   # heading within e_enter of home: the distance must shrink
                d_straight = dist if d_straight is None else min(d_straight, dist)
                if dist > d_straight + rc.diverge_m:
                    result = "diverging"
                    break
            else:
                d_straight = None
            L, R = rc.straight if mode == "straight" else (rc.pivot_ccw if e > 0 else rc.pivot_cw)
            self.send(P.OP_RECENTRE, L, R, seg, f"recentre_{mode}")
            n += 1
        self.stop(seg, f"recentre_{result}")
        detail = f"seg {seg} frames {n} in {self.clock.now() - t0:.1f}s dist {dist:.3f}"
        self.event("recentred" if result == "done" else f"recentre_{result}", detail)
        return result

    def _ensure_home(self, seg):
        hx, hy = self.cfg.arena.centre
        while True:
            self.wait_for_marker()
            _, x, y, _ = self.body_pose()
            if math.hypot(hx - x, hy - y) <= self.cfg.recentre.r_start:
                return
            result = self.recentre(seg)
            if result == "done":
                return
            if result == "lowbat":
                self._lowbat_pause(seg)
                continue
            hint = (" (mono camera: a low-battery latch looks like this, check the battery)"
                    if result == "no_progress" and self._mono() else "")
            if self.on_pause(f"re-centring failed ({result}) before segment {seg}: place the bot near home{hint}") \
                    == "abort":
                raise SessionAbort(f"re-centring failed ({result})")

    def _lowbat_pause(self, seg, status="lowbat"):
        self.event(status, f"seg {seg}")
        why = ("red flash: low battery latched" if status == "lowbat" else
               f"segment {seg} flashed but did not move: on a mono camera this is how the low-battery latch looks")
        if self.on_pause(f"{why}. Charge or swap, reset the bot (or send 'r' over USB), then continue") == "abort":
            raise SessionAbort("low battery")

    # ------------------------------------------------------------------ manual re-centring
    def _manual_pause(self, seg, spec: SegmentSpec):
        """The operator pause of recentre_mode 'manual' (module doc): motors off, a status line with the bot's
        position, and the keys of MANUAL_KEYS. Returns "run" or "skip"; q raises SessionAbort. The marker may be
        hidden meanwhile (a hand over the bot): the line says so and nothing prompts."""
        self.stop(seg, "manual")
        hx, hy = self.cfg.arena.centre
        print(f"\n--- segment {seg}: ({spec.L}, {spec.R}) block {spec.block}, {spec.duration_s:g} s.  {MANUAL_KEYS}")
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
                self.stop(seg, "manual_stop")
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
        """One RECENTRE burst of nudge_s with the re-centring drive pair of the key (w: straight, a: pivot CCW,
        d: pivot CW), then STOP. Ends early when the marker is lost, the predicted pose leaves the arena fence
        while moving away from home, or a red flash answers (low battery: the usual pause)."""
        rc, saf, arena = self.cfg.recentre, self.cfg.safety, self.cfg.arena
        hx, hy = arena.centre
        L, R = {"w": rc.straight, "a": rc.pivot_ccw, "d": rc.pivot_cw}[key]
        t0 = self.clock.now()
        _, base_r = self._led_baseline(t0)
        result = "done"
        while self.clock.now() < t0 + rc.nudge_s:
            now = self.clock.now()
            if self.pose_age() > saf.marker_loss_s:
                result = "marker_loss"
                break
            t_pose, x, y, _ = self.body_pose()
            vx, vy, _ = self.recent_rates()
            h = self.lookahead_s() + max(0.0, now - t_pose)
            px, py = x + vx * h, y + vy * h
            if not arena.inside(px, py, min(rc.fence_m, arena.margin)) and math.hypot(hx - px, hy - py) > \
                    math.hypot(hx - x, hy - y):
                result = "geofence"
                break
            if not self._mono() and self._flash_seen(t0, now, base_r, "r"):
                result = "lowbat"
                break
            self.send(P.OP_RECENTRE, L, R, seg, f"manual_{key}")
        self.stop(seg, "manual_stop" if result == "done" else f"manual_{result}")
        if result != "done":
            self.event(f"manual_{result}", f"seg {seg} key {key}")
            print(f"\r    nudge stopped: {result:60s}")
        if result == "lowbat":
            self._lowbat_pause(seg)

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
        saf, arena, g = self.cfg.safety, self.cfg.arena, self.cfg.geometry
        pose = self.body_pose()
        pose0 = pose[1:] if pose else None
        base_b, base_r = self._led_baseline(self.clock.now())
        L0, R0 = spec.level_at(0.0)
        rec_seg = self.send(P.OP_SEG, L0, R0, i, "seg")
        t0 = rec_seg.t_ack
        status, n, flash, checked = "ok", 1, False, False
        reach = turn = 0.0                           # largest displacement and heading change from pose0
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
            if pose0:
                reach = max(reach, math.hypot(x - pose0[0], y - pose0[1]))
                turn = max(turn, abs(wrap(th - pose0[2])))
            vx, vy, _ = self.recent_rates()
            h = la + max(0.0, now - t_pose)          # the latest pose is already pose_age old
            if not arena.inside(x + vx * h, y + vy * h):
                status = "geofence"
                break
            if not self._mono():
                if not math.isfinite(base_r) and el > saf.flash_window_s + 0.1:
                    # no red level before the SEG (LED patch outside the image): the median of the last second,
                    # where the ~70 ms sync flash (a dip in led_r) or red flash is a minority. Not the lowest sample
                    base_r = self._led_baseline(now)[1]
                if math.isfinite(base_r) and self._flash_seen(t0, now, base_r, "r"):
                    status = "lowbat"
                    break
            if not checked and el > saf.flash_window_s + 0.1:
                checked = True
                flash = self._flash_seen(t0, t0 + saf.flash_window_s + 0.1, base_b, "b")
                moved = math.hypot(x - pose0[0], y - pose0[1]) if pose0 else 0.0
                if not flash and moved < saf.missed_motion_m and (L0 or R0):
                    status = "missed"
                    break
            L, R = spec.level_at(el + 0.5 * self.period_s())
            self.send(P.OP_DRIVE, L, R, i, "drive")
            n += 1
        rec_end = self.stop(i, "end" if status == "ok" else status)
        if not checked:
            flash = self._flash_seen(t0, t0 + saf.flash_window_s + 0.1, base_b, "b")
        strong = (max(max(abs(lv[1]), abs(lv[2])) for lv in spec.levels) >= saf.stall_drive
                  and spec.duration_s >= 1.0)        # long enough to move whatever the lag
        if (status == "ok" and flash and pose0 and strong and reach < saf.missed_motion_m
                and turn < saf.stall_turn_rad and self._mono()):
            status = "stalled"                       # mono camera: how the low-battery latch looks (module doc)
        pose = self.body_pose()
        moved = math.hypot(pose[1] - pose0[0], pose[2] - pose0[1]) if (pose and pose0) else float("nan")
        self._log_segment(i, attempt, spec, rec_seg, rec_end, status, flash, moved, pose0, n)
        return status

    # ------------------------------------------------------------------ whole schedule
    def run(self, specs):
        self.meta["n_segments"] = len(specs)
        self._write_meta()
        try:
            self._mono()                             # warns at the start when the camera is known to be mono
            for i, spec in enumerate(specs):
                if spec.kind == "static":
                    self.run_static(i, spec)
                    continue
                if spec.kind == "ping":
                    self.run_ping(i, spec)
                    continue
                for attempt in range(self.cfg.safety.max_retries + 1):
                    if spec.recentre and self.cfg.recentre_mode == "manual":
                        if self._manual_pause(i, spec) == "skip":
                            self.event("skipped", f"seg {i}")
                            break
                        self.wait_for_marker()
                    else:
                        self.wait_for_marker()
                        if spec.recentre:
                            self._ensure_home(i)
                    self.stop(i, "pre_rest")
                    self.clock.sleep(spec.rest_s)
                    status = self.run_drive(i, attempt, spec)
                    if status in ("lowbat", "stalled"):
                        self._lowbat_pause(i, status)
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
