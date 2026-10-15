"""Synthetic GRASPion rig for offline tests and dry runs (`python -m graspion_calib run --sim`).

Pieces, each mirroring the real one closely enough to exercise the pipeline end to end:
  SimFirmware  frame handling of firmware/calib_bot/calib_bot.ino (CRC, address filter, ops, command watchdog,
               low-battery latch, sync flash, log records with a drifting bot clock)
  SimPlant     EllipticalBot2D kinematics (tilemat_control_stack Ellipse2dObj.py: rotation about a CoR offset
               laterally from the CoM by d_cw / d_ccw, plus translation along the heading), driven by a
               dead-zone power-law truth map with dead time, first-order lag and heading diffusion
  SimWorld     physics at 2 ms, the shared IR channel (68 ms frames, random loss) and a camera with latency,
               noise, dropouts and the sync-LED brightness
  SimBridge    the SerialBridge API on top of SimWorld

Ground truth for the tests: the truth map outputs (v, u2, omega) where v is EllipticalBot2D's translational
speed; the CoM body twist is then (v + d |omega|, u2, omega) with d = d_ccw if omega > 0 else d_cw.
Every number here is synthetic: nothing is a GRASPion measurement.
"""
from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field

import numpy as np

from . import protocol as P
from .bridge import SendRecord
from .forward_map import PowerLawMap
from .geometry import BotGeometry, transfer_twist, wrap
from .poses import Detection, PoseStore

NEC_FRAME_S = 0.0675


def default_truth() -> PowerLawMap:
    """Synthetic truth loosely shaped like the digital-twin envelope (4-12 cm/s, CW bias at L = R)."""
    return PowerLawMap(p0=(52.0, 50.0), gamma=(0.65, 0.60),
                       coef=np.array([[0.012, 0.0040, 0.0036],     # v [m/s]
                                      [0.0, 0.0002, -0.0002],      # u2 [m/s]
                                      [-0.25, -0.26, 0.20]]))      # omega [rad/s]


@dataclass
class SimParams:
    truth: PowerLawMap = field(default_factory=default_truth)
    d_cor: tuple = (0.0145, 0.0161)          # EllipticalBot2D dist_rotPoint_to_CoM [CW, CCW], m
    tau: float = 0.06                        # first-order lag, s
    dead: float = 0.03                       # mechanical dead time after decode, s
    D_r: float = 0.01                        # heading diffusion, rad^2/s
    D_t: float = 2e-7                        # translational diffusion, m^2/s
    geometry: BotGeometry = field(default_factory=lambda: BotGeometry(10, (0.004, -0.002), 0.03, "CoM"))
    cam_fps: float = 60.0
    cam_latency: float = 0.05
    pos_noise: float = 3e-4
    th_noise: float = 0.004
    p_drop: float = 0.01
    ir_loss: float = 0.02
    decode_delay: float = 0.005
    usb_latency: float = 0.002
    v_bat0: float = 4.05
    v_drain: float = 2.5e-4                  # V per second at full drive on both motors
    v_sag: float = 0.12                      # V under full load
    clock_err: float = 0.012                 # bot millis() runs 1.2 % fast
    watchdog_s: float = 0.3
    drive_max: int = 120
    led_base: tuple = (30.0, 25.0)           # (blue, red) background at the LED spot
    led_flash: float = 120.0
    flash_s: float = 0.07
    mvbat_abort: float = 3.4


class SimFirmware:
    """Frame semantics of calib_bot.ino. Times in host seconds; the log uses the drifting bot clock (ms)."""

    def __init__(self, bot_id, p: SimParams, plant):
        self.id, self.p, self.plant = bot_id, p, plant
        self.L = self.R = 0
        self.seg_active = False
        self.rec = None
        self.log = []
        self.last_frame = -1e9
        self.lowbat = False
        self.low_count = 0
        self.flash_until = -1.0
        self.flash_red = False
        self.stats = {"rx_ok": 0, "crc_err": 0, "other_addr": 0, "wd_trips": 0, "drive_no_seg": 0}
        self._next_pwr = 0.0

    def bot_ms(self, t):
        return int(t * 1000 * (1 + self.p.clock_err)) + 12345

    def _set(self, L, R, t):
        L = max(-self.p.drive_max, min(self.p.drive_max, L))
        R = max(-self.p.drive_max, min(self.p.drive_max, R))
        self.L, self.R = L, R
        self.plant.command(t + self.p.dead, L, R)

    def _close(self, reason, t):
        self.seg_active = False
        if self.rec is not None:
            self.rec["t_end_ms"] = self.bot_ms(t)
            self.rec["end_reason"] = reason
            self.rec = None

    def _flash(self, t, red=False):
        self.flash_until, self.flash_red = t + self.p.flash_s, red

    def on_frame(self, raw, t):
        if P.check(raw) != P.OK:
            self.stats["crc_err"] += 1
            return
        f = P.unpack(raw)
        if f.bot_id == P.ADDR_BROADCAST:
            if f.op == P.OP_STOP:
                self._close(3, t)
                self._set(0, 0, t)
            return
        if f.bot_id != self.id:
            self.stats["other_addr"] += 1
            return
        self.stats["rx_ok"] += 1
        self.last_frame = t
        if self.lowbat:
            self._set(0, 0, t)
            if f.op not in (P.OP_STOP, P.OP_DRIVE):
                self._flash(t, red=True)
            return
        if f.op == P.OP_STOP:
            self._close(3, t)
            self._set(0, 0, t)
        elif f.op == P.OP_SEG:
            self._close(1, t)
            self._set(f.L, f.R, t)
            self.seg_active = True
            self.rec = {"kind": 1, "t_start_ms": self.bot_ms(t), "t_end_ms": self.bot_ms(t), "L": self.L, "R": self.R,
                        "end_reason": 0, "n_frames": 1, "mv_rest": int(1000 * self.plant.battery(loaded=False))}
            self.log.append(self.rec)
            self._flash(t)
        elif f.op == P.OP_DRIVE:
            if not self.seg_active:
                self.stats["drive_no_seg"] += 1
                return
            if self.rec is not None:
                self.rec["n_frames"] += 1
            self._set(f.L, f.R, t)
        elif f.op == P.OP_RECENTRE:
            self._close(2, t)
            self._set(f.L, f.R, t)
        elif f.op == P.OP_PING:
            self.log.append({"kind": 2, "t_start_ms": self.bot_ms(t), "t_end_ms": self.bot_ms(t), "L": self.L,
                             "R": self.R, "end_reason": 6, "n_frames": 1,
                             "mv_rest": int(1000 * self.plant.battery(loaded=False))})
            self._flash(t)

    def tick(self, t):
        moving = self.L != 0 or self.R != 0
        if moving and t - self.last_frame > self.p.watchdog_s:
            self._close(4, t)
            self._set(0, 0, t)
            self.stats["wd_trips"] += 1
        if t >= self._next_pwr:
            self._next_pwr = t + (0.2 if moving else 1.0)
            v = self.plant.battery(loaded=moving)
            if v < self.p.mvbat_abort:
                self.low_count += 1
                if self.low_count >= 3 and not self.lowbat:
                    self._close(5, t)
                    self._set(0, 0, t)
                    self.lowbat = True
                    self._flash(t, red=True)
            else:
                self.low_count = 0

    def led(self, t):
        on = t < self.flash_until
        return (on and not self.flash_red), (on and self.flash_red)

    DUMP_COLUMNS = ("kind,t_start_ms,t_end_ms,L,R,end_reason,n_frames,n_changes,flags,mv_rest,mv_load_min,"
                    "mv_load_mean,n_mag,mag_rest_x,mag_rest_y,mag_rest_z,mag_load_x,mag_load_y,mag_load_z")

    def dump_text(self):
        """The log in calib_bot's USB dump format ('d'), so botlog.py can be tested without a bot."""
        lines = [f"# I fw=sim proto=1 id={self.id} id_override=0 serial=SIM pwm_hz=sim drive_max={self.p.drive_max}",
                 "# S " + " ".join(f"{k}={v}" for k, v in self.stats.items()), self.DUMP_COLUMNS]
        for r in self.log:
            mv = r.get("mv_rest", 0)
            lines.append(",".join(str(v) for v in (
                r["kind"], r["t_start_ms"], r["t_end_ms"], r["L"], r["R"], r["end_reason"], r["n_frames"], 0, 0, mv,
                mv - 120, mv - 100, 0, 32767, 32767, 32767, 32767, 32767, 32767)))
        lines.append(f"# end n={len(self.log)}")
        return "\n".join(lines)


class SimPlant:
    """EllipticalBot2D kinematics at the CoM with lag, dead time and noise (pure-Python loop for speed)."""

    def __init__(self, p: SimParams, pose=(0.5, 0.5, 0.0), rng=None):
        self.p = p
        self.x, self.y, self.th = pose
        self.v = self.u2 = self.w = 0.0
        self.target = (0.0, 0.0, 0.0)
        self.pending = []                    # heap of (t_effect, seq, L, R)
        self._seq = 0
        self.rng = rng or np.random.default_rng(0)
        self.energy = 0.0
        self.load = 0.0
        fL, fR = p.truth.activations(p.drive_max, p.drive_max)
        self._fmax = float(fL + fR)
        self._noise = self.rng.standard_normal((4096, 3))
        self._k = 0

    def command(self, t_eff, L, R):
        self._seq += 1
        heapq.heappush(self.pending, (t_eff, self._seq, L, R))

    def battery(self, loaded):
        return self.p.v_bat0 - self.p.v_drain * self.energy - (self.p.v_sag * self.load if loaded else 0.0)

    def truth_twist_com(self, L, R):
        v, u2, w = self.p.truth.predict(L, R)[0]
        d = self.p.d_cor[1] if w > 0 else self.p.d_cor[0]
        return v + d * abs(w), u2, w

    def step(self, t, dt):
        while self.pending and self.pending[0][0] <= t:
            _, _, L, R = heapq.heappop(self.pending)
            self.target = tuple(self.p.truth.predict(L, R)[0])
            fL, fR = self.p.truth.activations(L, R)
            self.load = float(fL + fR) / self._fmax
        a = 1.0 - math.exp(-dt / self.p.tau)
        self.v += (self.target[0] - self.v) * a
        self.u2 += (self.target[1] - self.u2) * a
        self.w += (self.target[2] - self.w) * a
        self.energy += self.load * dt
        c, s = math.cos(self.th), math.sin(self.th)
        if self.w > 0:                       # EllipticalBot2D.get_CoR / move_bot
            cx, cy = self.x - self.p.d_cor[1] * s, self.y + self.p.d_cor[1] * c
        else:
            cx, cy = self.x + self.p.d_cor[0] * s, self.y - self.p.d_cor[0] * c
        dth = self.w * dt
        cr, sr = math.cos(dth), math.sin(dth)
        rx, ry = self.x - cx, self.y - cy
        nz = self._noise[self._k]
        self._k = (self._k + 1) % len(self._noise)
        if self._k == 0:
            self._noise = self.rng.standard_normal((4096, 3))
        active = 1.0 if self.load > 0 else 0.0          # a resting bot does not diffuse
        st = active * math.sqrt(2 * self.p.D_t * dt)
        self.x = cx + cr * rx - sr * ry + (self.v * c - self.u2 * s) * dt + st * nz[0]
        self.y = cy + sr * rx + cr * ry + (self.v * s + self.u2 * c) * dt + st * nz[1]
        self.th = self.th + dth + active * math.sqrt(2 * self.p.D_r * dt) * nz[2]


class SimWorld:
    def __init__(self, bots: dict, seed=0, dt=0.002, pose_store: PoseStore | None = None, tracker_csv=None):
        """bots: {ir_id: (SimParams, (x0, y0, th0))}."""
        self.t = 0.0
        self.dt = dt
        self.rng = np.random.default_rng(seed)
        self.plants, self.fw, self.params = {}, {}, {}
        for bid, (p, pose) in bots.items():
            plant = SimPlant(p, pose, rng=np.random.default_rng(self.rng.integers(1 << 31)))
            self.plants[bid] = plant
            self.fw[bid] = SimFirmware(bid, p, plant)
            self.params[bid] = p
        self.events = []
        self._seq = 0
        self.poses = pose_store or PoseStore()
        self.csv = tracker_csv
        first = next(iter(self.params.values()))
        self.cam_dt = 1.0 / first.cam_fps
        self.cam_latency = first.cam_latency
        self._next_exp = 0.0
        self._frame = 0
        self._pending_frames = []
        self.rows = []
        self.keep_rows = tracker_csv is None
        self.occluded = set()                 # marker ids hidden from the camera (tests of the PC-side stop)

    def schedule(self, t, fn):
        self._seq += 1
        heapq.heappush(self.events, (t, self._seq, fn))

    def transmit(self, raw, t_start):
        t_dec = t_start + NEC_FRAME_S
        for bid, fw in self.fw.items():
            p = self.params[bid]
            if self.rng.random() < p.ir_loss:
                continue
            self.schedule(t_dec + p.decode_delay, lambda t, fw=fw: fw.on_frame(raw, t))

    def _camera(self, t):
        while t >= self._next_exp:
            te = self._next_exp
            self._next_exp += self.cam_dt
            dets = []
            for bid, plant in self.plants.items():
                p = self.params[bid]
                g = p.geometry
                if g.marker_id in self.occluded or self.rng.random() < p.p_drop:
                    continue
                mx, my, mth = g.marker_from_body(plant.x, plant.y, plant.th)
                blue, red = self.fw[bid].led(te)
                dets.append(Detection(
                    frame=self._frame, t=te + self.cam_latency, id=g.marker_id,
                    x=float(mx) + self.rng.normal(0, p.pos_noise), y=float(my) + self.rng.normal(0, p.pos_noise),
                    theta=float(wrap(float(mth) + self.rng.normal(0, p.th_noise))),
                    led_b=p.led_base[0] + (p.led_flash if blue else 0.0) + self.rng.normal(0, 3.0),
                    led_r=p.led_base[1] + (p.led_flash if red else 0.0) + self.rng.normal(0, 3.0),
                    side_m=0.02, n_ref=4))
            self._pending_frames.append((te + self.cam_latency, self._frame, dets))
            self._frame += 1
        while self._pending_frames and self._pending_frames[0][0] <= t:
            t_emit, _, dets = self._pending_frames.pop(0)
            for d in dets:
                self.poses.put(d)
                if self.csv is not None:
                    self.csv.write(d)
                elif self.keep_rows:
                    self.rows.append(d)
            self.poses.frame_done(t_emit)

    def advance(self, t_target):
        while self.t < t_target - 1e-12:
            t_next = min(self.t + self.dt, t_target)
            while self.events and self.events[0][0] <= t_next:
                te, _, fn = heapq.heappop(self.events)
                fn(te)
            h = t_next - self.t
            for bid, plant in self.plants.items():
                plant.step(t_next, h)
                self.fw[bid].tick(t_next)
            self.t = t_next
            self._camera(self.t)

    def truth_twist_at_reference(self, bot_id, L, R):
        """Steady-state twist of the bot's reference point (the CoM in this simulator)."""
        return self.plants[bot_id].truth_twist_com(L, R)

    def truth_twist_at_marker(self, bot_id, L, R):
        u1, u2, w = self.truth_twist_at_reference(bot_id, L, R)
        return transfer_twist(u1, u2, w, self.params[bot_id].geometry.marker_offset)


class SimBridge:
    """SerialBridge API on the simulated IR channel."""

    def __init__(self, world: SimWorld, period_ms=110, usb_latency=0.002):
        self.world = world
        self.period_s = period_ms / 1000.0
        self.usb = usb_latency
        self._last_start = -1e9
        self.info = "I sim_blaster proto=1"

    def set_period(self, period_ms):
        self.period_s = period_ms / 1000.0

    def send(self, raw: int, check=True) -> SendRecord:
        if check and P.check(raw) != P.OK:
            raise ValueError(f"invalid frame {raw:08X}")
        w = self.world
        t_req = w.t
        t_start = max(t_req + self.usb / 2, self._last_start + self.period_s)
        w.advance(t_start)
        self._last_start = t_start
        w.transmit(raw, t_start)
        w.advance(t_start + NEC_FRAME_S + self.usb / 2)
        return SendRecord(t_req, w.t, raw, int(t_start * 1000), int((t_start + NEC_FRAME_S) * 1000))

    def close(self):
        pass
