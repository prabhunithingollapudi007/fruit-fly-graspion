"""Synthetic GRASPion rig for offline tests and dry runs (`python -m graspion_calib run --sim`).

Pieces, each mirroring the real one closely enough to exercise the pipeline end to end:
  SimFirmware  frame handling of firmware/calib_bot/calib_bot.ino (CRC, address filter, ops, command watchdog,
               sync flash)
  SimPlant     EllipticalBot2D kinematics (tilemat_control_stack Ellipse2dObj.py: rotation about a CoR offset
               laterally from the CoM by d_cw / d_ccw, plus translation along the heading), driven by a
               dead-zone power-law truth map with dead time, first-order lag and heading diffusion
  SimWorld     physics at 2 ms, the shared IR channel (frames of 50-86 ms, payload-dependent: bridge.nec_frame_s;
               handled decode_delay = RECORD_GAP_S + one loop() pass after the frame ends; random loss; frames
               closer than RECORD_GAP_S are both lost) and a camera with latency, noise, dropouts and the sync-LED
               brightness
  SimBridge    the SerialBridge API on top of SimWorld (period limits of ir_blaster.ino)

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
from .bridge import PERIOD_MAX_MS, PERIOD_MIN_MS, RECORD_GAP_S, BridgeError, SendRecord, nec_frame_s
from .forward_map import PowerLawMap
from .geometry import BotGeometry, wrap
from .poses import Detection, PoseStore


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
    decode_delay: float = RECORD_GAP_S + 0.0005   # frame end -> handleFrame: IRremote's end-of-frame silence + loop()
    usb_latency: float = 0.002
    watchdog_s: float = 0.3
    drive_max: int = 120
    led_base: tuple = (30.0, 25.0)           # (blue, red) background at the LED spot
    led_flash: float = 120.0
    flash_s: float = 0.07


class SimFirmware:
    """Frame semantics of calib_bot.ino. Times in host seconds.

    Only frames that command the drive (STOP, SEG, DRIVE inside a segment, RECENTRE) feed the command watchdog;
    a DRIVE without an open segment stops the motors; PING only flashes.
    """

    def __init__(self, bot_id, p: SimParams, plant):
        self.id, self.p, self.plant = bot_id, p, plant
        self.L = self.R = 0
        self.seg_active = False
        self.last_frame = -1e9
        self.flash_until = -1.0
        self.stats = {"frames_ok": 0, "crc_err": 0, "other_addr": 0, "drive_no_seg": 0, "wd_trips": 0}

    def _set(self, L, R, t):
        L = max(-self.p.drive_max, min(self.p.drive_max, L))
        R = max(-self.p.drive_max, min(self.p.drive_max, R))
        self.L, self.R = L, R
        self.plant.command(t + self.p.dead, L, R)

    def _flash(self, t):
        self.flash_until = t + self.p.flash_s

    def on_frame(self, raw, t):
        if P.check(raw) != P.OK:
            self.stats["crc_err"] += 1
            return
        f = P.unpack(raw)
        if f.bot_id == P.ADDR_BROADCAST:
            if f.op == P.OP_STOP:
                self.seg_active = False
                self._set(0, 0, t)
            return
        if f.bot_id != self.id:
            self.stats["other_addr"] += 1
            return
        self.stats["frames_ok"] += 1
        if f.op == P.OP_STOP:
            self.last_frame = t
            self.seg_active = False
            self._set(0, 0, t)
        elif f.op == P.OP_SEG:
            self.last_frame = t
            self._set(f.L, f.R, t)
            self.seg_active = True
            self._flash(t)
        elif f.op == P.OP_DRIVE:
            if not self.seg_active:              # SEG missed or segment ended: stop, never unlogged motion
                self.stats["drive_no_seg"] += 1
                self._set(0, 0, t)
                return
            self.last_frame = t
            self._set(f.L, f.R, t)
        elif f.op == P.OP_RECENTRE:
            self.last_frame = t
            self.seg_active = False
            self._set(f.L, f.R, t)
        elif f.op == P.OP_PING:                  # no watchdog feed
            self._flash(t)

    def tick(self, t):
        if (self.L != 0 or self.R != 0) and t - self.last_frame > self.p.watchdog_s:
            self.seg_active = False
            self._set(0, 0, t)
            self.stats["wd_trips"] += 1

    def led(self, t):
        """True while the blue sync flash is on."""
        return t < self.flash_until


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
        self.load = 0.0
        fL, fR = p.truth.activations(p.drive_max, p.drive_max)
        self._fmax = float(fL + fR)
        self._noise = self.rng.standard_normal((4096, 3))
        self._k = 0

    def command(self, t_eff, L, R):
        self._seq += 1
        heapq.heappush(self.pending, (t_eff, self._seq, L, R))

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
        self._last_tx = None                  # latest frame on the IR channel: {"t0", "t1", "lost"}
        self.collisions = 0                   # frames lost because another frame came within RECORD_GAP_S

    def schedule(self, t, fn):
        self._seq += 1
        heapq.heappush(self.events, (t, self._seq, fn))

    def transmit(self, raw, t_start):
        """One frame on the shared IR channel, decoded decode_delay after its last mark (bridge.nec_frame_s).
        IRremote ends a received frame only after RECORD_GAP_S of silence, so two frames that overlap or leave a
        shorter gap merge into one record that overflows the receive buffer: both are lost, for every bot. Call it
        no later than t_start (SimBridge does), so the earlier frame's decode is still pending."""
        t_end = t_start + nec_frame_s(raw)
        tx = {"t0": t_start, "t1": t_end, "lost": False}
        prev = self._last_tx
        if prev is not None and t_start < prev["t1"] + RECORD_GAP_S and prev["t0"] < t_end + RECORD_GAP_S:
            self.collisions += (not prev["lost"]) + 1
            tx["lost"] = prev["lost"] = True
        if prev is None or t_end >= prev["t1"]:
            self._last_tx = tx
        for bid, fw in self.fw.items():
            p = self.params[bid]
            if self.rng.random() < p.ir_loss:
                continue
            self.schedule(t_end + p.decode_delay,
                          lambda t, fw=fw, tx=tx: None if tx["lost"] else fw.on_frame(raw, t))

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
                blue = self.fw[bid].led(te)
                dets.append(Detection(
                    frame=self._frame, t=te + self.cam_latency, id=g.marker_id,
                    x=float(mx) + self.rng.normal(0, p.pos_noise), y=float(my) + self.rng.normal(0, p.pos_noise),
                    theta=float(wrap(float(mth) + self.rng.normal(0, p.th_noise))),
                    led_b=p.led_base[0] + (p.led_flash if blue else 0.0) + self.rng.normal(0, 3.0),
                    led_r=p.led_base[1] + self.rng.normal(0, 3.0),
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


class SimBridge:
    """SerialBridge API on the simulated IR channel. Like ir_blaster.ino it spaces frame starts by the period and
    replies once the frame has left the LED (t_ack, blaster_t1_ms = frame end)."""

    def __init__(self, world: SimWorld, period_ms=110, usb_latency=0.002):
        self.world = world
        self.usb = usb_latency
        self._last_start = -1e9
        self.info = f"I sim_blaster proto={P.PROTO_VERSION}"
        self.set_period(period_ms)

    def set_period(self, period_ms):
        period_ms = int(period_ms)
        if not PERIOD_MIN_MS <= period_ms <= PERIOD_MAX_MS:     # ir_blaster.ino answers "E range"
            raise BridgeError(f"blaster period {period_ms} ms outside {PERIOD_MIN_MS}..{PERIOD_MAX_MS} ms")
        self.period_s = period_ms / 1000.0

    def send(self, raw: int, check=True) -> SendRecord:
        if check and P.check(raw) != P.OK:
            raise ValueError(f"invalid frame {raw:08X}")
        w = self.world
        t_req = w.t
        t_start = max(t_req + self.usb / 2, self._last_start + self.period_s)
        t_end = t_start + nec_frame_s(raw)
        w.advance(t_start)
        self._last_start = t_start
        w.transmit(raw, t_start)
        w.advance(t_end + self.usb / 2)
        return SendRecord(t_req, w.t, raw, int(t_start * 1000), int(t_end * 1000))

    def close(self):
        pass
