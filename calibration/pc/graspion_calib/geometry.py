"""Frames, bot geometry and arena geometry shared by the tracker, the scheduler and the analysis.

World frame: right-handed, z towards the camera, x and y set by the reference-marker coordinates; heading
theta is counter-clockwise from +x, so omega > 0 is a CCW (left) turn. Body frame: x forward (front module),
y to the bot's left. A body twist (u1, u2, omega) of a point P means dP/dt = Rot(theta) [u1, u2] and
dtheta/dt = omega.
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass, field
from pathlib import Path

import numpy as np


def wrap(a):
    """Wrap angles to [-pi, pi)."""
    out = np.mod(np.asarray(a, float) + np.pi, 2 * np.pi) - np.pi
    return float(out) if np.ndim(a) == 0 else out


def transfer_twist(u1, u2, omega, r):
    """Twist of the point P + r (r in the body frame) from the twist of P: u' = u + omega z x r."""
    return u1 - omega * r[1], u2 + omega * r[0], omega


def icr_body(u1, u2, omega, eps=1e-3):
    """Instantaneous centre of rotation in the body frame, relative to the tracked point: (-u2/w, u1/w)."""
    if abs(omega) < eps:
        return np.nan, np.nan
    return -u2 / omega, u1 / omega


@dataclass
class BotGeometry:
    """Where the ArUco marker sits on the bot, measured per bot (FEASIBILITY.md 3.3, marker placement).

    marker_offset: marker centre in the body frame relative to the body reference point [m].
    marker_yaw:    marker heading minus the body +x (forward) direction [rad]. The marker heading is the tracker's
                   theta: the marker's 'up' direction, bottom edge -> top edge of the printed marker (corner 0 -> 1,
                   the top edge, rotated +90 deg). A marker glued with its top edge towards the bot's front has
                   marker_yaw = 0.
    The body reference point is whatever point you define (e.g. the ellipse centre); report it with the map.
    """
    marker_id: int
    marker_offset: tuple = (0.0, 0.0)
    marker_yaw: float = 0.0
    reference_point: str = "marker centre"

    def body_from_marker(self, x, y, theta_m):
        x, y, theta_m = (np.asarray(v, float) for v in (x, y, theta_m))
        th = wrap(theta_m - self.marker_yaw)
        c, s = np.cos(th), np.sin(th)
        dx, dy = self.marker_offset
        return x - (c * dx - s * dy), y - (s * dx + c * dy), th

    def marker_from_body(self, x, y, th):
        x, y, th = (np.asarray(v, float) for v in (x, y, th))
        c, s = np.cos(th), np.sin(th)
        dx, dy = self.marker_offset
        return x + c * dx - s * dy, y + s * dx + c * dy, wrap(th + self.marker_yaw)


@dataclass
class Arena:
    """Axis-aligned arena in world metres. The geofence is the arena shrunk by `margin` on every side."""
    xmin: float = 0.0
    xmax: float = 1.0
    ymin: float = 0.0
    ymax: float = 1.0
    margin: float = 0.08
    home: tuple | None = None          # re-centring target; default: arena centre

    @property
    def centre(self):
        return (0.5 * (self.xmin + self.xmax), 0.5 * (self.ymin + self.ymax)) if self.home is None else tuple(self.home)

    def inside(self, x, y, margin=None):
        m = self.margin if margin is None else margin
        return (self.xmin + m <= x <= self.xmax - m) and (self.ymin + m <= y <= self.ymax - m)


def load_arena_markers(path):
    """Reference markers from graspion_tracking's arena.json ({"markers": [{"id", "center", "side", "z"}]},
    metres, right-handed, z up) -> ({id: (x, y)}, printed side). All markers must lie in one plane (equal z) and
    have one printed side."""
    with open(path) as f:
        d = json.load(f)
    ms = d["markers"]
    zs = {round(float(m.get("z", 0.0)), 6) for m in ms}
    sides = {round(float(m["side"]), 6) for m in ms}
    if len(zs) != 1:
        raise ValueError(f"{path}: reference markers at several heights {sorted(zs)}; the tracker needs one plane")
    if len(sides) != 1:
        raise ValueError(f"{path}: reference markers of several sizes {sorted(sides)}")
    return {int(m["id"]): (float(m["center"][0]), float(m["center"][1])) for m in ms}, sides.pop()


@dataclass
class RigConfig:
    """Everything a session needs to know about the rig. Stored as JSON (configs/rig_example.json)."""
    arena: Arena = field(default_factory=Arena)
    reference_markers: dict = field(default_factory=dict)   # id -> (x, y) metres, all in one plane
    camera: dict = field(default_factory=dict)               # index, width, height, fps, exposure, intrinsics
    bots: dict = field(default_factory=dict)                 # bot IR id (str) -> BotGeometry fields
    bridge: dict = field(default_factory=dict)               # port, baud, period_ms

    @classmethod
    def load(cls, path):
        """Relative file paths in the config (reference_markers_file, camera.intrinsics) are taken from the
        config file's folder."""
        base = Path(path).resolve().parent
        with open(path) as f:
            d = json.load(f)
        arena = Arena(**d.get("arena", {}))
        camera = dict(d.get("camera", {}))
        if camera.get("intrinsics"):
            camera["intrinsics"] = str(base / camera["intrinsics"])
        refs = {int(k): tuple(v) for k, v in d.get("reference_markers", {}).items()}
        if d.get("reference_markers_file"):
            if refs:
                raise ValueError("give reference_markers or reference_markers_file, not both")
            refs, side = load_arena_markers(base / d["reference_markers_file"])
            camera.setdefault("reference_size_m", side)
        return cls(arena=arena, reference_markers=refs, camera=camera, bots=d.get("bots", {}),
                   bridge=d.get("bridge", {}))

    def geometry(self, bot_id: int) -> BotGeometry:
        b = self.bots.get(str(bot_id))
        if b is None:
            raise KeyError(f"bot {bot_id} is not in the rig config 'bots' table")
        return BotGeometry(marker_id=int(b["marker_id"]), marker_offset=tuple(b.get("marker_offset", (0.0, 0.0))),
                           marker_yaw=float(b.get("marker_yaw", 0.0)),
                           reference_point=b.get("reference_point", "marker centre"))

    def to_dict(self):
        return {"arena": asdict(self.arena), "reference_markers": {str(k): list(v) for k, v in self.reference_markers.items()},
                "camera": self.camera, "bots": self.bots, "bridge": self.bridge}
