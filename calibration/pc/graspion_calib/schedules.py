"""Calibration schedules (FEASIBILITY.md 5.2 steps 3-7). A schedule is a list of SegmentSpec, saved as JSON.

Drive values are on the signed drive scale of the firmware (-120..120, 0 = motor off).
"""
from __future__ import annotations

import json
from dataclasses import asdict, dataclass

import numpy as np

GRID = (0, 55, 60, 70, 80, 90, 100, 110, 120)
REVERSE = ((-90, -90), (-90, 90), (90, -90), (-60, -60), (-120, -120), (-90, 0), (0, -90), (-60, -80))


@dataclass
class SegmentSpec:
    kind: str                       # drive | steps | prbs | ping | static
    levels: list                    # [[t_offset_s, L, R], ...], first offset 0
    duration_s: float
    rest_s: float = 0.8
    block: int = 0
    tag: str = ""
    recentre: bool = True

    @property
    def L(self):
        return self.levels[0][1] if self.levels else 0

    @property
    def R(self):
        return self.levels[0][2] if self.levels else 0

    def level_at(self, t_off):
        cur = self.levels[0]
        for lv in self.levels:
            if lv[0] <= t_off:
                cur = lv
        return int(cur[1]), int(cur[2])


def drive(L, R, duration_s=3.0, rest_s=0.8, block=0, tag="grid"):
    return SegmentSpec("drive", [[0.0, int(L), int(R)]], duration_s, rest_s, block, tag)


def static_grid(levels=GRID, reverse=REVERSE, blocks=5, drive_s=3.0, rest_s=0.8, seed=0):
    """Signed grid: every (L, R) in levels x levels except (0, 0), plus reverse-polarity checks; each block holds
    every cell once in a new random order (randomised blocks, so drift with battery and leg wear averages out)."""
    rng = np.random.default_rng(seed)
    cells = [(a, b) for a in levels for b in levels if (a, b) != (0, 0)] + list(reverse)
    out = []
    for blk in range(blocks):
        for k in rng.permutation(len(cells)):
            L, R = cells[k]
            out.append(drive(L, R, drive_s, rest_s, blk, "grid" if (L >= 0 and R >= 0) else "reverse"))
    return out


def steps(pairs, hold_s=1.5, rest_s=0.8, block=0):
    """Two-level step segments: start at A (a step from rest), switch to B after hold_s; both orders."""
    out = []
    for a, b in pairs:
        for x, y in ((a, b), (b, a)):
            out.append(SegmentSpec("steps", [[0.0, *x], [hold_s, *y]], 2 * hold_s, rest_s, block, "step"))
    return out


def prbs(levels, duration_s=3.0, switch_min=0.1, switch_max=0.5, n_segments=10, rest_s=0.8, seed=0):
    """Random switching between drive levels (pairs), dwell U(switch_min, switch_max); one chunk per segment."""
    rng = np.random.default_rng(seed)
    out = []
    for i in range(n_segments):
        t, seq, last = 0.0, [], None
        while t < duration_s - 1e-9:
            choices = [k for k in range(len(levels)) if k != last] or [0]
            last = int(rng.choice(choices))
            seq.append([round(t, 3), *levels[last]])
            t += rng.uniform(switch_min, switch_max)
        out.append(SegmentSpec("prbs", seq, duration_s, rest_s, i, "prbs"))
    return out


def circles(pairs, duration_s=60.0, rest_s=2.0):
    """Long steady runs (CW and CCW circles of R ~ 5-15 cm) for heading diffusion and speed jitter."""
    return [SegmentSpec("drive", [[0.0, int(L), int(R)]], duration_s, rest_s, 0, "circle") for L, R in pairs]


def pings(duration_s=30.0):
    """Link test: PING frames at the blaster period; the bot flashes, no motion. Measures loss and latency."""
    return [SegmentSpec("ping", [[0.0, 0, 0]], duration_s, 0.5, 0, "ping", recentre=False)]


def static(duration_s=60.0):
    """Tracker-only window with the bot at rest: measurement-noise floor."""
    return [SegmentSpec("static", [[0.0, 0, 0]], duration_s, 0.0, 0, "static", recentre=False)]


def validation(targets, inverse_map, duration_s=3.0, rest_s=0.8, reps=2, seed=0):
    """Command (v, omega) targets through the fitted inverse map; feasible targets only, randomised order."""
    rng = np.random.default_rng(seed)
    out = []
    for r in range(reps):
        for k in rng.permutation(len(targets)):
            v, w = targets[k]
            L, R, ok = inverse_map.drive_for(v, w)
            if ok:
                s = drive(L, R, duration_s, rest_s, r, f"validate v={v:.4f} w={w:.3f}")
                out.append(s)
    return out


def save(specs, path):
    with open(path, "w") as f:
        json.dump([asdict(s) for s in specs], f, indent=1)


def load(path):
    with open(path) as f:
        return [SegmentSpec(**d) for d in json.load(f)]


def summary(specs, recentre_s=4.0):
    drive_s = sum(s.duration_s for s in specs)
    rest_s = sum(s.rest_s for s in specs)
    n_rc = sum(1 for s in specs if s.recentre)
    return {"segments": len(specs), "drive_min": drive_s / 60, "rest_min": rest_s / 60,
            "estimated_total_min": (drive_s + rest_s + n_rc * recentre_s) / 60}
