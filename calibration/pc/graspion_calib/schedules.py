"""Calibration schedules. A schedule is a list of SegmentSpec, saved as JSON together with how it was made
({"meta": {"kind", "seed", ...}, "segments": [...]}).

Drive values are on the signed drive scale of the firmware (-120..120, 0 = motor off). The grid takes an explicit
seed; the command line draws a fresh one per schedule (new_seed) and records it, so every bot and every battery
gets its own order and any schedule can be regenerated.
"""
from __future__ import annotations

import json
import secrets
from dataclasses import asdict, dataclass

import numpy as np

DRIVE_MAX = 120                 # the firmware's cap (HARDWARE.md 4.4); the frame itself carries up to protocol.DRIVE_LIMIT
GRID = (0, 55, 60, 70, 80, 90, 100, 110, 120)
REVERSE = ((-90, -90), (-90, 90), (90, -90), (-60, -60), (-120, -120), (-90, 0), (0, -90), (-60, -80))


@dataclass
class SegmentSpec:
    kind: str                       # drive | ping | static
    levels: list                    # [[t_offset_s, L, R]]; one level per segment
    duration_s: float
    rest_s: float = 0.8
    block: int = 0
    tag: str = ""
    pause: bool = True              # stop and wait for the operator before this segment (scheduler)

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
    cells_ = [(a, b) for a in levels for b in levels if (a, b) != (0, 0)] + list(reverse)
    out = []
    for blk in range(blocks):
        for k in rng.permutation(len(cells_)):
            L, R = cells_[k]
            out.append(drive(L, R, drive_s, rest_s, blk, "grid" if (L >= 0 and R >= 0) else "reverse"))
    return out


def cells(pairs, blocks=1, drive_s=3.0, rest_s=0.8):
    """Chosen (L, R) cells in the given order, every block once more: trial runs, re-runs of single cells and the
    few-shot subsets of a held-out bot. No shuffling: the operator sees what comes next."""
    return [drive(L, R, drive_s, rest_s, blk, "cells") for blk in range(blocks) for L, R in pairs]


def pings(duration_s=30.0):
    """Link test: PING frames at the blaster period; the bot flashes, no motion. Measures loss and latency."""
    return [SegmentSpec("ping", [[0.0, 0, 0]], duration_s, 0.5, 0, "ping", pause=False)]


def static(duration_s=60.0):
    """Tracker-only window with the bot at rest: measurement-noise floor."""
    return [SegmentSpec("static", [[0.0, 0, 0]], duration_s, 0.0, 0, "static", pause=False)]


def new_seed() -> int:
    """A fresh 32-bit seed, so the cell order is not repeated across bots and batteries (seed 0 every time would
    repeat the same order, and with it the same battery and warm-up state per cell, across the whole fleet)."""
    return secrets.randbits(32)


def save(specs, path, meta=None):
    with open(path, "w") as f:
        json.dump({"meta": meta or {}, "segments": [asdict(s) for s in specs]}, f, indent=1)


def load_with_meta(path):
    """(specs, meta). Older files hold a bare list of segments (meta {})."""
    with open(path) as f:
        d = json.load(f)
    if isinstance(d, list):
        d = {"meta": {}, "segments": d}
    return [SegmentSpec(**s) for s in d["segments"]], d.get("meta", {})


def load(path):
    return load_with_meta(path)[0]


def parse_selection(text, n):
    """Segment indices from "0-3,7,12-" (ranges inclusive, an open end runs to the last segment), in order, each
    once. ValueError on anything else or an index outside 0..n-1."""
    out = []
    for part in text.split(","):
        part = part.strip()
        if not part:
            continue
        a, dash, b = part.partition("-")
        lo = int(a)
        hi = (n - 1 if b == "" else int(b)) if dash else lo
        if not 0 <= lo <= hi < n:
            raise ValueError(f"segment selection {part!r} outside 0..{n - 1}")
        out.extend(k for k in range(lo, hi + 1) if k not in out)
    if not out:
        raise ValueError("empty segment selection")
    return out


def summary(specs, pause_s=8.0):
    """Segment count and a time estimate; pause_s is the operator's handling time per paused segment."""
    drive_s = sum(s.duration_s for s in specs)
    rest_s = sum(s.rest_s for s in specs)
    n_pause = sum(1 for s in specs if s.pause)
    return {"segments": len(specs), "drive_min": drive_s / 60, "rest_min": rest_s / 60,
            "estimated_total_min": (drive_s + rest_s + n_pause * pause_s) / 60}
