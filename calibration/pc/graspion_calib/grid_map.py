"""Grid lookup map: (L, R) -> body twist (u1, u2, omega) from the measured grid-cell means, bilinear between the
grid's drive levels, and its inverse (v, omega) -> (L, R) by search over every drive pair.

Why: on the 20 held-out drives of 2026-11-12 (udZn9) the lookup predicted 0.48 cm/s and 0.33 rad/s rms, the fitted
power law (forward_map.py) 0.68 cm/s and 0.51 rad/s: speed plateaus and turn-rate peaks that the formula smooths away
are kept. Between drive 0 and the first measured level (55) the line is only a bridge across the dead zone, so the
inverse never commands a drive in 1..54.

The interfaces copy the two maps it stands in for:
  predict(L, R) -> (n, 3) array (u1, u2, omega)        like forward_map.PowerLawMap (live_view, analysis)
  twist_to_pwm(v, omega) -> (pwm_l, pwm_r, enable)      like graspion_sim.command_map.CommandMap (hw_fig8)
  pwm_to_twist(pwm_l, pwm_r, enable) -> (v, omega)
PWM byte = 128 + drive, what ir_drive_rx writes to analogWrite and what the calibration drove; both motors at 0 is
"stop" (127, 127, enable off). Reverse drives act like forward ones on this robot (polarity check), so |drive| is used.
Pure numpy.
"""
from __future__ import annotations

import csv
import json

import numpy as np

LEVELS = (0, 55, 60, 70, 80, 90, 100, 110, 120)
OUTPUTS = ("u1", "u2", "omega")
NEUTRAL = 127


class GridMap:
    def __init__(self, levels, table, counts=None, meta=None, scale=(0.005, 0.1)):
        """table: (len(levels), len(levels), 3) cell means of (u1, u2, omega), rows L, columns R. scale: the speed and
        turn-rate errors the inverse weighs as equal (m/s, rad/s)."""
        self.levels = np.asarray(levels, float)
        self.table = np.asarray(table, float)
        self.counts = None if counts is None else np.asarray(counts, int)
        self.meta = dict(meta or {})
        self.scale = tuple(scale)
        self.drives = np.r_[0, np.arange(int(self.levels[1]), int(self.levels[-1]) + 1)]   # what the inverse may send
        dl, dr = np.meshgrid(self.drives, self.drives, indexing="ij")
        self._cand = np.c_[dl.ravel(), dr.ravel()]
        self._cand_tw = self.predict(self._cand[:, 0], self._cand[:, 1])

    # ------------------------------------------------------------------ building
    @classmethod
    def from_segments(cls, paths, levels=LEVELS, **kw):
        """Cell means from analyze's segments_fit.csv (one or more files); reverse drives are left out. A cell with no
        measurement takes the mean of its measured neighbours; (0, 0) is zero (both motors off)."""
        lv = [int(v) for v in levels]
        acc = {}
        for p in [paths] if isinstance(paths, str) else paths:
            with open(p, newline="") as f:
                for r in csv.DictReader(f):
                    L, R = int(float(r["L"])), int(float(r["R"]))
                    if r.get("u1") in ("", None) or L not in lv or R not in lv:
                        continue
                    acc.setdefault((L, R), []).append([float(r[k]) for k in OUTPUTS])
        n = len(lv)
        tab, cnt = np.full((n, n, 3), np.nan), np.zeros((n, n), int)
        for (L, R), v in acc.items():
            tab[lv.index(L), lv.index(R)] = np.mean(v, axis=0)
            cnt[lv.index(L), lv.index(R)] = len(v)
        tab[0, 0] = 0.0
        filled = []
        for _ in range(n):                                   # fill gaps from measured neighbours, nearest first
            gaps = [(i, j) for i in range(n) for j in range(n) if np.isnan(tab[i, j, 0])]
            if not gaps:
                break
            for i, j in gaps:
                nb = [tab[a, b] for a, b in ((i - 1, j), (i + 1, j), (i, j - 1), (i, j + 1))
                      if 0 <= a < n and 0 <= b < n and not np.isnan(tab[a, b, 0])]
                if nb:
                    tab[i, j] = np.mean(nb, axis=0)
                    filled.append([lv[i], lv[j]])
        meta = {"source": [str(p) for p in ([paths] if isinstance(paths, str) else paths)],
                "segments": int(cnt.sum()), "filled_cells": filled,
                "note": "u1, u2: m/s, omega: rad/s, body twist of the tracked tag point; rows L, columns R"}
        return cls(lv, tab, cnt, meta, **kw)

    # ------------------------------------------------------------------ forward
    def predict(self, L, R):
        """(n, 3) twists (u1, u2, omega) at drives L, R (scalars or arrays), bilinear between the levels."""
        L = np.clip(np.abs(np.atleast_1d(np.asarray(L, float))), 0, self.levels[-1])
        R = np.clip(np.abs(np.atleast_1d(np.asarray(R, float))), 0, self.levels[-1])
        lv = self.levels
        i = np.clip(np.searchsorted(lv, L, side="right") - 1, 0, len(lv) - 2)
        j = np.clip(np.searchsorted(lv, R, side="right") - 1, 0, len(lv) - 2)
        s = ((L - lv[i]) / (lv[i + 1] - lv[i]))[:, None]
        t = ((R - lv[j]) / (lv[j + 1] - lv[j]))[:, None]
        T = self.table
        return ((1 - s) * (1 - t) * T[i, j] + s * (1 - t) * T[i + 1, j] + (1 - s) * t * T[i, j + 1]
                + s * t * T[i + 1, j + 1])

    # ------------------------------------------------------------------ inverse
    def drive_for(self, v, omega):
        """The drive pair whose predicted (u1, omega) is nearest to (v, omega), weighted by `scale`; (0, 0) is a
        candidate (stop). Returns (L, R, predicted u1, predicted omega)."""
        sv, sw = self.scale
        tw = self._cand_tw
        k = int(np.argmin(((tw[:, 0] - v) / sv) ** 2 + ((tw[:, 2] - omega) / sw) ** 2))
        L, R = self._cand[k]
        return int(L), int(R), float(tw[k, 0]), float(tw[k, 2])

    def twist_to_pwm(self, v, omega):
        """(PWM left, PWM right, enable) for cmd_vel (v, omega); stop is (127, 127, False)."""
        v, omega = float(v), float(omega)
        if not (np.isfinite(v) and np.isfinite(omega)):
            return NEUTRAL, NEUTRAL, False
        L, R, _, _ = self.drive_for(v, omega)
        if L == 0 and R == 0:
            return NEUTRAL, NEUTRAL, False
        return 128 + L, 128 + R, True

    def pwm_to_twist(self, pwm_left, pwm_right, enable=True):
        """(v, omega) the map expects at a PWM command (bytes 0..255, 127/128 = no drive)."""
        if not enable:
            return 0.0, 0.0
        L, R = (max(0, abs(int(p) - 128)) if int(p) != NEUTRAL else 0 for p in (pwm_left, pwm_right))
        u1, _, w = self.predict(L, R)[0]
        return float(u1), float(w)

    # ------------------------------------------------------------------ files
    def to_dict(self):
        return {"model": "grid_lookup", "levels": self.levels.tolist(), "outputs": list(OUTPUTS),
                "table": self.table.tolist(), "counts": None if self.counts is None else self.counts.tolist(),
                "scale": list(self.scale), "pwm": "128 + drive", "meta": self.meta}

    def save(self, path):
        with open(path, "w") as f:
            json.dump(self.to_dict(), f, indent=1)

    @classmethod
    def load(cls, path, **kw):
        with open(path) as f:
            d = json.load(f)
        if d.get("model") != "grid_lookup":
            raise ValueError(f"{path} is not a grid_lookup map (model {d.get('model')!r})")
        return cls(d["levels"], d["table"], d.get("counts"), d.get("meta"), **({"scale": d["scale"]} | kw))


def load_map(path):
    """A forward_map.json (power law) or a grid_map.json (grid lookup), whichever the file is."""
    with open(path) as f:
        model = json.load(f).get("model")
    if model == "grid_lookup":
        return GridMap.load(path)
    from .forward_map import PowerLawMap
    return PowerLawMap.load(path)
