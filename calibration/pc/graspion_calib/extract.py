"""Load a session folder, align commands with the video, and cut it into per-segment pose windows.

Time alignment: every SEG and PING frame makes the bot flash its top LED at decode time. The delay from the
blaster acknowledgement (t_ack, host clock) to the flash onset in the tracker trace (host clock of the frame
read) is cam_offset = IR decode + camera latency (+ frame quantisation). A command acknowledged at t_ack
therefore shows up in the video at t_ack + cam_offset. Segments whose flash was not found use the median.
"""
from __future__ import annotations

import csv
import json
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

from .geometry import BotGeometry, Arena
from .twist import body_velocity, fit_twist


def read_csv(path):
    """CSV -> {column: np.ndarray (float where every value parses, else str)}."""
    with open(path, newline="") as f:
        rows = list(csv.DictReader(f))
    cols = {}
    if not rows:
        return cols
    for k in rows[0]:
        vals = [r[k] for r in rows]
        try:
            cols[k] = np.array([float(v) if v != "" else np.nan for v in vals])
        except ValueError:
            cols[k] = np.array(vals, dtype=object)
    return cols


@dataclass
class Session:
    path: Path
    meta: dict
    geometry: BotGeometry
    arena: Arena
    segments: dict
    frames: dict
    track: dict                 # tracker rows of this bot's marker only, sorted by t
    bot_id: int


def load_session(path, tracker_csv=None, geometry: BotGeometry | None = None) -> Session:
    path = Path(path)
    with open(path / "session.json") as f:
        meta = json.load(f)
    g = geometry or BotGeometry(**meta["geometry"])
    g.marker_offset = tuple(g.marker_offset)
    arena = Arena(**{k: (tuple(v) if isinstance(v, list) else v) for k, v in meta["arena"].items()})
    segs = read_csv(path / "segments.csv")
    frames = read_csv(path / "frames.csv")
    tr = read_csv(Path(tracker_csv) if tracker_csv else path / "tracker.csv")
    m = tr["id"] == g.marker_id
    order = np.argsort(tr["t"][m])
    track = {k: np.asarray(v)[m][order] for k, v in tr.items()}
    return Session(path, meta, g, arena, segs, frames, track, int(meta["bot_id"]))


# ------------------------------------------------------------------------------------------- flash / latency
def flash_onsets(t, led, rise=40.0):
    """Rising edges of the LED trace above median + rise. Returns onset times (first frame above threshold)."""
    t, led = np.asarray(t, float), np.asarray(led, float)
    if len(t) == 0:
        return np.array([])
    on = led > np.median(led) + rise
    idx = np.flatnonzero(on & ~np.r_[False, on[:-1]])
    return t[idx]


def match_flashes(t_acks, onsets, max_lag=0.5):
    """Lag of the first onset within (0, max_lag] after each acknowledgement (NaN if none)."""
    onsets = np.sort(np.asarray(onsets, float))
    lags = np.full(len(t_acks), np.nan)
    for i, ta in enumerate(t_acks):
        j = np.searchsorted(onsets, ta, side="right")
        if j < len(onsets) and onsets[j] - ta <= max_lag:
            lags[i] = onsets[j] - ta
    return lags


def frame_period(track):
    t = np.unique(np.asarray(track["t"], float))
    return float(np.median(np.diff(t))) if len(t) > 1 else 0.0


def command_offset(sess: Session, rise=40.0, max_lag=0.5, half_frame=True):
    """cam_offset statistics from all SEG and PING frames of the session.

    The first frame showing the flash was exposed on average half a frame period after the flash started
    (short exposure assumed), so half a period is subtracted from every lag when half_frame is set."""
    fr = sess.frames
    sel = np.isin(fr["op"], ["SEG", "PING"]) & (fr["bot"] == sess.bot_id)
    t_ack = fr["t_ack"][sel]
    corr = 0.5 * frame_period(sess.track) if half_frame else 0.0
    lags = match_flashes(t_ack, flash_onsets(sess.track["t"], sess.track["led_b"], rise), max_lag) - corr
    ok = np.isfinite(lags)
    med = float(np.median(lags[ok])) if ok.any() else float("nan")
    return {"cam_offset": med, "cam_offset_mad": float(np.median(np.abs(lags[ok] - med))) if ok.any() else float("nan"),
            "half_frame_correction": corr, "found": int(ok.sum()), "sent": int(len(lags)), "lags": lags,
            "t_ack": t_ack, "seg": fr["seg"][sel], "op": fr["op"][sel]}


def link_stats(sess: Session, rise=40.0, max_lag=0.5):
    """Ping test: fraction of PING frames that produced a flash, and the latency distribution."""
    fr = sess.frames
    sel = (fr["op"] == "PING") & (fr["bot"] == sess.bot_id)
    lags = match_flashes(fr["t_ack"][sel], flash_onsets(sess.track["t"], sess.track["led_b"], rise), max_lag)
    ok = np.isfinite(lags)
    t_ack = fr["t_ack"][sel]
    period = float(np.median(np.diff(t_ack))) if len(t_ack) > 1 else float("nan")
    q = np.percentile(lags[ok], [5, 50, 95]).tolist() if ok.any() else [float("nan")] * 3
    return {"pings": int(sel.sum()), "flashes": int(ok.sum()), "loss": float(1 - ok.mean()) if len(ok) else float("nan"),
            "frame_period_s": period, "frames_per_s": 1.0 / period if period == period else float("nan"),
            "offset_p5_p50_p95": q, "bridge_req_to_ack_s": float(np.median(fr["t_ack"][sel] - fr["t_req"][sel]))
            if sel.any() else float("nan")}


# ------------------------------------------------------------------------------------------- segments
@dataclass
class SegmentData:
    seg: int
    attempt: int
    kind: str
    tag: str
    block: int
    L: int
    R: int
    status: str
    flash: bool
    lag: float
    t_cmd: float                  # SEG effect time on the camera clock
    t_end: float                  # STOP effect time on the camera clock
    levels: list                  # [(t_effect, L, R), ...] incl. the final (t_end, 0, 0)
    t: np.ndarray = field(repr=False, default=None)
    x: np.ndarray = field(repr=False, default=None)
    y: np.ndarray = field(repr=False, default=None)
    th: np.ndarray = field(repr=False, default=None)
    extra: dict = field(default_factory=dict)


def _segment_levels(fr, seg, bot, offset, t_end_eff):
    sel = (fr["seg"] == seg) & (fr["bot"] == bot) & np.isin(fr["op"], ["SEG", "DRIVE"])
    out = []
    for t, L, R in zip(fr["t_ack"][sel], fr["L"][sel], fr["R"][sel]):
        if not out or (int(L), int(R)) != out[-1][1:]:
            out.append((float(t) + offset, int(L), int(R)))
    out.append((t_end_eff, 0, 0))
    return out


def extract_segments(sess: Session, pad_before=0.5, pad_after=1.0, rise=40.0):
    off = command_offset(sess, rise=rise)
    lag_by_ack = {round(float(t), 4): lag for t, op, lag in zip(off["t_ack"], off["op"], off["lags"]) if op == "SEG"}
    med = off["cam_offset"] if np.isfinite(off["cam_offset"]) else 0.08
    S, tr = sess.segments, sess.track
    g = sess.geometry
    bx, by, bth = g.body_from_marker(tr["x"], tr["y"], tr["theta"])
    fr = sess.frames
    motion = np.sort(fr["t_ack"][(fr["bot"] == sess.bot_id) & np.isin(fr["op"], ["SEG", "DRIVE", "RECENTRE"])])
    out = []
    for k in range(len(S["seg"])):
        kind = str(S["kind"][k])
        if kind in ("ping",):
            continue
        seg = int(S["seg"][k])
        t_ack, t_end_ack = float(S["t_seg_ack"][k]), float(S["t_end_ack"][k])
        own = lag_by_ack.get(round(t_ack, 4), np.nan)
        flash = bool(np.isfinite(own)) and kind != "static"
        lag = own if flash else med
        t_cmd, t_end = (t_ack, t_end_ack) if kind == "static" else (t_ack + lag, t_end_ack + lag)
        j = np.searchsorted(motion, t_end_ack + 1e-6)          # next motion command (re-centring, next segment)
        t_stop_window = min(t_end + pad_after, motion[j] + lag) if j < len(motion) else t_end + pad_after
        m = (tr["t"] >= t_cmd - pad_before) & (tr["t"] <= t_stop_window)
        levels = []
        if kind != "static":
            sel = (sess.frames["t_ack"] >= t_ack - 1e-6) & (sess.frames["t_ack"] <= t_end_ack + 1e-6)
            levels = _segment_levels({k2: v[sel] for k2, v in sess.frames.items()}, seg, sess.bot_id, lag, t_end)
        out.append(SegmentData(seg, int(S["attempt"][k]), kind, str(S["tag"][k]), int(S["block"][k]),
                               int(S["L"][k]), int(S["R"][k]), str(S["status"][k]), flash,
                               lag, t_cmd, t_end, levels, tr["t"][m], bx[m], by[m], bth[m]))
    return out, off


def detect_early_stop(sd: SegmentData, t_from, t_to, lever=0.02, frac=0.25, min_still_s=0.3):
    """If the bot stopped inside the window (e.g. its command watchdog tripped after lost frames), return the
    time it stopped; else None. 'Speed' = sqrt(u1^2 + u2^2 + (lever omega)^2) from a smoothed derivative."""
    m = (sd.t >= t_from) & (sd.t <= t_to)
    if m.sum() < 15:
        return None
    tv, u1, u2, w, _ = body_velocity(sd.t[m], sd.x[m], sd.y[m], sd.th[m], window_s=0.15)
    if len(tv) < 10:
        return None
    sp = np.sqrt(u1 ** 2 + u2 ** 2 + (lever * w) ** 2)
    ref = np.median(sp[: max(5, len(sp) // 2)])
    if ref < 1e-3:
        return None
    moving = sp > frac * ref
    last = np.flatnonzero(moving)
    if not len(last):
        return None
    t_last = tv[last[-1]]
    return float(t_last) if (t_to - t_last) > min_still_s else None


USABLE = ("ok", "geofence", "marker_loss")   # the last two end early at the PC-side STOP; "missed" never moved


def fit_segment_twists(segs, settle_s=0.5, min_duration_s=1.0, kinds=("drive",), statuses=USABLE):
    """Constant-twist fit for every single-level segment. Returns a list of row dicts."""
    rows = []
    for sd in segs:
        if sd.kind not in kinds or len(sd.levels) > 2 or sd.status not in statuses:
            continue
        t0, t1 = sd.t_cmd + settle_s, sd.t_end
        stop = detect_early_stop(sd, t0, t1)
        truncated = stop is not None
        if truncated:
            t1 = stop - 0.1
        m = (sd.t >= t0) & (sd.t <= t1)
        row = {"seg": sd.seg, "attempt": sd.attempt, "kind": sd.kind, "tag": sd.tag, "block": sd.block,
               "L": sd.L, "R": sd.R, "status": sd.status, "flash": int(sd.flash), "lag": sd.lag,
               "truncated": int(truncated)}
        if m.sum() < 10 or (t1 - t0) < min_duration_s:
            row.update({"fit": "too_short", "window_s": float(max(0.0, t1 - t0))})
            rows.append(row)
            continue
        try:
            f = fit_twist(sd.t[m], sd.x[m], sd.y[m], sd.th[m])
        except ValueError as e:
            row.update({"fit": f"error: {e}"})
            rows.append(row)
            continue
        row.update({k: f[k] for k in ("u1", "u2", "omega", "se_u1", "se_u2", "se_omega", "icr_x", "icr_y",
                                      "pos_rms", "th_rms", "n", "n_rejected", "duration")})
        row.update({"fit": "ok", "window_s": float(t1 - t0), **sd.extra})
        rows.append(row)
    return rows


def write_rows(rows, path):
    keys = []
    for r in rows:
        for k in r:
            if k not in keys:
                keys.append(k)
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: (f"{v:.6g}" if isinstance(v, float) else v) for k, v in r.items()})
