"""Load a session folder, align commands with the video, and cut it into per-segment pose windows.

Time alignment: every SEG and PING frame makes the bot flash its top LED at decode time. The delay from the
blaster acknowledgement (t_ack, host clock) to the flash onset in the tracker trace (host clock of the frame
read) is cam_offset = IR decode + camera latency (+ frame quantisation). A command acknowledged at t_ack
therefore shows up in the video at t_ack + cam_offset. Segments whose flash was not found use the median.
A session with no flash at all uses the offset measured in the other analysed sessions, else none (0, with a
warning: times are then relative to the acknowledgement and the fitted dead time includes the camera latency).
"""
from __future__ import annotations

import csv
import json
import warnings
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from numpy.lib.stride_tricks import sliding_window_view

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
def _rolling_low(t, v, win_s, q):
    """Low percentile q of v within +-win_s/2, evaluated on knots every win_s/4 and interpolated to t."""
    step = win_s / 4
    knots = np.arange(t[0], t[-1] + step, step)
    lo = np.searchsorted(t, knots - win_s / 2)
    hi = np.searchsorted(t, knots + win_s / 2)
    vals = np.array([np.percentile(v[a:b], q) if b - a >= 5 else np.nan for a, b in zip(lo, hi)])
    ok = np.isfinite(vals)
    if not ok.any():
        return np.full(len(t), np.percentile(v, q))
    return np.interp(t, knots[ok], vals[ok])


def flash_onsets(t, led, rise=40.0, base_win_s=2.0, base_q=10.0, min_gap_s=0.04):
    """Rising edges of the LED trace above a dark-level baseline + rise. Returns onset times (first frame above
    threshold).

    The baseline is a low percentile (base_q) of the trace within +-base_win_s/2: the dark level between
    flashes. A whole-session median would sit on the *lit* level in a ping session (the LED is on in ~2/3 of
    the frames at the 110 ms blaster period) and find no flash at all; the rolling window also follows slow
    background changes as the bot moves. Non-finite samples (LED patch outside the image) are dropped rather
    than taken as dark, and an onset within min_gap_s of the previous one (flicker inside one flash) is ignored.
    """
    t, led = np.asarray(t, float), np.asarray(led, float)
    ok = np.isfinite(t) & np.isfinite(led)
    t, led = t[ok], led[ok]
    if len(t) == 0:
        return np.array([])
    order = np.argsort(t, kind="stable")
    t, led = t[order], led[order]
    on = led > _rolling_low(t, led, base_win_s, base_q) + rise
    ts = t[np.flatnonzero(on & ~np.r_[False, on[:-1]])]
    keep = np.r_[True, np.diff(ts) >= min_gap_s] if len(ts) else np.zeros(0, bool)
    return ts[keep]


def _gate(ta, onsets, c, h, max_lag):
    """Per-ack gate (ta + max(c - h, 0), ta + min(c + h, max_lag)] as onset index ranges [j_lo, j_hi)."""
    lo = ta + max(c - h, 0.0)
    hi = ta + min(c + h, max_lag)
    return np.searchsorted(onsets, lo, side="right"), np.searchsorted(onsets, hi, side="right")


def match_flashes(t_acks, onsets, max_lag=0.5, offset=None, report=None):
    """Lag (onset - acknowledgement) of the flash each acknowledged frame caused; NaN if it caused none.

    One-to-one and causal: an onset can only follow its own acknowledgement, each onset is claimed at most once,
    and it must fall in a gate of width min(ack spacing, max_lag) centred on the session's typical lag. So in a
    ping train (frames every ~110 ms) a frame the bot never decoded stays NaN instead of inheriting the next
    frame's flash. The typical lag is `offset` if given, else the gate centre that matches the most frames
    (smallest such lag on ties), refined to the median matched lag.

    In an evenly spaced train, lag c and c +- one spacing match equally many frames when the first or last frame
    was lost, so the data cannot tell them apart (with a camera offset above the period, the smallest lag is the
    wrong one). report, if a dict, gets 'ambiguous' and 'rival_lags' (gate centres at least one gate away that
    match as many frames). Frames spaced more than max_lag apart match only the true lag: scheduler.run_ping
    starts every ping run with such anchor PINGs.
    """
    t_acks = np.asarray(t_acks, float)
    lags = np.full(len(t_acks), np.nan)
    onsets = np.sort(np.asarray(onsets, float))
    onsets = onsets[np.isfinite(onsets)]
    if report is not None:
        report.update(ambiguous=False, rival_lags=[])
    if len(t_acks) == 0 or len(onsets) == 0:
        return lags
    order = np.argsort(t_acks, kind="stable")
    ta = t_acks[order]
    sp = np.diff(ta)
    sp = sp[sp > 0]
    h = 0.5 * min(float(np.median(sp)) if len(sp) else np.inf, max_lag)

    def assign(c):
        j_lo, j_hi = _gate(ta, onsets, c, h, max_lag)
        out = np.full(len(ta), np.nan)
        claimed = np.zeros(len(onsets), bool)
        for i in range(len(ta)):
            j = j_lo[i]
            while j < j_hi[i] and claimed[j]:
                j += 1
            if j < j_hi[i]:
                claimed[j] = True
                out[i] = onsets[j] - ta[i]
        return out

    if offset is None:
        cands = []
        for a in ta:                                   # every onset within (0, max_lag] after an acknowledgement
            j0, j1 = np.searchsorted(onsets, [a, a + max_lag], side="right")
            cands.extend(onsets[j0:j1] - a)
        if not cands:
            return lags
        best_n, c, counts = -1, None, []
        cands = np.unique(np.round(cands, 4))
        for cand in cands:                             # ascending: the first maximum is the smallest lag
            j_lo, j_hi = _gate(ta, onsets, cand, h, max_lag)
            n = int(np.sum(j_hi > j_lo))
            counts.append(n)
            if n > best_n:
                best_n, c = n, float(cand)
        if report is not None:                         # another alignment (outside c's gate) that matches as many
            rivals = []
            for cand, n in zip(cands, counts):
                if n == best_n and abs(cand - c) > h and (not rivals or cand - rivals[-1] > h):
                    rivals.append(float(cand))
            report.update(ambiguous=bool(rivals), rival_lags=rivals)
        first = assign(c)
        if np.isfinite(first).any():
            c = float(np.nanmedian(first))
    else:
        c = float(offset)
    lags[order] = assign(c)
    return lags


def frame_period(track):
    t = np.unique(np.asarray(track["t"], float))
    return float(np.median(np.diff(t))) if len(t) > 1 else 0.0


def _ambiguity_warning(name, lag, rep):
    return (f"flash-to-frame lag in {name} is ambiguous: {lag:.3f} s and "
            f"{', '.join(f'{x:.3f}' for x in rep['rival_lags'])} s (one frame spacing apart) match equally many "
            "frames, as in a ping train without anchor PINGs whose first or last frame was lost. Its cam_offset is "
            "not used for other sessions")


def command_offset(sess: Session, rise=40.0, max_lag=0.5, half_frame=True):
    """cam_offset statistics from all SEG and PING frames of the session.

    The first frame showing the flash was exposed on average half a frame period after the flash started
    (short exposure assumed), so half a period is subtracted from every lag when half_frame is set.
    'ambiguous': the lag could be one frame spacing off (match_flashes); report.analyze then leaves this
    session's cam_offset out of the offset it lends to sessions without a flash."""
    fr = sess.frames
    sel = np.isin(fr["op"], ["SEG", "PING"]) & (fr["bot"] == sess.bot_id)
    t_ack = fr["t_ack"][sel]
    corr = 0.5 * frame_period(sess.track) if half_frame else 0.0
    rep = {}
    lags = match_flashes(t_ack, flash_onsets(sess.track["t"], sess.track["led_b"], rise), max_lag, report=rep) - corr
    ok = np.isfinite(lags)
    med = float(np.median(lags[ok])) if ok.any() else float("nan")
    out = {"cam_offset": med, "cam_offset_mad": float(np.median(np.abs(lags[ok] - med))) if ok.any() else float("nan"),
           "half_frame_correction": corr, "found": int(ok.sum()), "sent": int(len(lags)), "lags": lags,
           "t_ack": t_ack, "seg": fr["seg"][sel], "op": fr["op"][sel], "ambiguous": rep["ambiguous"]}
    if len(lags) and not ok.any():
        out["warning"] = (f"no sync flash found for any of {len(lags)} SEG/PING frames in {sess.path.name}: is "
                          "led_b flat in tracker.csv? (LED covered, LED patch misplaced: led_offset / led_height, "
                          "or a camera without usable colour, where the tracker must log brightness instead)")
    elif len(lags) >= 4 and ok.mean() < 0.5:
        out["warning"] = (f"sync flash found for only {int(ok.sum())} of {len(lags)} SEG/PING frames in "
                          f"{sess.path.name}: check led_b in tracker.csv")
    if rep["ambiguous"]:
        w = _ambiguity_warning(sess.path.name, med + corr, rep)
        out["warning"] = f"{out['warning']}; {w}" if out.get("warning") else w
    return out


def link_stats(sess: Session, rise=40.0, max_lag=0.5):
    """Ping test: fraction of PING frames that produced a flash, and the latency distribution (raw lags, without
    the half-frame correction). 'offset_ambiguous': the lags could be one PING spacing off (match_flashes)."""
    fr = sess.frames
    sel = (fr["op"] == "PING") & (fr["bot"] == sess.bot_id)
    rep = {}
    lags = match_flashes(fr["t_ack"][sel], flash_onsets(sess.track["t"], sess.track["led_b"], rise), max_lag,
                         report=rep)
    ok = np.isfinite(lags)
    t_ack = fr["t_ack"][sel]
    period = float(np.median(np.diff(t_ack))) if len(t_ack) > 1 else float("nan")
    q = np.percentile(lags[ok], [5, 50, 95]).tolist() if ok.any() else [float("nan")] * 3
    out = {"pings": int(sel.sum()), "flashes": int(ok.sum()), "loss": float(1 - ok.mean()) if len(ok) else float("nan"),
           "frame_period_s": period, "frames_per_s": 1.0 / period if period == period else float("nan"),
           "offset_p5_p50_p95": q, "offset_ambiguous": rep["ambiguous"],
           "bridge_req_to_ack_s": float(np.median(fr["t_ack"][sel] - fr["t_req"][sel])) if sel.any() else float("nan")}
    if rep["ambiguous"]:
        out["warning"] = _ambiguity_warning(sess.path.name, q[1], rep)
    return out


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
    stop: float | None = None     # visible early stop found by steady_window (camera clock)


def _segment_levels(fr, seg, bot, offset, t_end_eff):
    """[(t_effect, L, R), ...] from the SEG and DRIVE frames (one entry per level change), ending with
    (t_end_eff, 0, 0)."""
    sel = (fr["seg"] == seg) & (fr["bot"] == bot) & np.isin(fr["op"], ["SEG", "DRIVE"])
    out = []
    for t, L, R in zip(fr["t_ack"][sel], fr["L"][sel], fr["R"][sel]):
        if not out or (int(L), int(R)) != out[-1][1:]:
            out.append((float(t) + offset, int(L), int(R)))
    out.append((t_end_eff, 0, 0))
    return out


def extract_segments(sess: Session, pad_before=0.5, pad_after=1.0, rise=40.0, off=None, fallback_offset=None):
    """Per-segment windows. off: command_offset(sess) if already computed. fallback_offset: cam_offset to use
    when this session has no flash at all (e.g. the median of the other sessions on the same rig); without one
    the offset is 0 and off['cam_offset_source'] = 'none' (times relative to the acknowledgement)."""
    off = command_offset(sess, rise=rise) if off is None else off
    lag_by_ack = {round(float(t), 4): lag for t, op, lag in zip(off["t_ack"], off["op"], off["lags"]) if op == "SEG"}
    if np.isfinite(off["cam_offset"]):
        med, off["cam_offset_source"] = off["cam_offset"], "measured"
    elif fallback_offset is not None and np.isfinite(fallback_offset):
        med, off["cam_offset_source"] = float(fallback_offset), "other_sessions"
    else:
        med, off["cam_offset_source"] = 0.0, "none"
    off["cam_offset_used"] = med
    if off.get("warning"):
        warnings.warn(off["warning"] + f"; using cam_offset {med:.3f} s ({off['cam_offset_source']})", stacklevel=2)
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
            sel = (fr["t_ack"] >= t_ack - 1e-6) & (fr["t_ack"] <= t_end_ack + 1e-6)
            levels = _segment_levels({k2: v[sel] for k2, v in fr.items()}, seg, sess.bot_id, lag, t_end)
        out.append(SegmentData(seg, int(S["attempt"][k]), kind, str(S["tag"][k]), int(S["block"][k]),
                               int(S["L"][k]), int(S["R"][k]), str(S["status"][k]), flash,
                               lag, t_cmd, t_end, levels, tr["t"][m], bx[m], by[m], bth[m]))
    return out, off


def detect_early_stop(sd: SegmentData, t_from, t_to, lever=0.02, frac=0.25, min_still_s=0.3, ref_s=0.2):
    """If the bot stopped inside the window (e.g. its command watchdog tripped after lost frames), return the
    time it stopped; else None. 'Speed' = sqrt(u1^2 + u2^2 + (lever omega)^2) from a smoothed derivative.

    'Moving' is a speed above frac times the driving speed, taken as the highest median speed over ref_s
    (the median over the first half of the window missed most stops in that half). A stop needs evidence:
    still samples spanning more than min_still_s after the last moving one. The velocity series skips runs
    between dropped frames that are shorter than the filter, so a tail lost that way is not a stop. On 20
    simulated grid sessions (34 watchdog trips more than 0.45 s before the STOP) this saw 31 trips with no false
    stop."""
    m = (sd.t >= t_from) & (sd.t <= t_to)
    if m.sum() < 15:
        return None
    tv, u1, u2, w, _ = body_velocity(sd.t[m], sd.x[m], sd.y[m], sd.th[m], window_s=0.15)
    if len(tv) < 10:
        return None
    sp = np.sqrt(u1 ** 2 + u2 ** 2 + (lever * w) ** 2)
    dt = np.median(np.diff(tv))
    k = max(5, int(round(ref_s / dt)))
    ref = np.max(np.median(sliding_window_view(sp, k), axis=1)) if len(sp) > k else np.median(sp)
    if ref < 1e-3:
        return None
    last = np.flatnonzero(sp > frac * ref)
    if not len(last):
        return None
    t_last = tv[last[-1]]
    still = tv[last[-1] + 1:]                                  # all below frac * ref
    return float(t_last) if len(still) and (still[-1] - t_last) > min_still_s else None


USABLE = ("ok", "geofence", "marker_loss")   # the last two end early at the PC-side STOP; "missed" never moved


def steady_window(sd: SegmentData, settle_s=0.5, margin_s=0.1):
    """(t0, t1, truncated): the part of a segment where the commanded drive was still acting, on the camera clock.

    From t_cmd + settle_s to the STOP, cut short when the bot visibly stopped (detect_early_stop, e.g. its
    command watchdog tripped after lost frames; then t1 = stop - margin_s). The PC logs such a segment as 'ok',
    so the twist fit must use this window. The visible stop is stored in sd.stop."""
    t0, t1 = sd.t_cmd + settle_s, sd.t_end
    sd.stop = detect_early_stop(sd, t0, t1)
    if sd.stop is not None:
        t1 = min(t1, sd.stop - margin_s)
    return t0, t1, bool(t1 < sd.t_end)


def fit_segment_twists(segs, settle_s=0.5, min_duration_s=1.0, kinds=("drive",), statuses=USABLE):
    """Constant-twist fit for every drive segment with a usable status. Returns a list of row dicts."""
    rows = []
    for sd in segs:
        if sd.kind not in kinds or sd.status not in statuses:
            continue
        t0, t1, truncated = steady_window(sd, settle_s)
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
