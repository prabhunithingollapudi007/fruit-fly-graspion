"""The bot's own log (USB dump of calib_bot: 'd'), merged into the PC segment table.

The bot records every SEG it decoded with its own crystalless clock (+-2 % on battery). Records are matched to
the PC's SEG frames in order, by (L, R) and by consistent spacing (a chained alignment that tolerates leftover
records from bench checks or earlier sessions, and SEGs the bot never decoded); then t_host = a * t_bot + b is
fitted over the matches (a - 1 is the bot's clock-rate error). Matched rows gain battery (mV) and magnetometer
(uT) columns. V_bat (the 'bat' regressor) is the rest voltage read just before the segment's drive was applied:
the under-load voltage sags with the commanded drive itself, so it would confound battery and drive effects.
"""
from __future__ import annotations

import csv
import io
import time
import warnings

import numpy as np

END_REASONS = {0: "open", 1: "next_seg", 2: "recentre", 3: "stop", 4: "watchdog", 5: "lowbat", 6: "ping"}


def parse_dump(text: str):
    """Returns (info: dict, stats: dict, records: list[dict])."""
    info, stats, body = {}, {}, []
    for line in text.splitlines():
        line = line.strip()
        if not line:
            continue
        if line.startswith("# I "):
            info = dict(kv.split("=", 1) for kv in line[4:].split() if "=" in kv)
        elif line.startswith("# S "):
            stats = dict(kv.split("=", 1) for kv in line[4:].split() if "=" in kv)
        elif not line.startswith("#"):
            body.append(line)
    recs = list(csv.DictReader(io.StringIO("\n".join(body))))
    for r in recs:
        for k, v in r.items():
            try:
                r[k] = int(v)
            except (TypeError, ValueError):
                pass
    return info, stats, recs


def read_dump_serial(port, baud=115200, timeout_s=20.0):
    """Ask a USB-connected bot for its log ('d') and return the raw text (needs pyserial)."""
    import serial
    with serial.Serial(port, baud, timeout=0.5) as s:
        time.sleep(0.3)
        s.reset_input_buffer()
        s.write(b"d")
        lines, t0 = [], time.time()
        while time.time() - t0 < timeout_s:
            ln = s.readline().decode("ascii", "replace")
            if ln:
                lines.append(ln.rstrip("\r\n"))
                if ln.startswith("# end"):
                    break
        return "\n".join(lines)


def read_info_serial(port, baud=115200):
    import serial
    with serial.Serial(port, baud, timeout=1.0) as s:
        time.sleep(0.3)
        s.reset_input_buffer()
        s.write(b"i")
        ln = s.readline().decode("ascii", "replace").strip()
    if not ln.startswith("I "):
        raise RuntimeError(f"unexpected reply {ln!r}")
    return dict(kv.split("=", 1) for kv in ln[2:].split() if "=" in kv)


def _chain(acks, LRp, tb, LRb, i0, j0, rate_tol, abs_tol, rate_tol_fit, fit_span_s):
    """Extend one anchor pair (i0, j0) forward in time. Returns the list of (pc_index, rec_index) pairs."""
    pairs = [(i0, j0)]
    h_ref, b_ref, rate, tol_r = acks[i0], tb[j0], 1.0, rate_tol
    j = j0 + 1
    for i in range(i0 + 1, len(acks)):
        dh = acks[i] - h_ref
        if dh <= 0:
            continue
        tol = abs_tol + tol_r * dh
        best, best_err = None, np.inf
        jj = j
        while jj < len(tb):
            err = rate * (tb[jj] - b_ref) - dh             # predicted host time of record jj minus this PC SEG
            if err < -tol:                               # too early for this and every later PC SEG: pass it
                if jj == j:
                    j += 1
                jj += 1
                continue
            if err > tol:                                # records are in time order
                break
            if LRb[jj] == LRp[i] and abs(err) < best_err:
                best, best_err = jj, abs(err)
            jj += 1
        if best is None:
            continue
        pairs.append((i, best))
        h_ref, b_ref, j = acks[i], tb[best], best + 1
        if acks[i] - acks[pairs[0][0]] >= fit_span_s and len(pairs) >= 3:
            hs = np.array([acks[p] for p, _ in pairs])
            bs = np.array([tb[q] for _, q in pairs])     # clock rate: median of the step slopes, so one wrong
            rate, tol_r = float(np.median(np.diff(hs) / np.diff(bs))), rate_tol_fit   # pair cannot tilt it
    return pairs


def _local_outliers(hs, bs, tol, rate_tol_fit):
    """Pairs whose host time disagrees with every local clock line: through the two pairs before, the two after,
    and the two either side (tolerance tol + rate_tol_fit * distance). One bad pair cannot condemn its
    neighbours, because they always have a line on the far side."""
    n = len(hs)
    bad = np.zeros(n, bool)
    if n < 4:
        return bad

    def agrees(nb, k):
        a, b0 = np.polyfit(bs[nb], hs[nb], 1)
        return abs(a * bs[k] + b0 - hs[k]) <= tol + rate_tol_fit * np.min(np.abs(hs[nb] - hs[k]))

    for k in range(n):
        sides = ([[k - 2, k - 1]] if k >= 2 else []) + ([[k + 1, k + 2]] if k <= n - 3 else []) \
            + ([[k - 1, k + 1]] if 0 < k < n - 1 else [])
        bad[k] = not any(agrees(nb, k) for nb in sides)
    return bad


def match_records(pc_seg_acks, pc_LR, bot_recs, rate_tol=0.05, abs_tol=0.25, rate_tol_fit=0.01, fit_span_s=20.0):
    """In-order matching of PC SEG frames to bot SEG records. Returns list of (pc_index, bot_index).

    Every (L, R)-compatible pair (PC SEG i0, bot SEG record j0) is tried as an anchor and extended forward: the
    next PC SEG takes the record whose host time, predicted from the last match (clock rate 1 +- rate_tol, then
    the chain's own fitted rate +- rate_tol_fit), lies within abs_tol + tol * spacing and whose (L, R) agrees.
    Records predicted before the current PC SEG are passed for good, so leftover records (bench checks, an
    earlier session in the same dump) and SEGs the bot never decoded cannot stall or shift the alignment. The
    longest chain wins (ties: smallest clock residual); distinct chains of equal length that cannot be told
    apart leave everything unmatched rather than attach another segment's battery data. Finally pairs that
    disagree with their neighbours' clock line are dropped.
    """
    acks = np.asarray(pc_seg_acks, float)
    LRp = [(int(L), int(R)) for L, R in pc_LR]
    idx = [k for k, r in enumerate(bot_recs) if r.get("kind") == 1]
    tb = np.array([bot_recs[k]["t_start_ms"] / 1000.0 for k in idx], float)
    LRb = [(int(bot_recs[k]["L"]), int(bot_recs[k]["R"])) for k in idx]
    if not len(acks) or not len(tb):
        return []
    tried, chains = set(), []
    for i0 in range(len(acks)):
        for j0 in range(len(tb)):
            if LRb[j0] != LRp[i0] or (i0, j0) in tried:
                continue
            ch = _chain(acks, LRp, tb, LRb, i0, j0, rate_tol, abs_tol, rate_tol_fit, fit_span_s)
            tried.update(ch)                             # an anchor inside a known chain only gives its tail
            chains.append(ch)
    if not chains:
        return []
    n_best = max(len(c) for c in chains)
    best = [c for c in chains if len(c) == n_best]

    def resid(c):
        if len(c) < 3:
            return np.inf
        hs, bs = np.array([acks[i] for i, _ in c]), np.array([tb[j] for _, j in c])
        a, b0 = np.polyfit(bs, hs, 1)
        return float(np.std(hs - (a * bs + b0)))

    distinct = {tuple(c) for c in best}
    if len(distinct) > 1:
        rs = sorted((resid(c), k) for k, c in enumerate(best))
        if not (np.isfinite(rs[0][0]) and rs[0][0] < 0.5 * rs[1][0]):
            return []                                    # ambiguous (e.g. the same schedule twice in one dump)
        best = [best[rs[0][1]]]
    ch = best[0]
    hs, bs = np.array([acks[i] for i, _ in ch]), np.array([tb[j] for _, j in ch])
    bad = _local_outliers(hs, bs, abs_tol, rate_tol_fit)
    return [(i, idx[j]) for (i, j), b in zip(ch, bad) if not b]


def segment_records(frames, bot_recs):
    """Match a session's SEG frames to the bot log. Returns (info, {(seg, attempt): (record, t_ack)}).

    The k-th SEG frame of segment `seg` is its attempt k (the scheduler sends one SEG per attempt)."""
    sel = (frames["op"] == "SEG")
    acks = frames["t_ack"][sel]
    LR = list(zip(frames["L"][sel], frames["R"][sel]))
    pairs = match_records(acks, LR, bot_recs)
    n_pc, n_bot = int(sel.sum()), sum(1 for r in bot_recs if r.get("kind") == 1)
    info = {"matched": len(pairs), "pc_seg_frames": n_pc, "bot_seg_records": n_bot}
    if len(pairs) >= 2:
        th = np.array([acks[i] for i, _ in pairs])
        tb = np.array([bot_recs[b]["t_start_ms"] / 1000.0 for _, b in pairs])
        (a, b0), *_ = np.linalg.lstsq(np.c_[tb, np.ones_like(tb)], th, rcond=None)
        info.update({"clock_rate": float(a), "clock_offset_s": float(b0),
                     "clock_resid_ms": float(1000 * np.std(th - (a * tb + b0)))})
    if min(n_pc, n_bot) and len(pairs) < 0.8 * min(n_pc, n_bot):
        info["warning"] = (f"only {len(pairs)} of {n_pc} PC SEG frames matched a bot log record ({n_bot} SEG "
                           "records): battery / magnetometer columns are missing for the rest. Clear the bot log "
                           "('c') before each session and dump it after the session.")
    seg_of = np.asarray(frames["seg"])[sel]
    attempt, by_seg = {}, {}
    k_of = []
    for s in seg_of:
        k_of.append(attempt.get(int(s), 0))
        attempt[int(s)] = k_of[-1] + 1
    for i, b in pairs:
        by_seg[(int(seg_of[i]), k_of[i])] = (bot_recs[b], float(acks[i]))
    return info, by_seg


def rest_voltage(rec):
    """Battery voltage [V] at rest just before the segment's drive was applied (None if not read)."""
    mv = rec.get("mv_rest")
    return mv / 1000.0 if isinstance(mv, (int, float)) and mv > 0 else None


def merge(seg_rows, frames, bot_recs, matched=None):
    """Attach bot-log fields to segment-fit rows (matched via the SEG acknowledgement time).

    matched: segment_records(frames, bot_recs) if already computed. Returns the match info dict."""
    info, by_seg = matched if matched is not None else segment_records(frames, bot_recs)
    if info.get("warning"):
        warnings.warn(info["warning"], stacklevel=2)
    for row in seg_rows:
        hit = by_seg.get((int(row["seg"]), int(row.get("attempt", 0))))
        if hit is None:
            continue
        rec = hit[0]
        row["bot_end"] = END_REASONS.get(rec.get("end_reason"), rec.get("end_reason"))
        row["bot_n_frames"] = rec.get("n_frames")
        for k in ("mv_rest", "mv_load_min", "mv_load_mean", "n_mag", "flags", "mag_rest_x", "mag_rest_y",
                  "mag_rest_z", "mag_load_x", "mag_load_y", "mag_load_z"):
            if k in rec:
                row[k] = rec[k]
        v = rest_voltage(rec)
        if v is not None:
            row["V_bat"] = v
    return info
