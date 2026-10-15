"""The bot's own log (USB dump of calib_bot: 'd'), merged into the PC segment table.

The bot records every SEG it decoded with its own crystalless clock (+-2 % on battery). Records are matched to
the PC's SEG frames in order, by (L, R) and by consistent spacing; then t_host = a * t_bot + b is fitted over the
matches (a - 1 is the bot's clock-rate error). Matched rows gain battery (mV) and magnetometer (uT) columns.
"""
from __future__ import annotations

import csv
import io
import time

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


def match_records(pc_seg_acks, pc_LR, bot_recs, rate_tol=0.05):
    """Greedy in-order matching of PC SEG frames to bot SEG records. Returns list of (pc_index, bot_index)."""
    seg_recs = [(i, r) for i, r in enumerate(bot_recs) if r.get("kind") == 1]
    pairs, j = [], 0
    last = None
    for i, (ta, (L, R)) in enumerate(zip(pc_seg_acks, pc_LR)):
        for jj in range(j, min(j + 3, len(seg_recs))):
            bi, r = seg_recs[jj]
            if (r["L"], r["R"]) != (int(L), int(R)):
                continue
            if last is not None:
                dh = ta - last[0]
                db = (r["t_start_ms"] - last[1]) / 1000.0
                if dh <= 0 or abs(db / dh - 1) > rate_tol:
                    continue
            pairs.append((i, bi))
            last = (ta, r["t_start_ms"])
            j = jj + 1
            break
    return pairs


def merge(seg_rows, frames, bot_recs):
    """Attach bot-log fields to segment-fit rows (matched via the SEG acknowledgement time)."""
    sel = (frames["op"] == "SEG")
    acks = frames["t_ack"][sel]
    LR = list(zip(frames["L"][sel], frames["R"][sel]))
    pairs = match_records(acks, LR, bot_recs)
    out = {"matched": len(pairs), "pc_seg_frames": int(sel.sum()),
           "bot_seg_records": sum(1 for r in bot_recs if r.get("kind") == 1)}
    if len(pairs) >= 2:
        th = np.array([acks[i] for i, _ in pairs])
        tb = np.array([bot_recs[b]["t_start_ms"] / 1000.0 for _, b in pairs])
        (a, b0), *_ = np.linalg.lstsq(np.c_[tb, np.ones_like(tb)], th, rcond=None)
        out.update({"clock_rate": float(a), "clock_offset_s": float(b0),
                    "clock_resid_ms": float(1000 * np.std(th - (a * tb + b0)))})
    by_ack = {round(float(acks[i]), 4): bot_recs[b] for i, b in pairs}
    seg_ack = {}
    for k in range(len(frames["t_ack"])):
        if frames["op"][k] == "SEG":
            seg_ack.setdefault(int(frames["seg"][k]), []).append(round(float(frames["t_ack"][k]), 4))
    for row in seg_rows:
        acks_of_seg = seg_ack.get(int(row["seg"]), [])
        rec = None
        if acks_of_seg:
            rec = by_ack.get(acks_of_seg[min(int(row.get("attempt", 0)), len(acks_of_seg) - 1)])
        if rec is None:
            continue
        row["bot_end"] = END_REASONS.get(rec.get("end_reason"), rec.get("end_reason"))
        row["bot_n_frames"] = rec.get("n_frames")
        for k in ("mv_rest", "mv_load_min", "mv_load_mean", "n_mag", "flags", "mag_rest_x", "mag_rest_y",
                  "mag_rest_z", "mag_load_x", "mag_load_y", "mag_load_z"):
            if k in rec:
                row[k] = rec[k]
        v = rec.get("mv_load_mean") or rec.get("mv_rest")
        if v:
            row["V_bat"] = v / 1000.0
    return out
