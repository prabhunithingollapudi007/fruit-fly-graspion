"""Session folder(s) -> per-segment twists, forward map, inverse map + polytope, lag/dead time, noise.

Outputs (in out_dir):
  segments_fit.csv            one row per usable segment: (L, R) -> (u1, u2, omega), ICR, fit quality, battery
  forward_map.json            selected parametric model + grouped-CV scores (and GP scores with --gp)
  forward_map_bootstrap.json  cell-bootstrap parameter draws (posterior-predictive DR in training)
  inverse_map.json            closed-form inverse, polytope H z <= h, single-motor pivots
  <name>.h                    closed-form inverse as C (lin model only)
  calib.json                  everything above in one summary except the bootstrap draws (those stay in
                              forward_map_bootstrap.json), plus latency, dynamics and noise
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from . import botlog, dynamics, extract, forward_map, inverse_map, noise


def _usable(rows, tags_exclude=("circle",)):
    return [r for r in rows if r.get("fit") == "ok" and r.get("tag") not in tags_exclude]


def mark_bot_ends(segs, matched):
    """Copy the bot log's end reason onto each SegmentData, and for segments its command watchdog or low-battery
    latch ended early, the time it stopped driving on the camera clock (t_trip)."""
    info, by_seg = matched
    rate = info.get("clock_rate", 1.0)
    for s in segs:
        hit = by_seg.get((s.seg, s.attempt))
        if hit is None:
            continue
        rec = hit[0]
        s.bot_end = botlog.END_REASONS.get(rec.get("end_reason"), str(rec.get("end_reason")))
        dur = (rec.get("t_end_ms", 0) - rec.get("t_start_ms", 0)) / 1000.0
        if s.bot_end in extract.BOT_ABANDONED and dur > 0:
            s.t_trip = s.t_cmd + rate * dur


def noise_runs(segs, settle_s=0.5, margin_s=0.3):
    """(heading_runs, circle_runs, statics) for noise.py. Drive runs stop where the commanded drive stopped
    acting (extract.steady_window, with a wider margin than the twist fit): a bot that stopped early (watchdog
    trip) breaks the constant-omega premise of the second-difference estimator and inflates D_r."""
    heading, circle, statics = [], [], []
    for s in segs:
        if s.kind == "static":
            m = (s.t >= s.t_cmd) & (s.t <= s.t_end)
            if m.sum() > 20:
                statics.append(noise.static_noise(s.t[m], s.x[m], s.y[m], s.th[m]))
        elif s.kind in ("drive", "steps", "prbs") and s.status in extract.USABLE and len(s.levels) <= 2:
            t0, t1, _ = extract.steady_window(s, settle_s, margin_s)
            m = (s.t >= t0) & (s.t <= t1)
            if m.sum() > 20:
                (circle if s.tag == "circle" else heading).append((s.t[m], s.th[m]))
    return heading, circle, statics


def dynamics_segments(seg_lists, fwd, settle_s=0.5, v_bat=None):
    """DynSegments for the lag fit (step / PRBS segments and non-circle drive segments) and the number skipped
    because the bot abandoned them early: the model assumes the commanded levels act until the STOP. A step down
    to a slow level is not taken for a stop: extract.abandoned judges only the levels fwd expects to be fast."""
    dyn, skipped = [], 0
    for segs in seg_lists:
        for s in segs:
            if s.status not in extract.USABLE or s.kind == "static":
                continue
            if s.kind in ("steps", "prbs") or (s.kind == "drive" and s.tag != "circle"):
                if extract.abandoned(s, settle_s, fwd=fwd, v_bat=v_bat):
                    skipped += 1
                    continue
                dyn.append(dynamics.DynSegment(s.t, s.x, s.y, s.th, s.levels, fwd, v_bat=v_bat))
    return dyn, skipped


def analyze(session_dirs, out_dir, gp=False, bootstrap_n=50, settle_s=0.5, p_min=None, p_max=(120.0, 120.0),
            lag_max_segments=60, name="graspion_map"):
    """p_min=None: per motor max(60, p0 + 5) (inverse_map.InverseMap), so a large dead zone still gets an inverse."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    all_rows, link, offsets, merges, statics, heading_runs, circle_runs = [], [], [], [], [], [], []
    seg_lists, notes = [], []
    sessions = [extract.load_session(sd) for sd in session_dirs]
    cmd_offs = [extract.command_offset(s) for s in sessions]
    measured = [o["cam_offset"] for o in cmd_offs if np.isfinite(o["cam_offset"])]
    # lent to sessions without a flash; a lag that may be one frame spacing off (extract.match_flashes) is not
    lendable = [o["cam_offset"] for o in cmd_offs if np.isfinite(o["cam_offset"]) and not o["ambiguous"]]
    pooled = float(np.median(lendable)) if lendable else None
    for sd, sess, off0 in zip(session_dirs, sessions, cmd_offs):
        segs, off = extract.extract_segments(sess, off=off0, fallback_offset=pooled)
        offsets.append({k: off[k] for k in ("cam_offset", "cam_offset_mad", "found", "sent", "cam_offset_used",
                                            "cam_offset_source", "ambiguous")})
        if off.get("warning"):
            offsets[-1]["warning"] = off["warning"]
            notes.append(off["warning"])
        fr = sess.frames
        offsets[-1]["bridge_req_to_ack_s"] = float(np.median(fr["t_ack"] - fr["t_req"])) if len(fr["t_ack"]) else float("nan")
        if np.any(sess.segments["kind"] == "ping"):
            link.append(extract.link_stats(sess))
        dump = Path(sd) / "bot_dump.csv"
        matched = None
        if dump.exists():
            _, _, recs = botlog.parse_dump(dump.read_text())
            matched = botlog.segment_records(fr, recs)
            mark_bot_ends(segs, matched)
        rows = extract.fit_segment_twists(segs, settle_s=settle_s)
        if matched is not None:
            merges.append(botlog.merge(rows, fr, recs, matched=matched))
            if merges[-1].get("warning"):
                notes.append(f"{Path(sd).name}: {merges[-1]['warning']}")
        for r in rows:
            r["session"] = Path(sd).name
        all_rows += rows
        h, c, st = noise_runs(segs, settle_s)
        heading_runs += h
        circle_runs += c
        statics += st
        seg_lists.append(segs)
    extract.write_rows(all_rows, out / "segments_fit.csv")

    pwm = {str(s.meta.get("pwm_hz")) for s in sessions}
    if len(pwm) > 1:
        raise ValueError(f"sessions mix PWM modes {sorted(pwm)}: a map from one mode is invalid in the other")
    use = _usable(all_rows)
    summary = {"sessions": [str(s) for s in session_dirs], "n_segments_fit": len(use), "latency": offsets,
               "link_tests": link, "bot_log_merge": merges}
    if notes:
        summary["warnings"] = notes
    if len(use) < 8:
        summary["error"] = f"only {len(use)} usable segments; need more for a map"
        (out / "calib.json").write_text(json.dumps(summary, indent=2, default=float))
        return summary

    L = np.array([r["L"] for r in use], float)
    R = np.array([r["R"] for r in use], float)
    Y = np.array([[r["u1"], r["u2"], r["omega"]] for r in use], float)
    V = np.array([r.get("V_bat", np.nan) for r in use], float)
    V = V if np.all(np.isfinite(V)) else None

    fwd = forward_map.select_and_fit(L, R, Y, V=V, gp=gp)
    fwd.meta["reference_point"] = sessions[0].geometry.reference_point
    fwd.meta["pwm_hz"] = sessions[0].meta.get("pwm_hz")
    if V is not None:
        fwd.meta["v_bat_median"] = float(np.median(V))     # where a 'bat' map is evaluated without per-sample V
    fwd.save(out / "forward_map.json")
    resid = Y - fwd.predict(L, R, V if fwd.features == "bat" else None)
    rev = (L < 0) | (R < 0)
    polarity = None
    if rev.any() and (~rev).any():
        polarity = {"rmse_reverse": np.sqrt(np.mean(resid[rev] ** 2, axis=0)).tolist(),
                    "rmse_forward": np.sqrt(np.mean(resid[~rev] ** 2, axis=0)).tolist(),
                    "mean_resid_reverse": resid[rev].mean(axis=0).tolist(),
                    "note": "the model uses |drive|; a large reverse bias means polarity matters on this bot"}
    if bootstrap_n:
        draws = forward_map.bootstrap(L, R, Y, V=V if fwd.features == "bat" else None, features=fwd.features,
                                      n=bootstrap_n)
        (out / "forward_map_bootstrap.json").write_text(json.dumps(draws))

    inv = None
    try:
        inv = inverse_map.InverseMap(fwd, p_min=p_min, p_max=p_max)     # a 'bat' map: at meta v_bat_median
        inv.save(out / "inverse_map.json")
        if inv.linear:
            (out / f"{name}.h").write_text(inv.c_header(name))
        if inv.p_min_auto and np.any(inv.p_min > inverse_map.P_MIN_DEFAULT):
            notes.append(f"dead zone p0 {np.round(fwd.p0, 1).tolist()} reaches the default p_min: the operating "
                         f"region starts at p_min {np.round(inv.p_min, 1).tolist()}")
    except ValueError as e:
        summary["inverse_error"] = str(e)

    # dynamics: step / PRBS segments first, else the start/stop transients of plain drive segments
    dyn_segs, n_abandoned = dynamics_segments(seg_lists, fwd, settle_s)
    stepped = [d for d in dyn_segs if len(d.levels) > 2]
    pool = stepped if len(stepped) >= 5 else dyn_segs
    if len(pool) > lag_max_segments:
        pool = [pool[i] for i in np.random.default_rng(0).choice(len(pool), lag_max_segments, replace=False)]
    offset_measured = bool(measured)
    lag = dynamics.fit_lag(pool, d_range=(0.0, 0.4) if offset_measured else (0.0, 0.6)) if pool else {}
    if lag and not offset_measured:
        notes.append("no sync flash in any session: segment times are blaster acknowledgements, so 'dead_time' "
                     "is the total delay (dead time + IR decode + camera latency), and cam_offset is unmeasured")

    icr = {}
    for name_, sel in (("ccw", lambda r: r["omega"] > 0.5), ("cw", lambda r: r["omega"] < -0.5)):
        rr = [r for r in use if sel(r)]
        if rr:
            icr[name_] = {"icr_x_median": float(np.median([r["icr_x"] for r in rr])),
                          "icr_y_median": float(np.median([r["icr_y"] for r in rr])), "n": len(rr)}

    summary.update({
        "forward_map": fwd.to_dict(), "polarity_check": polarity,
        "inverse_map": None if inv is None else {k: v for k, v in inv.to_dict().items() if k != "forward"},
        "dynamics": lag, "dynamics_segments": len(pool), "dynamics_skipped_abandoned": n_abandoned,
        "dead_time_includes_cam_offset": not offset_measured,
        "noise": {"heading_segments": noise.heading_diffusion(heading_runs, max_lag_s=0.5, ou=False),
                  "heading_long_runs": noise.heading_diffusion(circle_runs, max_lag_s=2.0, ou=True) if circle_runs else None,
                  "static": statics,
                  "segment_pos_rms_median": float(np.median([r["pos_rms"] for r in use])),
                  "lateral_u2_median": float(np.median(Y[:, 1])), "lateral_u2_sd": float(np.std(Y[:, 1]))},
        "icr": icr,
        "notes": ["UNTESTED pipeline: validated on simulated sessions only until run on a bot.",
                  "u1/u2/omega are the body twist of the reference point named in forward_map.meta."],
    })
    if notes:
        summary["warnings"] = notes
    (out / "calib.json").write_text(json.dumps(summary, indent=2, default=float))
    return summary
