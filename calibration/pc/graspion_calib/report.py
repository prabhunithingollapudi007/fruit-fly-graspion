"""Session folder(s) -> per-segment twists, forward map, inverse map and feasible set.

Outputs (in out_dir):
  segments_fit.csv            one row per usable segment: (L, R) -> (u1, u2, omega), ICR, fit quality
  forward_map.json            selected model variant + grouped-CV scores
  forward_map_bootstrap.json  cell-bootstrap parameter draws
  inverse_map.json            closed-form inverse and the feasible polytope H z <= h
  calib.json                  the summary: latency, link tests, forward map, polarity check, inverse map,
                              replicate scatter, static noise, ICR, warnings
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from . import extract, forward_map, inverse_map


def _usable(rows):
    return [r for r in rows if r.get("fit") == "ok"]


def static_noise(segs):
    """Per static segment: SD of the body point's x, y [m] and heading [rad] at rest (the tracker's noise floor)."""
    out = []
    for s in segs:
        if s.kind != "static":
            continue
        m = (s.t >= s.t_cmd) & (s.t <= s.t_end)
        if m.sum() > 20:
            out.append({"seg": s.seg, "n": int(m.sum()), "sd_x": float(np.std(s.x[m])), "sd_y": float(np.std(s.y[m])),
                        "sd_theta": float(np.std(np.unwrap(s.th[m])))})
    return out


def replicate_scatter(use):
    """SD of (u1, u2, omega) across the replicates of each drive cell, summarised over cells: the segment-to-
    segment noise that a between-robot spread has to exceed to mean anything."""
    cells = {}
    for r in use:
        cells.setdefault((r["L"], r["R"]), []).append((r["u1"], r["u2"], r["omega"]))
    sds = np.array([np.std(np.array(v), axis=0, ddof=1) for v in cells.values() if len(v) >= 2])
    if not len(sds):
        return None
    return {"cells": int(len(sds)), "outputs": list(forward_map.OUTPUTS), "sd_median": np.median(sds, axis=0).tolist(),
            "sd_p90": np.percentile(sds, 90, axis=0).tolist()}


def analyze(session_dirs, out_dir, bootstrap_n=50, settle_s=0.5, p_min=None, p_max=(120.0, 120.0)):
    """p_min=None: per motor max(60, p0 + 5) (inverse_map.InverseMap), so a large dead zone still gets an inverse."""
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    all_rows, link, offsets, statics, notes = [], [], [], [], []
    sessions = [extract.load_session(sd) for sd in session_dirs]
    cmd_offs = [extract.command_offset(s) for s in sessions]
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
        rows = extract.fit_segment_twists(segs, settle_s=settle_s)
        for r in rows:
            r["session"] = Path(sd).name
        all_rows += rows
        statics += static_noise(segs)
    extract.write_rows(all_rows, out / "segments_fit.csv")

    pwm = {str(s.meta.get("pwm_hz")) for s in sessions}
    if len(pwm) > 1:
        raise ValueError(f"sessions mix PWM modes {sorted(pwm)}: a map from one mode is invalid in the other")
    use = _usable(all_rows)
    summary = {"sessions": [str(s) for s in session_dirs], "n_segments_fit": len(use), "latency": offsets,
               "link_tests": link, "static_noise": statics}
    if notes:
        summary["warnings"] = notes
    if len(use) < 8:
        summary["error"] = f"only {len(use)} usable segments; need more for a map"
        (out / "calib.json").write_text(json.dumps(summary, indent=2, default=float))
        return summary

    L = np.array([r["L"] for r in use], float)
    R = np.array([r["R"] for r in use], float)
    Y = np.array([[r["u1"], r["u2"], r["omega"]] for r in use], float)

    fwd = forward_map.select_and_fit(L, R, Y)
    fwd.meta["reference_point"] = sessions[0].geometry.reference_point
    fwd.meta["pwm_hz"] = sessions[0].meta.get("pwm_hz")
    fwd.save(out / "forward_map.json")
    resid = Y - fwd.predict(L, R)
    rev = (L < 0) | (R < 0)
    polarity = None
    if rev.any() and (~rev).any():
        polarity = {"rmse_reverse": np.sqrt(np.mean(resid[rev] ** 2, axis=0)).tolist(),
                    "rmse_forward": np.sqrt(np.mean(resid[~rev] ** 2, axis=0)).tolist(),
                    "mean_resid_reverse": resid[rev].mean(axis=0).tolist(),
                    "note": "the model uses |drive|; a large reverse bias means polarity matters on this bot"}
    if bootstrap_n:
        draws = forward_map.bootstrap(L, R, Y, features=fwd.features, n=bootstrap_n)
        (out / "forward_map_bootstrap.json").write_text(json.dumps(draws))

    inv = None
    try:
        inv = inverse_map.InverseMap(fwd, p_min=p_min, p_max=p_max)
        inv.save(out / "inverse_map.json")
        if inv.p_min_auto and np.any(inv.p_min > inverse_map.P_MIN_DEFAULT):
            notes.append(f"dead zone p0 {np.round(fwd.p0, 1).tolist()} reaches the default p_min: the operating "
                         f"region starts at p_min {np.round(inv.p_min, 1).tolist()}")
    except ValueError as e:
        summary["inverse_error"] = str(e)

    icr = {}
    for name_, sel in (("ccw", lambda r: r["omega"] > 0.5), ("cw", lambda r: r["omega"] < -0.5)):
        rr = [r for r in use if sel(r)]
        if rr:
            icr[name_] = {"icr_x_median": float(np.median([r["icr_x"] for r in rr])),
                          "icr_y_median": float(np.median([r["icr_y"] for r in rr])), "n": len(rr)}

    summary.update({
        "forward_map": fwd.to_dict(), "polarity_check": polarity,
        "inverse_map": None if inv is None else {k: v for k, v in inv.to_dict().items() if k != "forward"},
        "replicate_scatter": replicate_scatter(use), "icr": icr,
        "segment_pos_rms_median": float(np.median([r["pos_rms"] for r in use])),
        "lateral_u2_median": float(np.median(Y[:, 1])), "lateral_u2_sd": float(np.std(Y[:, 1])),
        "notes": ["UNTESTED pipeline: validated on simulated sessions only until run on a bot.",
                  "u1/u2/omega are the body twist of the reference point named in forward_map.meta."],
    })
    if notes:
        summary["warnings"] = notes
    (out / "calib.json").write_text(json.dumps(summary, indent=2, default=float))
    return summary
