"""Session folder(s) -> per-segment twists, forward map, inverse map + polytope, lag/dead time, noise.

Outputs (in out_dir):
  segments_fit.csv            one row per usable segment: (L, R) -> (u1, u2, omega), ICR, fit quality, battery
  forward_map.json            selected parametric model + grouped-CV scores (and GP scores with --gp)
  forward_map_bootstrap.json  cell-bootstrap parameter draws (posterior-predictive DR in training)
  inverse_map.json            closed-form inverse, polytope H z <= h, single-motor pivots
  <name>.h                    closed-form inverse as C (lin model only)
  calib.json                  everything above in one summary, plus latency, dynamics and noise
"""
from __future__ import annotations

import json
from pathlib import Path

import numpy as np

from . import botlog, dynamics, extract, forward_map, inverse_map, noise


def _usable(rows, tags_exclude=("circle",)):
    return [r for r in rows if r.get("fit") == "ok" and r.get("tag") not in tags_exclude]


def analyze(session_dirs, out_dir, gp=False, bootstrap_n=50, settle_s=0.5, p_min=(60.0, 60.0), p_max=(120.0, 120.0),
            lag_max_segments=60, name="graspion_map"):
    out = Path(out_dir)
    out.mkdir(parents=True, exist_ok=True)
    all_rows, dyn_segs, link, offsets, merges, statics, heading_runs, circle_runs = [], [], [], [], [], [], [], []
    sessions, seg_lists = [], []
    for sd in session_dirs:
        sess = extract.load_session(sd)
        sessions.append(sess)
        segs, off = extract.extract_segments(sess)
        offsets.append({k: off[k] for k in ("cam_offset", "cam_offset_mad", "found", "sent")})
        fr = sess.frames
        offsets[-1]["bridge_req_to_ack_s"] = float(np.median(fr["t_ack"] - fr["t_req"])) if len(fr["t_ack"]) else float("nan")
        if np.any(sess.segments["kind"] == "ping"):
            link.append(extract.link_stats(sess))
        rows = extract.fit_segment_twists(segs, settle_s=settle_s)
        dump = Path(sd) / "bot_dump.csv"
        if dump.exists():
            _, _, recs = botlog.parse_dump(dump.read_text())
            merges.append(botlog.merge(rows, fr, recs))
        for r in rows:
            r["session"] = Path(sd).name
        all_rows += rows
        for s in segs:
            if s.kind == "static":
                m = (s.t >= s.t_cmd) & (s.t <= s.t_end)
                if m.sum() > 20:
                    statics.append(noise.static_noise(s.t[m], s.x[m], s.y[m], s.th[m]))
            elif s.kind in ("drive", "steps", "prbs") and s.status in extract.USABLE:
                m = (s.t >= s.t_cmd + settle_s) & (s.t <= s.t_end)
                if m.sum() > 20 and len(s.levels) <= 2:
                    (circle_runs if s.tag == "circle" else heading_runs).append((s.t[m], s.th[m]))
        seg_lists.append(segs)
    extract.write_rows(all_rows, out / "segments_fit.csv")

    pwm = {str(s.meta.get("pwm_hz")) for s in sessions}
    if len(pwm) > 1:
        raise ValueError(f"sessions mix PWM modes {sorted(pwm)}: a map from one mode is invalid in the other")
    use = _usable(all_rows)
    summary = {"sessions": [str(s) for s in session_dirs], "n_segments_fit": len(use), "latency": offsets,
               "link_tests": link, "bot_log_merge": merges}
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
        inv = inverse_map.InverseMap(fwd, p_min=p_min, p_max=p_max,
                                     v_bat=float(np.median(V)) if (V is not None and fwd.features == "bat") else None)
        inv.save(out / "inverse_map.json")
        if inv.linear:
            (out / f"{name}.h").write_text(inv.c_header(name))
    except ValueError as e:
        summary["inverse_error"] = str(e)

    # dynamics: step / PRBS segments first, else the start/stop transients of plain drive segments
    for segs in seg_lists:
        for s in segs:
            if s.status not in extract.USABLE or s.kind == "static":
                continue
            if s.kind in ("steps", "prbs") or (s.kind == "drive" and s.tag != "circle"):
                dyn_segs.append(dynamics.DynSegment(s.t, s.x, s.y, s.th, s.levels, fwd))
    stepped = [d for d in dyn_segs if len(d.levels) > 2]
    pool = stepped if len(stepped) >= 5 else dyn_segs
    if len(pool) > lag_max_segments:
        pool = [pool[i] for i in np.random.default_rng(0).choice(len(pool), lag_max_segments, replace=False)]
    lag = dynamics.fit_lag(pool) if pool else {}

    icr = {}
    for name_, sel in (("ccw", lambda r: r["omega"] > 0.5), ("cw", lambda r: r["omega"] < -0.5)):
        rr = [r for r in use if sel(r)]
        if rr:
            icr[name_] = {"icr_x_median": float(np.median([r["icr_x"] for r in rr])),
                          "icr_y_median": float(np.median([r["icr_y"] for r in rr])), "n": len(rr)}

    summary.update({
        "forward_map": fwd.to_dict(), "polarity_check": polarity,
        "inverse_map": None if inv is None else {k: v for k, v in inv.to_dict().items() if k != "forward"},
        "dynamics": lag, "dynamics_segments": len(pool),
        "noise": {"heading_segments": noise.heading_diffusion(heading_runs, max_lag_s=0.5, ou=False),
                  "heading_long_runs": noise.heading_diffusion(circle_runs, max_lag_s=2.0, ou=True) if circle_runs else None,
                  "static": statics,
                  "segment_pos_rms_median": float(np.median([r["pos_rms"] for r in use])),
                  "lateral_u2_median": float(np.median(Y[:, 1])), "lateral_u2_sd": float(np.std(Y[:, 1]))},
        "icr": icr,
        "notes": ["UNTESTED pipeline: validated on simulated sessions only until run on a bot.",
                  "u1/u2/omega are the body twist of the reference point named in forward_map.meta."],
    })
    (out / "calib.json").write_text(json.dumps(summary, indent=2, default=float))
    return summary
