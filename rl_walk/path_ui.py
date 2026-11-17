"""Path editor: draw a path in the browser, see whether the tracking policy follows it in simulation, then run it on
the robot and watch the driven path appear next to the camera view.

    python rl_walk/path_ui.py            # then open http://localhost:8090/

The page (path_ui.html) sends the waypoints here. This server turns them into a reference (track_env.PathRef),
simulates the policy on it (the plant fitted from robot logs if there is one, plus a few randomised robots), and for
the robot starts deploy.py as a separate process: --shape path --approach, with a stop file behind the Stop button
and the log it writes read back while it runs. Only the standard library on the server side; nothing is sent to the
robot unless "Run on robot" is pressed.
"""
import argparse
import csv
import glob
import json
import os
import subprocess
import sys
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

from env import ARENA
from track_env import EDGE, LOST, PathRef, TrackEnv

HERE = os.path.dirname(os.path.abspath(__file__))
RUNS = os.path.join(HERE, "runs")
PATHS = os.path.join(RUNS, "paths")
STOP = os.path.join(PATHS, ".stop")
TRAINED_SPEED, FASTEST_CELL, TESTED_RADIUS = 0.062, 0.075, 0.02     # limits the page warns about [m/s, m/s, m]
MODELS, ROBOT = {}, {"proc": None, "log": None, "csv": None, "spec": None, "approach": True}


def policies():
    names = sorted(os.path.basename(p)[:-4] for p in glob.glob(os.path.join(RUNS, "ppo_track*.zip")))
    return sorted(names, key=lambda n: n != "ppo_track_ft")           # the fine-tuned one first


def plants():
    return sorted((os.path.basename(p) for p in glob.glob(os.path.join(RUNS, "residual_*.json"))), reverse=True)


def model(name):
    if name not in MODELS:
        from stable_baselines3 import PPO
        MODELS[name] = PPO.load(os.path.join(RUNS, name + ".zip"), device="cpu")
    return MODELS[name]


def reference(q):
    ref = PathRef(q["points"], speed=float(q.get("speed", 0.045)), closed=bool(q.get("closed")),
                  w_max=float(q.get("w_max", 1.0)))
    s = ref.summary()
    x, y = ref.xy[:, 0], ref.xy[:, 1]
    inside = bool(x.min() >= ARENA[0, 0] + EDGE and x.max() <= ARENA[0, 1] - EDGE
                  and y.min() >= ARENA[1, 0] + EDGE and y.max() <= ARENA[1, 1] - EDGE)
    warn = []
    if not inside:
        warn.append(f"The path leaves the allowed area (it must stay {100 * EDGE:.0f} cm inside the arena). It cannot be run.")
    if s["v_max_cms"] > 100 * FASTEST_CELL:
        warn.append(f"Peak speed {s['v_max_cms']:.1f} cm/s is above the robot's fastest measured command (about 7.5 cm/s).")
    elif s["v_max_cms"] > 100 * TRAINED_SPEED:
        warn.append(f"Peak speed {s['v_max_cms']:.1f} cm/s is above anything the policy trained on (6.2 cm/s).")
    if s["min_radius_cm"] < 100 * TESTED_RADIUS:
        warn.append(f"Tightest turn has a {s['min_radius_cm']:.1f} cm radius; the tightest tested on the robot is 2 cm.")
    return ref, s, inside, warn


def thin(a, n=400):
    a = np.asarray(a)
    return np.round(a[:: max(1, len(a) // n)], 4).tolist()


def simulate(q):
    ref, s, inside, warn = reference(q)
    loops = float(q.get("loops", 1))
    dur = loops * ref.T if ref.closed else ref.T + 1.0
    plant = os.path.join(RUNS, q["plant"]) if q.get("plant") else None
    m = model(q.get("policy") or policies()[0])

    def run(seed, randomize):
        env = TrackEnv(ref=ref, randomize=randomize, residual=plant, t_max=dur)
        env.reset(seed=seed)
        x, y, th = (float(v) for v in ref.at(0.0)[:3])
        env.t0, env.pose = 0.0, np.array([x + 0.02, y, th + 0.2])   # start 2 cm beside the path, 0.2 rad off
        rows = []
        while True:
            _, _, term, trunc, info = env.step(int(m.predict(env._obs(), deterministic=True)[0]))
            rx, ry = (float(v) for v in ref.at(env.t)[:2])
            rows.append((env.t, env.pose[0], env.pose[1], rx, ry, 100 * info["err"]))
            if term or trunc:
                break
        a = np.array(rows)
        e = a[a[:, 0] > 3.0, 5] if (a[:, 0] > 3.0).any() else a[:, 5]
        return a, dict(median=float(np.median(e)), max=float(e.max()), lost=bool(info["lost"]))

    a, st = run(0, False)
    rand = [run(100 + i, True)[1] for i in range(5)]
    return dict(summary=s, warnings=warn, inside=inside, t=np.round(a[:, 0], 2).tolist(), robot=np.round(a[:, 1:3], 4).tolist(),
                ref=np.round(a[:, 3:5], 4).tolist(), err=np.round(a[:, 5], 2).tolist(), stats=st, random=rand,
                lost_cm=100 * LOST)


def robot_start(q):
    if ROBOT["proc"] is not None and ROBOT["proc"].poll() is None:
        return dict(ok=False, error="a run is already going")
    ref, s, inside, warn = reference(q)
    if not inside:
        return dict(ok=False, error="the path leaves the allowed area")
    os.makedirs(PATHS, exist_ok=True)
    if os.path.exists(STOP):
        os.remove(STOP)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    spec = os.path.join(PATHS, f"path_{stamp}.json")
    json.dump(dict(points=q["points"], speed=float(q.get("speed", 0.045)), closed=bool(q.get("closed")),
                   w_max=float(q.get("w_max", 1.0))), open(spec, "w"))
    mode = "run" if q.get("mode") == "run" else "dry"
    cmd = [sys.executable, os.path.join(HERE, "deploy.py"), "--mode", mode, "--task", "track", "--shape", "path",
           "--path", spec, "--approach", "--loops", str(float(q.get("loops", 1))), "--stop-file", STOP,
           "--policy", os.path.join(RUNS, (q.get("policy") or policies()[0]) + ".zip"), "--port", q.get("port") or "COM5"]
    log = os.path.join(PATHS, f"path_{stamp}.log")
    ROBOT.update(proc=subprocess.Popen(cmd, cwd=HERE, stdout=open(log, "w"), stderr=subprocess.STDOUT), log=log,
                 csv=None, spec=spec, mode=mode, t0=time.time())
    return dict(ok=True, mode=mode)


def robot_status():
    p = ROBOT["proc"]
    if p is None:
        return dict(state="idle")
    lines = [ln.rstrip() for ln in open(ROBOT["log"], errors="replace")] if os.path.exists(ROBOT["log"]) else []
    for ln in lines:
        if ln.startswith("log: "):
            ROBOT["csv"] = ln[5:].strip()
    out = dict(state="running" if p.poll() is None else "done", mode=ROBOT["mode"], lines=lines[-6:], approach=[],
               track=[], stats=None)
    if ROBOT["csv"] and os.path.exists(ROBOT["csv"]):
        try:
            rows = [r for r in csv.DictReader(open(ROBOT["csv"])) if r.get("sx")]
        except Exception:                       # the file is being written
            rows = []
        tr = [r for r in rows if r["task"] == "1"]
        out["approach"] = [[float(r["sx"]), float(r["sy"])] for r in rows if r["task"] == "0"][::2]
        out["track"] = [[float(r["sx"]), float(r["sy"])] for r in tr]
        if tr:
            t = np.array([float(r["t"]) for r in tr])
            e = 100 * np.array([float(r["dist"]) for r in tr])
            k = t - t[0] > 3.0
            out["now"] = dict(t=float(t[-1] - t[0]), err=float(e[-1]), target=[float(tr[-1]["tx"]), float(tr[-1]["ty"])])
            if k.any():
                out["stats"] = dict(median=float(np.median(e[k])), max=float(e[k].max()))
    if out["state"] == "done":
        out["result"] = next((ln for ln in reversed(lines) if ln.startswith("end:")), lines[-1] if lines else "")
    return out


def robot_stop():
    p = ROBOT["proc"]
    if p is None or p.poll() is not None:
        return dict(ok=True)
    os.makedirs(PATHS, exist_ok=True)
    open(STOP, "w").close()                     # deploy.py stops the robot and exits on its next tick
    try:
        p.wait(timeout=6)
    except subprocess.TimeoutExpired:           # e.g. still opening the camera: nothing is driving yet
        p.terminate()
    return dict(ok=True)


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def reply(self, obj, code=200, ctype="application/json"):
        body = obj if isinstance(obj, bytes) else json.dumps(obj).encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            self.reply(open(os.path.join(HERE, "path_ui.html"), "rb").read(), ctype="text/html; charset=utf-8")
        elif self.path == "/api/config":
            self.reply(dict(arena=ARENA.tolist(), edge=EDGE, policies=policies(), plants=plants(),
                            preview_port=8080))
        elif self.path == "/api/robot/status":
            self.reply(robot_status())
        else:
            self.reply(dict(error="not found"), 404)

    def do_POST(self):
        q = json.loads(self.rfile.read(int(self.headers.get("Content-Length", 0)) or 0) or b"{}")
        try:
            if self.path == "/api/reference":
                ref, s, inside, warn = reference(q)
                self.reply(dict(ok=True, path=thin(ref.xy), summary=s, inside=inside, warnings=warn,
                                start=[float(v) for v in ref.at(0.0)[:3]]))
            elif self.path == "/api/simulate":
                self.reply(dict(ok=True, **simulate(q)))
            elif self.path == "/api/robot/start":
                self.reply(robot_start(q))
            elif self.path == "/api/robot/stop":
                self.reply(robot_stop())
            else:
                self.reply(dict(error="not found"), 404)
        except Exception as e:                   # a bad path (too few points, ...) is the user's to fix, not a crash
            self.reply(dict(ok=False, error=str(e)))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8090)
    a = ap.parse_args()
    if not policies():
        raise SystemExit("no tracking policy in runs/ (train_track.py makes ppo_track.zip)")
    model(policies()[0])                         # load once, so the first Simulate is quick
    print(f"path editor: http://localhost:{a.port}/   (Ctrl-C ends it; policies {', '.join(policies())})", flush=True)
    srv = ThreadingHTTPServer(("127.0.0.1", a.port), Handler)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        robot_stop()


if __name__ == "__main__":
    main()
