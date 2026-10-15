"""Helpers: run a scheduler session against the simulated rig."""
from pathlib import Path

from graspion_calib import sim
from graspion_calib.clock import SimClock
from graspion_calib.geometry import Arena
from graspion_calib.poses import TrackerCSV
from graspion_calib.scheduler import Session, SessionConfig

BOT = 90


def abort_on_pause(msg):
    abort_on_pause.messages.append(msg)
    return "abort"


abort_on_pause.messages = []


def run_session(out, specs, params=None, start=(0.45, 0.5, 0.3), arena=None, seed=0, bridge_factory=None,
                on_pause=abort_on_pause, write_dump=True):
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    p = params or sim.SimParams()
    world = sim.SimWorld({BOT: (p, start)}, seed=seed, tracker_csv=TrackerCSV(out / "tracker.csv"))
    bridge = (bridge_factory or sim.SimBridge)(world)
    cfg = SessionConfig(BOT, p.geometry, arena or Arena(0.0, 1.0, 0.0, 1.0, 0.08), str(out))
    sess = Session(cfg, bridge, world.poses, SimClock(world), on_pause=on_pause, extra_meta={"pwm_hz": "sim"})
    try:
        counts = sess.run(specs)
    finally:
        world.csv.close()
    if write_dump:
        (out / "bot_dump.csv").write_text(world.fw[BOT].dump_text())
    return world, counts
