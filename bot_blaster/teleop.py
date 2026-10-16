"""Keyboard remote for one calib_bot, sent through the bot_blaster robot (or any ir_blaster).

UNTESTED on hardware with a calib_bot target. The blaster side is verified (HARDWARE.md 4.5); this script was
checked only against the simulated blaster and bot. For a target running an upstream example, use remote.html.

    python bot_blaster/teleop.py --port COM5 --id 42

Keys (Windows console): w / Up = forward, a / Left = turn left, d / Right = turn right, space / s / Down = stop,
+ / - = speed, q = quit. A command stays on until the next key; the script repeats it every blaster period
(110 ms) because calib_bot stops its motors 300 ms after the last drive frame. Frames are RECENTRE ops: they
drive (L, R) without opening a calibration segment, flashing or logging. On exit (q, Ctrl-C or an error) it
sends STOP to the bot and a broadcast STOP. If the script dies, the bot stops by its own watchdog.

The bot ID is in calib_bot's USB 'i' line (or bots/<id>.json after `register`). Needs pyserial.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "calibration", "pc"))
from graspion_calib.bridge import SerialBridge  # noqa: E402
from graspion_calib.protocol import ADDR_BROADCAST, OP_RECENTRE, OP_STOP, pack  # noqa: E402

KEYS = {"w": "fwd", "H": "fwd", "a": "left", "K": "left", "d": "right", "M": "right", " ": "stop", "s": "stop", "P": "stop"}


def getkey():
    """The next key or None (non-blocking). Arrow keys arrive as '\\xe0' + H/K/M/P."""
    import msvcrt
    if not msvcrt.kbhit():
        return None
    k = msvcrt.getwch()
    return msvcrt.getwch() if k in ("\x00", "\xe0") else k


def drive(mode, speed):
    # Only |drive| matters (HARDWARE.md 4.4); one motor alone turns the bot away from it (left motor -> right).
    return {"fwd": (speed, speed), "left": (0, speed), "right": (speed, 0)}.get(mode, (0, 0))


def run(bridge, bot_id, getkey=getkey, speed=80):
    mode = "stop"
    print("w/a/d or arrows = drive, space/s = stop, +/- = speed, q = quit")
    try:
        while True:
            k = getkey()
            if k == "q":
                break
            if k in KEYS:
                mode = KEYS[k]
            elif k in ("+", "="):
                speed = min(120, speed + 10)        # calib_bot caps |drive| at 120
            elif k == "-":
                speed = max(0, speed - 10)
            if k is not None:
                print(f"\r{mode:5s} speed {speed:3d}", end="", flush=True)
            L, R = drive(mode, speed)
            bridge.send(pack(bot_id, OP_RECENTRE, L, R) if (L or R) else pack(bot_id, OP_STOP))  # paces the loop
    finally:
        print()
        for raw in (pack(bot_id, OP_STOP), pack(ADDR_BROADCAST, OP_STOP)):
            try:
                bridge.send(raw)
            except Exception as e:  # still try the next STOP
                print(f"STOP failed: {e}", file=sys.stderr)


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--port", required=True, help="serial port of the blaster robot, e.g. COM5")
    p.add_argument("--id", type=int, required=True, help="bot ID of the calib_bot to drive (1..254)")
    p.add_argument("--speed", type=int, default=80, help="drive level 0..120 (default 80)")
    p.add_argument("--period-ms", type=int, default=110, help="frame period 100..250 ms (default 110); calib_bot stops 300 ms after a frame")
    a = p.parse_args()
    if not 1 <= a.id <= 254:
        p.error("--id must be 1..254")
    if not 100 <= a.period_ms <= 250:
        p.error("--period-ms must be 100..250: a longer gap trips calib_bot's 300 ms watchdog between frames")
    bridge = SerialBridge(a.port, period_ms=a.period_ms, reset_wait_s=0.2)  # native USB: no reset on open
    print(bridge.info)
    try:
        run(bridge, a.id, speed=max(0, min(120, a.speed)))
    finally:
        bridge.close()


if __name__ == "__main__":
    main()
