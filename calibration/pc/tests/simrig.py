"""Helpers: run a scheduler session against the simulated rig; fake blaster and bot serial ports."""
import contextlib
import io
import types
from pathlib import Path

from graspion_calib import protocol as P
from graspion_calib import sim
from graspion_calib.clock import SimClock
from graspion_calib.geometry import Arena
from graspion_calib.poses import TrackerCSV
from graspion_calib.scheduler import NudgeConfig, SafetyConfig, Session, SessionConfig

BOT = 90


def abort_on_pause(msg):
    abort_on_pause.messages.append(msg)
    return "abort"


abort_on_pause.messages = []


def enter_key():
    """A getkey that presses Enter at once: every operator pause runs the segment from where the bot stands."""
    return "\r"


class ScriptedKeys:
    """getkey for the operator pause in tests: one item per poll, a key, None (nothing pressed) or a callable run
    at that poll (the operator's hand, e.g. a teleport of the simulated bot). `items` may be a callable that
    builds the list at the first poll, when the simulated world exists. An exhausted script fails the test
    instead of letting the pause spin forever."""

    def __init__(self, items):
        self.items = items

    def __call__(self):
        if callable(self.items):
            self.items = list(self.items())
        if not self.items:
            raise AssertionError("the operator pause polled for a key after the script ended")
        k = self.items.pop(0)
        if callable(k):
            k()
            return None
        return k


def home_placer(world, bot=BOT, home=(0.5, 0.5)):
    """A getkey that acts like the operator of a long run: at every pause it sets the simulated bot down at the
    arena centre with a fresh heading, then presses Enter."""
    rng = world.rng

    def getkey():
        plant = world.plants[bot]
        plant.x, plant.y, plant.th = home[0], home[1], float(rng.uniform(-3.1, 3.1))
        return "\r"
    return getkey


def run_session(out, specs, params=None, start=(0.45, 0.5, 0.3), arena=None, seed=0, bridge_factory=None,
                on_pause=abort_on_pause, geometry=None, safety=None, nudge=None, getkey=enter_key, quiet=True,
                place_home=False):
    """geometry: what the PC believes about the marker (default: the simulated bot's true geometry).
    quiet: swallow the scheduler's console output (the operator pause prints a line per segment).
    place_home: the operator puts the bot back at the centre before every segment (home_placer)."""
    out = Path(out)
    out.mkdir(parents=True, exist_ok=True)
    p = params or sim.SimParams()
    world = sim.SimWorld({BOT: (p, start)}, seed=seed, tracker_csv=TrackerCSV(out / "tracker.csv"))
    if place_home:
        getkey = home_placer(world)
    bridge = (bridge_factory or sim.SimBridge)(world)
    cfg = SessionConfig(BOT, geometry or p.geometry, arena or Arena(0.0, 1.0, 0.0, 1.0, 0.08), str(out),
                        safety=safety or SafetyConfig(), nudge=nudge or NudgeConfig())
    try:
        with contextlib.redirect_stdout(io.StringIO()) if quiet else contextlib.nullcontext():
            sess = Session(cfg, bridge, world.poses, SimClock(world), getkey, on_pause=on_pause,
                           extra_meta={"pwm_hz": "sim"})
            counts = sess.run(specs)
    finally:
        world.csv.close()
    return world, counts


class FakeBlasterSerial:
    """The serial side of the blaster sketch (bot_blaster.ino) on the simulated IR channel, for
    graspion_calib.bridge.SerialBridge(ser=...).

    A request is handled as soon as its line is complete (the frame goes out on the simulated channel, the reply
    is queued); a CAN byte drops the partial line. interrupt_at=n raises KeyboardInterrupt in the n-th readline
    after consuming `consume` bytes of the queued reply: 0 is a Ctrl-C before the reply arrived, more is one in
    the middle of it. proto: what 'V' reports (another value impersonates a build of another protocol version).

    It is a Python model of the sketch's line handling, kept in step with it by hand: tests against it pin the
    PC side and the intended blaster behaviour, not the C code, which is checked only by compiling it.
    """
    LINE_MAX = 47                                    # bot_blaster.ino: char line[48]
    CANCEL = b"\x18"

    def __init__(self, world, interrupt_at=None, consume=0, proto=P.PROTO_VERSION):
        self.sim = sim.SimBridge(world)
        self.rx, self._line, self._drop = b"", b"", False
        self.sent = []                               # every raw frame handed to the IR channel
        self.lines = []                              # every complete request line, as handled
        self.interrupt_at, self.consume, self.n_read = interrupt_at, consume, 0
        self.proto = proto
        self.timeout = 1.0
        self.closed = False

    def _reply(self, text):
        self.rx += (text + "\r\n").encode("ascii")

    def _send(self, raw):
        rec = self.sim.send(raw, check=False)
        self.sent.append(raw)
        self._reply(f"K {raw:08X} {rec.blaster_t0_ms} {rec.blaster_t1_ms}")

    def _handle(self, s):
        self.lines.append(s)
        if not s:
            return
        cmd, arg = s[0], s[1:].strip()
        try:
            if cmd in "Ff":
                raw = int(arg, 16)
                if cmd == "F" and P.check(raw) != P.OK:
                    return self._reply("E crc")
                return self._send(raw)
            if cmd == "G":
                ms = int(arg)
                if not 100 <= ms <= 2000:
                    return self._reply("E range")
                self.sim.set_period(ms)
                return self._reply(f"K G {ms}")
        except ValueError:
            return self._reply("E parse")
        if cmd == "V":
            return self._reply(f"I sim_blaster proto={self.proto} pin=6 period_ms={round(self.sim.period_s * 1000)}")
        if cmd == "P":
            return self._reply("K P 0")
        self._reply("E ?")

    def write(self, data):
        for c in data:
            c = bytes([c])
            if c == b"\n":
                if self._drop:
                    self._reply("E long")
                else:
                    self._handle(self._line.decode("ascii"))
                self._line, self._drop = b"", False
            elif c == self.CANCEL:
                self._line, self._drop = b"", False
            elif c != b"\r" and not self._drop:
                if len(self._line) < self.LINE_MAX:
                    self._line += c
                else:
                    self._drop = True
        return len(data)

    def readline(self):
        self.n_read += 1
        if self.n_read == self.interrupt_at:
            self.rx = self.rx[self.consume:]
            raise KeyboardInterrupt
        i = self.rx.find(b"\n")
        if i < 0:                                    # timeout: return what there is, like pyserial
            line, self.rx = self.rx, b""
            return line
        line, self.rx = self.rx[:i + 1], self.rx[i + 1:]
        return line

    def reset_input_buffer(self):
        self.rx = b""

    def close(self):
        self.closed = True


SERIAL = "0123456789ABCDEF00112233DEADBEEF"     # a USB iSerial string; its ID is P.bot_id_from_usb_serial(SERIAL)


def bot_info_line(proto=P.PROTO_VERSION, serial=SERIAL):
    """The bot's 'i' reply in calib_bot.ino's format, abridged."""
    return (f"I fw=calib_bot/3 proto={proto} id={P.bot_id_from_usb_serial(serial)} id_override=0 serial={serial} "
            "pwm_hz=732 drive_max=120 watchdog_ms=300 rcause=0x1 frames_ok=0 millis=1234")


class FakeBotSerial:
    """pyserial's Serial for a bot on USB, as __main__.read_info_serial uses it: answers 'i' with info_line.
    serial_module() is a stand-in for the pyserial module (sys.modules['serial'])."""

    def __init__(self, info_line):
        self.info_line, self.rx = info_line, b""

    def serial_module(self):
        return types.SimpleNamespace(Serial=lambda *args, **kwargs: self)

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def reset_input_buffer(self):
        self.rx = b""

    def write(self, data):
        if b"i" in data:
            self.rx += (self.info_line + "\r\n").encode("ascii")
        return len(data)

    def readline(self):
        i = self.rx.find(b"\n")
        line, self.rx = (self.rx, b"") if i < 0 else (self.rx[:i + 1], self.rx[i + 1:])
        return line
