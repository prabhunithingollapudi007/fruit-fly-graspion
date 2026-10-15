"""Clocks: the scheduler only uses now() and sleep(), so the same code runs on the rig or in the simulator."""
from __future__ import annotations

import time


class RealClock:
    def now(self) -> float:
        return time.perf_counter()

    def sleep(self, dt: float):
        if dt > 0:
            time.sleep(dt)


class SimClock:
    """Virtual time: sleep() advances the simulated world instead of waiting."""

    def __init__(self, world):
        self.world = world

    def now(self) -> float:
        return self.world.t

    def sleep(self, dt: float):
        if dt > 0:
            self.world.advance(self.world.t + dt)
