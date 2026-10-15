"""Thread-safe store of the latest marker detections, shared by the camera thread (or the simulator) and the
scheduler, plus the tracker CSV format."""
from __future__ import annotations

import csv
import threading
from collections import deque
from dataclasses import dataclass

TRACKER_COLUMNS = ["frame", "t", "id", "x", "y", "theta", "led_b", "led_r", "side_m", "n_ref"]


@dataclass(frozen=True)
class Detection:
    frame: int
    t: float          # host clock (time.perf_counter) when the frame was read; exposure was earlier
    id: int           # ArUco marker id
    x: float          # marker centre, world metres (homography at marker height)
    y: float
    theta: float      # marker +x edge heading, world frame, rad
    led_b: float      # mean blue chroma B - (G+R)/2 at the sync-LED spot (-255..255)
    led_r: float      # mean red chroma R - (G+B)/2 at the same spot
    side_m: float     # mean marker side length in metres (sanity check against the printed size)
    n_ref: int        # reference markers visible in this frame

    def row(self):
        return [self.frame, f"{self.t:.6f}", self.id, f"{self.x:.6f}", f"{self.y:.6f}", f"{self.theta:.6f}",
                f"{self.led_b:.1f}", f"{self.led_r:.1f}", f"{self.side_m:.5f}", self.n_ref]


class PoseStore:
    def __init__(self, history=240):
        self._lock = threading.Lock()
        self._latest: dict[int, Detection] = {}
        self._hist: dict[int, deque] = {}
        self._n = history
        self.frames = 0
        self.last_frame_t = float("nan")

    def put(self, det: Detection):
        with self._lock:
            self._latest[det.id] = det
            self._hist.setdefault(det.id, deque(maxlen=self._n)).append(det)

    def frame_done(self, t):
        with self._lock:
            self.frames += 1
            self.last_frame_t = t

    def latest(self, marker_id) -> Detection | None:
        with self._lock:
            return self._latest.get(marker_id)

    def history(self, marker_id, since_t=None):
        with self._lock:
            h = list(self._hist.get(marker_id, ()))
        return h if since_t is None else [d for d in h if d.t >= since_t]


class TrackerCSV:
    """Append-only tracker CSV; flushes every `flush_every` rows so a crash loses little."""

    def __init__(self, path, flush_every=200):
        self._f = open(path, "w", newline="")
        self._w = csv.writer(self._f)
        self._w.writerow(TRACKER_COLUMNS)
        self._k = 0
        self._every = flush_every
        self._lock = threading.Lock()

    def write(self, det: Detection):
        with self._lock:
            self._w.writerow(det.row())
            self._k += 1
            if self._k % self._every == 0:
                self._f.flush()

    def close(self):
        with self._lock:
            self._f.close()
