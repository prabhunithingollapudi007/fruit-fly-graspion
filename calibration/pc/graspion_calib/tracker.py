"""Overhead ArUco tracker -> tracker.csv (+ optional raw video) and a live PoseStore for the scheduler.

UNTESTED with a real camera: checked only on synthetic rendered scenes (tests/test_tracker.py).

Geometry (FEASIBILITY.md 3.3):
  * Four reference markers on spacers AT MARKER HEIGHT (15-20 mm above the floor, the height of the bot
    caps), at surveyed world positions. The pixel -> metre homography is fitted to their centres, so it is
    exact in the marker plane and has no parallax error for the bot markers.
  * The homography is fitted once from the median of the first `ref_frames` frames (no per-frame jitter) and
    then watched: a reference drifting more than `ref_tol_m` raises a warning (bumped rig).
  * With exactly 4 references the fit is an exact solve, so their reprojection RMS is 0 whatever the survey
    says (reproj_rms_m is NaN then). The survey check is the shape of each reference marker in the fitted frame
    (ref_check, reference_summary()): side length against the other references, aspect and diagonals, and the
    median side against the printed size (ref_size_m). A mistyped or badly surveyed centre warps the fitted
    frame and distorts the markers. Gross errors only: the tolerance is max(5 %, 2 px / marker side), e.g. a
    0.1 m typo shows with 60 mm references, a 10 mm survey error does not. With 5 or more references
    reproj_rms_m is a real residual.
  * Right-handed world frame (z up, towards the camera): the reference coordinates must not be mirrored. This
    is checked from the corner order of the reference markers (ArUco corners are clockwise on the printed
    marker), and the tracker refuses to start on a mirrored layout, which would flip the sign of omega.
  * Bot markers: print and mount each one with its top edge towards the bot's front; marker heading = the
    marker's "up" direction. Residual misalignment and the offset to the body reference point go into the rig
    config (bots.<id>.marker_yaw / marker_offset) and are applied in the scheduler and the analysis.
  * Sync LED: the mean blue and red chroma (B - (G+R)/2, R - (G+B)/2) in a small patch at the LED position
    (bots.<id>.led_offset, metres in the marker frame: forward, left) are logged per frame. Chroma stays near 0
    on grey or white surroundings, so a flash shows up whatever the background brightness. The LED must sit
    outside the marker and its white quiet zone. The patch is located in the raw (distorted) image: the
    world point is mapped through H^-1 to undistorted pixels and then re-distorted.
  * LED height: the LED sits on the PCB, below the marker cap. Seen through the marker-plane homography it
    appears pulled towards the camera nadir by r * dh / (Hc - dh) (~5 mm, several pixels, 0.5 m off-centre with
    dh = 10 mm, Hc = 1 m), enough to miss the 7 px patch near the walls. Set bots.<id>.led_height (LED height
    minus marker height, negative below) and the tracker projects the 3-D LED point with the camera pose
    recovered from H and the intrinsics K, or, without K, scales about the image-centre point by Hc / (Hc - dh)
    with camera_height_m (camera above the marker plane).
  * Mono cameras (2-D frames, or B = G = R): chroma is identically 0 there, so no flash would ever be found.
    The tracker detects this when it fits the homography and logs the patch *brightness* in led_b instead
    (led_mode 'luma'; led_r = NaN, so a red low-battery flash cannot be told from a sync flash). Check the rise
    in tracker.csv; a colour camera is still preferred.
Use a fixed short exposure (<= 2-5 ms) and flicker-free DC lighting.
"""
from __future__ import annotations

import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np

from .poses import Detection, PoseStore, TrackerCSV

REF_WARN_EVERY = 300                        # frames between repeated "reference off" warnings (~5 s at 60 fps)


def make_detector(dictionary="DICT_4X4_50"):
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary)), params)


class Undistorter:
    def __init__(self, K=None, dist=None):
        self.K = None if K is None else np.asarray(K, float)
        self.dist = None if dist is None else np.asarray(dist, float).ravel()
        self.active = self.K is not None and self.dist is not None and np.any(self.dist != 0)

    @classmethod
    def from_npz(cls, path):
        d = np.load(path)
        return cls(d["K"], d["dist"])

    def undistort(self, pts):
        pts = np.asarray(pts, np.float64).reshape(-1, 2)
        if not self.active:
            return pts
        return cv2.undistortPoints(pts.reshape(-1, 1, 2), self.K, self.dist, P=self.K).reshape(-1, 2)

    def distort(self, pts):
        """Undistorted pixel coordinates -> raw image pixel coordinates."""
        pts = np.asarray(pts, np.float64).reshape(-1, 2)
        if not self.active:
            return pts
        n = cv2.undistortPoints(pts.reshape(-1, 1, 2), self.K, np.zeros(5), P=None).reshape(-1, 2)  # K^-1
        obj = np.c_[n, np.ones(len(n))].astype(np.float64)
        img, _ = cv2.projectPoints(obj, np.zeros(3), np.zeros(3), self.K, self.dist)
        return img.reshape(-1, 2)


def signed_area(poly):
    x, y = poly[:, 0], poly[:, 1]
    return 0.5 * float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


class FrameHandednessError(ValueError):
    pass


@dataclass
class BotMarker:
    marker_id: int
    led_offset: tuple = (0.012, 0.0)    # sync LED in the marker frame (forward, left) [m]; measure per bot
    size_m: float = 0.02
    led_height: float = 0.0             # LED height minus marker height [m]; negative when the LED sits lower


def led_mode_of(img):
    """'luma' for a mono stream (2-D frames, or B = G = R to within rounding nearly everywhere: what a grey
    stream looks like after the driver's conversion to BGR), else 'chroma'. A real colour sensor never gives
    equal channels across the frame (noise, demosaicing). Synthetic grey renders count as mono: pass
    led_mode='chroma' to the Tracker for those."""
    if img.ndim == 2 or img.shape[2] == 1:
        return "luma"
    sub = img[::8, ::8].astype(np.int16)
    diff = np.maximum(np.abs(sub[..., 0] - sub[..., 1]), np.abs(sub[..., 1] - sub[..., 2]))
    return "luma" if np.mean(diff > 1) < 1e-3 else "chroma"


class Tracker:
    def __init__(self, ref_world: dict, bot_markers: dict | None = None, undistorter: Undistorter | None = None,
                 detector=None, ref_frames=30, ref_tol_m=0.003, led_patch_px=3, size_tol=0.25, ref_size_m=None,
                 led_mode="auto", camera_height_m=None, ref_shape_tol=0.05):
        """led_mode: 'auto' (decided on the frame that completes the homography fit), 'chroma' or 'luma'.
        camera_height_m: camera above the marker plane, for the LED-height correction when there are no
        intrinsics. ref_shape_tol: relative side / squareness error of a reference marker that raises a warning."""
        self.ref_world = {int(k): tuple(v) for k, v in ref_world.items()}
        if len(self.ref_world) < 4:
            raise ValueError("need at least 4 reference markers")
        self.bots = bot_markers or {}
        self.und = undistorter or Undistorter()
        self.det = detector or make_detector()
        self.ref_frames, self.ref_tol, self.patch, self.size_tol = ref_frames, ref_tol_m, led_patch_px, size_tol
        self.ref_size_m = ref_size_m
        self.ref_shape_tol = ref_shape_tol
        self.led_mode = led_mode
        self.camera_height_m = camera_height_m
        self.H = self.Hinv = None           # published last in _fit_h: other threads wait for H (see __main__)
        self._H = self._Hinv = None         # used while the fit is still being checked
        self._pose = None                   # (K, R, t) of the camera, world (marker plane z = 0) -> camera
        self._nadir = None
        self._led_height_warned = False
        self._buf = []
        self.warnings = []
        self.ref_off_frames = {}            # reference id -> frames it was off by more than ref_tol
        self.reproj_rms_m = float("nan")
        self.ref_check = {}

    # ------------------------------------------------------------------ homography
    def _fit_h(self, img=None):
        ids = sorted(self.ref_world)
        src = np.median(np.array([[b[i][0] for i in ids] for b in self._buf]), axis=0)
        dst = np.array([self.ref_world[i] for i in ids], float)
        H, _ = cv2.findHomography(src, dst, 0)
        if H is None:
            raise RuntimeError("homography fit failed")
        corners = np.median(np.array([[b[i][1] for i in ids] for b in self._buf]), axis=0)   # (n, 4, 2)
        areas = [signed_area(cv2.perspectiveTransform(c.reshape(-1, 1, 2), H).reshape(4, 2)) for c in corners]
        if np.median(areas) > 0:
            raise FrameHandednessError(
                "reference-marker coordinates form a mirrored (left-handed) frame: omega would change sign. "
                "Swap x and y or negate one axis in reference_markers.")
        self._H, self._Hinv = H, np.linalg.inv(H)
        proj =cv2.perspectiveTransform(src.reshape(-1, 1, 2), H).reshape(-1, 2)
        # 4 points: exact solve, the residual is 0 by construction and says nothing about the survey
        self.reproj_rms_m = (float(np.sqrt(np.mean(np.sum((proj - dst) ** 2, axis=1)))) if len(ids) > 4
                             else float("nan"))
        w4s = [cv2.perspectiveTransform(c.reshape(-1, 1, 2), H).reshape(4, 2) for c in corners]
        sides = [np.mean(np.linalg.norm(np.roll(c4, -1, 0) - c4, axis=1)) for c4 in w4s]
        self.ref_side_m = float(np.median(sides))
        sides_px = [float(np.mean(np.linalg.norm(np.roll(c, -1, 0) - c, axis=1))) for c in corners]
        for i, c4, side_px, s in zip(ids, w4s, sides_px, sides):
            e = np.linalg.norm(np.roll(c4, -1, 0) - c4, axis=1)
            d1, d2 = np.linalg.norm(c4[2] - c4[0]), np.linalg.norm(c4[3] - c4[1])
            chk = {"side_m": float(s), "side_err": float(s / self.ref_side_m - 1),   # vs the other references
                   "aspect_err": float(abs((e[0] + e[2]) / (e[1] + e[3]) - 1)),
                   "diag_err": float(abs(d1 - d2) / (0.5 * (d1 + d2))),
                   "tol": float(max(self.ref_shape_tol, 2.0 / side_px))}             # ~2 px of corner error
            self.ref_check[i] = chk
        bad = [i for i, c in self.ref_check.items()
               if max(abs(c["side_err"]), c["aspect_err"], c["diag_err"]) > c["tol"]]
        if bad:                                 # one wrong centre warps the whole fit, so name no single culprit
            self.warnings.append((None, f"reference markers {bad} are distorted in the fitted frame "
                                        f"({self.reference_summary()}): a surveyed reference position is probably "
                                        "wrong (mistyped, swapped ids, or not at marker height)"))
        if self.ref_size_m:
            self.ref_size_err = float(self.ref_side_m / self.ref_size_m - 1)
            if abs(self.ref_size_err) > max(0.1, 3.0 / float(np.median(sides_px))):
                self.warnings.append((None, f"reference markers measure {self.ref_side_m * 1000:.1f} mm in the fitted "
                                            f"frame, not {self.ref_size_m * 1000:.1f} mm: check reference_size_m and "
                                            "the units of the surveyed positions"))
        if self.und.K is not None:
            self._pose = self._camera_pose()
        if img is not None:
            h, w = img.shape[:2]
            centre = self.und.undistort(np.array([[0.5 * (w - 1), 0.5 * (h - 1)]]))
            self._nadir = self.to_world(centre)[0]
            if self.led_mode == "auto":
                self.led_mode = led_mode_of(img)
                if self.led_mode == "luma":
                    self.warnings.append((None, "mono camera: sync LED logged as patch brightness in led_b "
                                                "(led_r = NaN; red low-battery flashes look like sync flashes)"))
        self.H, self.Hinv = self._H, self._Hinv  # publish last: the fit, its checks and led_mode are complete

    def _camera_pose(self):
        """Camera pose from H and K: H^-1 ~ K [r1 r2 t] (world z towards the camera in a right-handed frame)."""
        K = np.asarray(self.und.K, float)
        M = np.linalg.inv(K) @ self._Hinv
        M = M / (0.5 * (np.linalg.norm(M[:, 0]) + np.linalg.norm(M[:, 1])))
        if M[2, 2] < 0:                                     # the world origin lies in front of the camera
            M = -M
        U, _, Vt = np.linalg.svd(np.c_[M[:, 0], M[:, 1], np.cross(M[:, 0], M[:, 1])])
        R, t = U @ Vt, M[:, 2]
        if (-R.T @ t)[2] <= 0:                              # camera must be above the marker plane
            self.warnings.append((None, "camera pose from H and K is not above the plane: LED height ignored"))
            return None
        return K, R, t

    def reference_summary(self):
        """One line on the reference fit, for the operator (the 4-point RMS is not a check, see the module doc)."""
        if not self.ref_check:
            return "homography not fitted"
        worst = max(self.ref_check.items(), key=lambda kv: max(abs(kv[1]["side_err"]), kv[1]["aspect_err"],
                                                                  kv[1]["diag_err"]) / kv[1]["tol"])
        rms = ("exact 4-point fit, no residual" if not np.isfinite(self.reproj_rms_m)
               else f"reprojection RMS {self.reproj_rms_m * 1000:.2f} mm")
        c = worst[1]
        size = (f"; side {self.ref_side_m * 1000:.1f} mm vs reference_size_m {self.ref_size_m * 1000:.1f} mm"
                if self.ref_size_m else "")
        return (f"{rms}; reference marker shapes: worst id {worst[0]} side {c['side_err']:+.1%} vs the others, "
                f"aspect {c['aspect_err']:.1%}, diagonals {c['diag_err']:.1%} (tolerance {c['tol']:.1%}){size}")

    def to_world(self, pts_und):
        return cv2.perspectiveTransform(np.asarray(pts_und, np.float64).reshape(-1, 1, 2), self._H).reshape(-1, 2)

    def led_pixel_undistorted(self, pw, dz=0.0):
        """Undistorted pixel of the world point pw (marker-plane x, y) raised by dz above the marker plane."""
        pw = np.asarray(pw, float)
        if dz and self._pose is not None:
            K, R, t = self._pose
            pc = K @ (R @ np.array([pw[0], pw[1], dz]) + t)
            return (pc[:2] / pc[2]).reshape(1, 2)
        if dz and self.camera_height_m and self._nadir is not None:
            hc = float(self.camera_height_m)            # ray through the raised point meets the plane here
            pw = self._nadir + (pw - self._nadir) * hc / (hc - dz)
        elif dz and not self._led_height_warned:
            self._led_height_warned = True
            self.warnings.append((None, "led_height ignored: needs camera intrinsics or camera_height_m"))
        return cv2.perspectiveTransform(pw.reshape(1, 1, 2), self._Hinv).reshape(1, 2)

    # ------------------------------------------------------------------ per frame
    def process(self, img_bgr, t, frame_idx):
        gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY) if img_bgr.ndim == 3 else img_bgr
        corners, ids, _ = self.det.detectMarkers(gray)
        if ids is None:
            return []
        und = {int(i): self.und.undistort(c.reshape(4, 2)) for i, c in zip(ids.ravel(), corners)}
        seen_ref = [i for i in self.ref_world if i in und]
        if self.H is None:
            if len(seen_ref) == len(self.ref_world):
                self._buf.append({i: (und[i].mean(axis=0), und[i]) for i in self.ref_world})
            if len(self._buf) >= self.ref_frames:
                self._fit_h(img_bgr)
            return []
        for i in seen_ref:                                      # watch the references
            err = np.linalg.norm(self.to_world(und[i].mean(axis=0))[0] - np.array(self.ref_world[i]))
            if err > self.ref_tol:                              # warn on the first frame, then every REF_WARN_EVERY
                n = self.ref_off_frames[i] = self.ref_off_frames.get(i, 0) + 1
                if n % REF_WARN_EVERY == 1 or REF_WARN_EVERY == 1:
                    self.warnings.append((t, f"reference {i} off by {err * 1000:.1f} mm (rig bumped?; "
                                             f"{n} frames so far)"))
        out = []
        for i, c in und.items():
            if i in self.ref_world:
                continue
            w4 = self.to_world(c)
            centre = w4.mean(axis=0)
            up = 0.5 * (w4[0] + w4[1]) - 0.5 * (w4[3] + w4[2])        # bottom edge -> top edge
            right = 0.5 * (w4[1] + w4[2]) - 0.5 * (w4[0] + w4[3])     # left edge -> right edge
            hv = up / (np.linalg.norm(up) + 1e-12) + np.array([-right[1], right[0]]) / (np.linalg.norm(right) + 1e-12)
            theta = float(np.arctan2(hv[1], hv[0]))                   # both edge pairs: ~sqrt(2) less noise
            side = float(np.mean(np.linalg.norm(np.roll(w4, -1, 0) - w4, axis=1)))
            bm = self.bots.get(i)
            nominal = bm.size_m if bm else self.ref_size_m
            if nominal and abs(side / nominal - 1) > self.size_tol:
                continue                                        # misdetection or wrong plane
            led_b = led_r = float("nan")
            if bm is not None:
                u = up / (np.linalg.norm(up) + 1e-12)
                left = np.array([-u[1], u[0]])
                pw = centre + bm.led_offset[0] * u + bm.led_offset[1] * left
                pu = self.led_pixel_undistorted(pw, getattr(bm, "led_height", 0.0))
                px, py = self.und.distort(pu)[0]
                x0, y0, r = int(round(px)), int(round(py)), self.patch
                patch = img_bgr[max(y0 - r, 0):y0 + r + 1, max(x0 - r, 0):x0 + r + 1]
                if patch.size and (self.led_mode == "luma" or img_bgr.ndim == 2):
                    led_b = float(np.mean(patch.astype(np.float32)))    # brightness: the only signal in mono
                elif patch.size:
                    b, g, r_ = (patch[..., k].astype(np.float32) for k in range(3))
                    led_b = float(np.mean(b - 0.5 * (g + r_)))       # chroma: ~0 on grey/white backgrounds
                    led_r = float(np.mean(r_ - 0.5 * (g + b)))
            out.append(Detection(frame_idx, t, i, float(centre[0]), float(centre[1]), theta, led_b, led_r, side,
                                 len(seen_ref)))
        return out


def open_camera(index=0, width=1280, height=720, fps=60, exposure=None, backend=None):
    """UVC camera with fixed settings. Exposure values are driver-specific (DirectShow: log2 seconds, e.g. -9)."""
    be = backend if backend is not None else (cv2.CAP_DSHOW if hasattr(cv2, "CAP_DSHOW") else cv2.CAP_ANY)
    cap = cv2.VideoCapture(index, be)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    if exposure is not None:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)               # manual on many UVC drivers
        cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
    if not cap.isOpened():
        raise RuntimeError(f"camera {index} did not open")
    return cap


class CameraThread(threading.Thread):
    """Grab -> track -> PoseStore + CSV (+ raw video for offline re-tracking). Timestamps: perf_counter at read."""

    def __init__(self, cap, tracker: Tracker, poses: PoseStore, csv_path, video_path=None, fps=60.0):
        super().__init__(daemon=True)
        self.cap, self.tracker, self.poses = cap, tracker, poses
        self.csv = TrackerCSV(csv_path)
        self.video_path, self.fps = video_path, fps
        self._stop = threading.Event()
        self.error = None
        self.frame_idx = 0

    def run(self):
        writer = None
        try:
            while not self._stop.is_set():
                ok, img = self.cap.read()
                t = time.perf_counter()
                if not ok:
                    self.error = "camera read failed"
                    break
                if self.video_path and writer is None:
                    h, w = img.shape[:2]
                    writer = cv2.VideoWriter(str(self.video_path), cv2.VideoWriter_fourcc(*"MJPG"), self.fps, (w, h))
                if writer is not None:
                    writer.write(img)
                for d in self.tracker.process(img, t, self.frame_idx):
                    self.poses.put(d)
                    self.csv.write(d)
                self.poses.frame_done(t)
                self.frame_idx += 1
        except Exception as e:                                   # surfaced to the scheduler as marker loss
            self.error = repr(e)
        finally:
            if writer is not None:
                writer.release()
            self.csv.close()

    def stop(self):
        self._stop.set()


def track_video(path, tracker: Tracker, csv_path, fps=None, t0=0.0):
    """Offline re-tracking of a recorded video; t = t0 + frame / fps."""
    cap = cv2.VideoCapture(str(path))
    fps = fps or cap.get(cv2.CAP_PROP_FPS) or 60.0
    out = TrackerCSV(csv_path)
    k = 0
    while True:
        ok, img = cap.read()
        if not ok:
            break
        for d in tracker.process(img, t0 + k / fps, k):
            out.write(d)
        k += 1
    out.close()
    cap.release()
    return k


# ------------------------------------------------------------------------------------------- synthetic scenes
def marker_corners_world(x, y, theta, size):
    """World corners (TL, TR, BR, BL) of a marker centred at (x, y) whose 'up' points along theta."""
    u = np.array([np.cos(theta), np.sin(theta)])
    r = np.array([np.sin(theta), -np.cos(theta)])
    c = np.array([x, y])
    h = 0.5 * size
    return np.array([c + h * (-r + u), c + h * (r + u), c + h * (r - u), c + h * (-r - u)])


def render_scene(H_world_to_px, markers, img_size=(1280, 720), dictionary="DICT_4X4_50", px_per_cell=12,
                 leds=(), background=235):
    """Render flat ArUco markers (id, x, y, theta, size_m) seen through a pinhole homography (no distortion).

    leds: (x, y, (b, g, r), radius_px) world spots drawn after the markers. For tests only."""
    w, h = img_size
    img = np.full((h, w, 3), background, np.uint8)
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
    for mid, x, y, theta, size in markers:
        n = 6 * px_per_cell                                     # 4x4 bits + 1-cell black border
        mk = cv2.aruco.generateImageMarker(d, int(mid), n)
        q = px_per_cell * 2                                     # white quiet zone of 2 cells
        tile = cv2.copyMakeBorder(mk, q, q, q, q, cv2.BORDER_CONSTANT, value=255)
        tile = cv2.cvtColor(tile, cv2.COLOR_GRAY2BGR)
        full = size * (n + 2 * q) / n
        cw = marker_corners_world(x, y, theta, full)
        dst = cv2.perspectiveTransform(cw.reshape(-1, 1, 2), H_world_to_px).reshape(4, 2).astype(np.float32)
        m = tile.shape[0]
        src = np.array([[0, 0], [m - 1, 0], [m - 1, m - 1], [0, m - 1]], np.float32)
        M = cv2.getPerspectiveTransform(src, dst)
        warped = cv2.warpPerspective(tile, M, (w, h), flags=cv2.INTER_LINEAR, borderValue=(0, 0, 0))
        mask = cv2.warpPerspective(np.full((m, m), 255, np.uint8), M, (w, h), flags=cv2.INTER_NEAREST)
        img[mask > 0] = warped[mask > 0]
    for x, y, bgr, rad in leds:
        p = cv2.perspectiveTransform(np.array([[[x, y]]], np.float64), H_world_to_px).reshape(2)
        cv2.circle(img, (int(round(p[0])), int(round(p[1]))), int(rad), tuple(int(c) for c in bgr), -1)
    return img
