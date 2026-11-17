"""Overhead ArUco tracker -> tracker.csv (+ optional raw video) and a live PoseStore for the scheduler.

UNTESTED with a real camera.

Geometry (FEASIBILITY.md 3.3):
  * Four or more reference markers at surveyed world positions, all in one plane (the reference plane: flat
    on the arena floor, or on spacers). The pixel -> metre homography is fitted to their centres, so it is exact
    in that plane.
  * Bot markers above the reference plane (bots.<id>.marker_height: the tag face above the reference plane,
    the robot's real tag height) are seen through the homography pushed away from the camera nadir by
    r * h / (Hc - h): 22.9 mm under a camera 0.94 m up is +2.5 % on every position about the nadir, and on every
    speed. The tracker undoes it per corner, p_h = nadir + (p0 - nadir) * (1 - h / Hc), with the camera centre
    from the pose recovered from H and the intrinsics K, or, without K, with camera_height_m (camera above the
    reference plane) and the image centre as the nadir (off by h / Hc of the nadir's offset: a constant shift).
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
  * LED height: the LED sits on the PCB, below the marker cap. Seen through the reference-plane homography a
    raised point appears pushed away from the camera nadir by r * dz / (Hc - dz), enough to miss the 7 px patch
    near the walls. Set bots.<id>.led_height (LED height minus marker height, negative below); the tracker
    projects the 3-D LED point (marker_height + led_height above the reference plane) with the camera pose
    recovered from H and K, or, without K, scales about the image-centre point by Hc / (Hc - dz) with
    camera_height_m.
  * No visible LED (bots.<id>.led_offset null, e.g. the LED under the marker cap): led_b and led_r are NaN.
  * Mono cameras (2-D frames, or B = G = R): chroma is identically 0 there, so no flash would ever be found.
    The tracker detects this when it fits the homography and logs the patch *brightness* in led_b instead
    (led_mode 'luma'; led_r = NaN, so a red low-battery flash cannot be told from a sync flash). Check the rise
    in tracker.csv; a colour camera is still preferred.
  * Intrinsics: an OpenCV .npz (K, dist) or a JSON file with K, dist and optionally size [width, height]
    (graspion_tracking's intrinsics.json). With a size, frames of any other size are refused: a lens model
    fitted in a camera window only fits that window.
Use a fixed exposure and flicker-free DC lighting.
"""
from __future__ import annotations

import json
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
    def __init__(self, K=None, dist=None, size=None, offset=(0, 0)):
        """size: (width, height) of the frames the lens model is in, or None (not checked). offset: where those
        frames start on the sensor (the camera window's OffsetX, OffsetY), so that the model can be moved to
        another window (for_window)."""
        self.K = None if K is None else np.asarray(K, float)
        self.dist = None if dist is None else np.asarray(dist, float).ravel()
        self.size = None if size is None else (int(size[0]), int(size[1]))
        self.offset = (int(offset[0]), int(offset[1]))
        self.active = self.K is not None and self.dist is not None and np.any(self.dist != 0)

    def for_window(self, offset_x, offset_y, width, height):
        """The same lens in the pixels of another camera window: a window only crops the sensor, so the
        principal point moves by the change of offset and nothing else does. Outside the window the model was
        fitted in, the distortion is extrapolated."""
        if self.K is None:
            return self
        K = self.K.copy()
        K[0, 2] += self.offset[0] - int(offset_x)
        K[1, 2] += self.offset[1] - int(offset_y)
        return Undistorter(K, self.dist, (width, height), (offset_x, offset_y))

    @classmethod
    def from_npz(cls, path):
        d = np.load(path)
        return cls(d["K"], d["dist"], d["size"] if "size" in d else None)

    @classmethod
    def from_file(cls, path):
        """An OpenCV .npz (K, dist[, size]) or a JSON file with the same keys (graspion_tracking's
        intrinsics.json: K 3x3, dist k1 k2 p1 p2 k3, size [width, height] in the camera window's pixels,
        meta.window.offset_x / offset_y: where that window starts on the sensor, 0 if absent)."""
        if str(path).lower().endswith(".npz"):
            return cls.from_npz(path)
        with open(path) as f:
            d = json.load(f)
        win = (d.get("meta") or {}).get("window") or {}
        return cls(d["K"], d["dist"], d.get("size"), (win.get("offset_x", 0), win.get("offset_y", 0)))

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


class FrameSizeError(ValueError):
    pass


@dataclass
class BotMarker:
    marker_id: int
    led_offset: tuple | None = (0.012, 0.0)  # sync LED in the marker frame (forward, left) [m]; None: no LED
    size_m: float = 0.02
    led_height: float = 0.0             # LED height minus marker height [m]; negative when the LED sits lower
    height: float = 0.0                 # tag face above the reference-marker plane [m]: the robot's tag height


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
        self._nadir = None                  # world (x, y) under the camera centre (image centre without K)
        self._cam_z = None                  # camera centre above the reference plane [m]; None if unknown
        self._led_height_warned = False
        self._height_warned = False
        self._buf = []
        self.warnings = []
        self.ref_off_frames = {}            # reference id -> frames it was off by more than ref_tol
        self.reproj_rms_m = float("nan")
        self.ref_check = {}

    # ------------------------------------------------------------------ homography
    def check_frame_size(self, img):
        """Refuse frames of another size than the lens model's (a camera window that was not set)."""
        if self.und.size is not None:
            h, w = img.shape[:2]
            if (w, h) != self.und.size:
                raise FrameSizeError(f"frames are {w} x {h} px but the intrinsics were fitted in "
                                     f"{self.und.size[0]} x {self.und.size[1]}: set the camera window first")

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
        if self._pose is not None:
            _, R, t = self._pose
            C = -R.T @ t
            self._nadir, self._cam_z = C[:2].copy(), float(C[2])
        if img is not None:
            h, w = img.shape[:2]
            if self._pose is None:
                centre = self.und.undistort(np.array([[0.5 * (w - 1), 0.5 * (h - 1)]]))
                self._nadir = self.to_world(centre)[0]
                self._cam_z = float(self.camera_height_m) if self.camera_height_m else None
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
            self.warnings.append((None, "camera pose from H and K is not above the plane: marker and LED heights "
                                        "fall back to camera_height_m"))
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

    def at_height(self, pw, h):
        """Reference-plane points pw (where the camera rays meet that plane) -> the points h above the plane on
        the same rays: nadir + (pw - nadir) * (1 - h / Hc)."""
        pw = np.asarray(pw, float)
        if not h:
            return pw
        if self._cam_z is None or self._nadir is None:
            if not self._height_warned:
                self._height_warned = True
                self.warnings.append((None, "marker_height ignored: needs camera intrinsics or camera_height_m"))
            return pw
        return self._nadir + (pw - self._nadir) * (1.0 - h / self._cam_z)

    def led_pixel_undistorted(self, pw, dz=0.0):
        """Undistorted pixel of the world point pw (marker-plane x, y) raised by dz above the marker plane."""
        pw = np.asarray(pw, float)
        if dz and self._pose is not None:
            K, R, t = self._pose
            pc = K @ (R @ np.array([pw[0], pw[1], dz]) + t)
            return (pc[:2] / pc[2]).reshape(1, 2)
        if dz and self._cam_z and self._nadir is not None:
            hc = self._cam_z                            # ray through the raised point meets the plane here
            pw = self._nadir + (pw - self._nadir) * hc / (hc - dz)
        elif dz and not self._led_height_warned:
            self._led_height_warned = True
            self.warnings.append((None, "led_height ignored: needs camera intrinsics or camera_height_m"))
        return cv2.perspectiveTransform(pw.reshape(1, 1, 2), self._Hinv).reshape(1, 2)

    # ------------------------------------------------------------------ per frame
    def process(self, img_bgr, t, frame_idx):
        if self.H is None:
            self.check_frame_size(img_bgr)
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
            bm = self.bots.get(i)
            h = float(getattr(bm, "height", 0.0) or 0.0) if bm else 0.0
            w4 = self.at_height(self.to_world(c), h)
            centre = w4.mean(axis=0)
            up = 0.5 * (w4[0] + w4[1]) - 0.5 * (w4[3] + w4[2])        # bottom edge -> top edge
            right = 0.5 * (w4[1] + w4[2]) - 0.5 * (w4[0] + w4[3])     # left edge -> right edge
            hv = up / (np.linalg.norm(up) + 1e-12) + np.array([-right[1], right[0]]) / (np.linalg.norm(right) + 1e-12)
            theta = float(np.arctan2(hv[1], hv[0]))                   # both edge pairs: ~sqrt(2) less noise
            side = float(np.mean(np.linalg.norm(np.roll(w4, -1, 0) - w4, axis=1)))
            nominal = bm.size_m if bm else self.ref_size_m
            if nominal and abs(side / nominal - 1) > self.size_tol:
                continue                                        # misdetection or wrong plane
            led_b = led_r = float("nan")
            if bm is not None and bm.led_offset is not None:
                u = up / (np.linalg.norm(up) + 1e-12)
                left = np.array([-u[1], u[0]])
                pw = centre + bm.led_offset[0] * u + bm.led_offset[1] * left
                pu = self.led_pixel_undistorted(pw, h + (getattr(bm, "led_height", 0.0) or 0.0))
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
    """UVC camera with fixed settings. Exposure values are driver-specific (DirectShow: log2 seconds, e.g. -9).
    A Basler USB3 Vision camera (the lab's acA3088-57um) is not UVC: it opens here only through pylon's DirectShow
    driver (UNTESTED); set its exposure in pylon Viewer and pass exposure=None."""
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


class PylonCapture:
    """Basler camera through pypylon with a cv2.VideoCapture-like read() -> (ok, mono frame). UNTESTED on the
    camera: written after cam_focus/focus.py, which ran on it. It sets the camera window, Mono8 and fixed exposure
    and gain. A window without offsets is centred on the sensor, the offsets rounded down to the camera's steps
    (as graspion_tracking's camera_view.py does); settings["window"] is what the camera applied, and the lens
    model is moved to it (Undistorter.for_window). A window the camera does not take as asked is refused."""

    def __init__(self, window=None, exposure_us=None, gain_db=None, timeout_ms=1000):
        from pypylon import pylon
        self._pylon = pylon
        self.timeout_ms = int(timeout_ms)
        tl = pylon.TlFactory.GetInstance()
        if not tl.EnumerateDevices():
            raise SystemExit("no Basler camera found: is it plugged in, and not attached to WSL / the "
                             "graspion_ws_jazzy container (usbipd detach --busid <id>)? Close pylon Viewer too.")
        self.cam = pylon.InstantCamera(tl.CreateFirstDevice())
        self.cam.Open()
        c = self.cam
        c.PixelFormat.Value = "Mono8"
        c.ExposureAuto.Value = "Off"
        c.GainAuto.Value = "Off"
        if exposure_us is not None:
            c.ExposureTime.Value = float(exposure_us)
        if gain_db is not None:
            c.Gain.Value = float(gain_db)
        if window:
            w, h = int(window["width"]), int(window["height"])
            c.OffsetX.Value, c.OffsetY.Value = 0, 0               # an offset limits the size: clear it first
            c.Width.Value, c.Height.Value = w, h

            def centred(node, free):
                inc = int(node.GetInc()) if hasattr(node, "GetInc") else 1
                return max(0, free // 2 // inc * inc)
            ox = int(window["offset_x"]) if window.get("offset_x") is not None else centred(c.OffsetX, c.WidthMax.Value - w)
            oy = int(window["offset_y"]) if window.get("offset_y") is not None else centred(c.OffsetY, c.HeightMax.Value - h)
            want = (ox, oy, w, h)
            c.OffsetX.Value, c.OffsetY.Value = ox, oy
            got = (c.OffsetX.Value, c.OffsetY.Value, c.Width.Value, c.Height.Value)
            if got != want:
                c.Close()
                raise RuntimeError(f"camera window is (x, y, w, h) {got}, not {want}")
        self.settings = {"exposure_us": float(c.ExposureTime.Value), "gain_db": float(c.Gain.Value),
                         "window": [int(c.OffsetX.Value), int(c.OffsetY.Value), int(c.Width.Value),
                                    int(c.Height.Value)],
                         "fps": float(c.ResultingFrameRate.Value) if hasattr(c, "ResultingFrameRate") else None}
        c.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)

    def isOpened(self):
        return self.cam.IsGrabbing()

    def read(self):
        try:
            r = self.cam.RetrieveResult(self.timeout_ms, self._pylon.TimeoutHandling_ThrowException)
        except Exception:
            return False, None
        try:
            if not r.GrabSucceeded():
                return False, None
            return True, r.Array.copy()
        finally:
            r.Release()

    def release(self):
        try:
            self.cam.StopGrabbing()
        finally:
            self.cam.Close()


def open_camera_from_config(cam_cfg):
    """camera.backend 'pylon' (Basler through pypylon: window, exposure_us, gain_db) or the default OpenCV
    camera (index, width, height, fps, exposure)."""
    if cam_cfg.get("backend") == "pylon":
        return PylonCapture(cam_cfg.get("window"), cam_cfg.get("exposure_us"), cam_cfg.get("gain_db"))
    return open_camera(cam_cfg.get("index", 0), cam_cfg.get("width", 1280), cam_cfg.get("height", 720),
                       cam_cfg.get("fps", 60), cam_cfg.get("exposure"))


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
                    writer = cv2.VideoWriter(str(self.video_path), cv2.VideoWriter_fourcc(*"MJPG"), self.fps, (w, h),
                                             img.ndim == 3)
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
            try:
                self.cap.release()
            except Exception:
                pass

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
    """Render flat ArUco markers (id, x, y, theta, size_m[, H]) seen through a pinhole homography (no distortion);
    a marker's own H (its plane -> pixels) renders it in another plane, e.g. raised above the floor.

    leds: (x, y, (b, g, r), radius_px) world spots drawn after the markers. For tests only."""
    w, h = img_size
    img = np.full((h, w, 3), background, np.uint8)
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
    for mk in markers:
        mid, x, y, theta, size = mk[:5]
        Hm = mk[5] if len(mk) > 5 else H_world_to_px
        n = 6 * px_per_cell                                     # 4x4 bits + 1-cell black border
        mk = cv2.aruco.generateImageMarker(d, int(mid), n)
        q = px_per_cell * 2                                     # white quiet zone of 2 cells
        tile = cv2.copyMakeBorder(mk, q, q, q, q, cv2.BORDER_CONSTANT, value=255)
        tile = cv2.cvtColor(tile, cv2.COLOR_GRAY2BGR)
        full = size * (n + 2 * q) / n
        cw = marker_corners_world(x, y, theta, full)
        dst = cv2.perspectiveTransform(cw.reshape(-1, 1, 2), Hm).reshape(4, 2).astype(np.float32)
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
