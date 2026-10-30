"""Tracker on rendered scenes: homography at marker height, heading, handedness check, LED sampling, intrinsics."""
import math
import unittest

import cv2
import numpy as np

from graspion_calib.tracker import (BotMarker, FrameHandednessError, Tracker, Undistorter, led_mode_of,
                                    marker_corners_world, render_scene)

REFS = {0: (0.05, 0.05), 1: (0.95, 0.05), 2: (0.95, 0.95), 3: (0.05, 0.95)}
SIZE = 0.03


def camera_h(tilt=0.15):
    """World (x right, y up, metres) -> pixels for a slightly oblique overhead camera (image y points down)."""
    S = np.array([[640.0, 0, 320], [0, -640.0, 680], [0, 0, 1]])        # 1 m -> 640 px, y flipped
    P = np.array([[1, 0, 0], [0.0, 1, 0], [tilt * 0.3, tilt * 0.2, 1]])  # mild perspective
    return S @ P


def scene(bots, H=None, leds=()):
    H = camera_h() if H is None else H
    markers = [(i, x, y, 0.0, SIZE) for i, (x, y) in REFS.items()] + [(i, x, y, th, SIZE) for i, x, y, th in bots]
    return render_scene(H, markers, img_size=(1280, 720), leds=leds)


def pinhole(C=(0.5, 0.5, 1.0), f=600.0, tilt=0.1, size=(1280, 720)):
    """Overhead pinhole camera at C (world metres, z up towards the camera), tilted about x.
    Returns K, R, t (camera = R world + t) and the world-plane (z = 0) -> pixel homography."""
    K = np.array([[f, 0, size[0] / 2], [0, f, size[1] / 2], [0, 0, 1.0]])
    c, s = math.cos(tilt), math.sin(tilt)
    R = np.array([[1, 0, 0], [0, c, -s], [0, s, c]]) @ np.diag([1.0, -1.0, -1.0])   # looking down, image y = -y
    t = -R @ np.asarray(C, float)
    return K, R, t, K @ np.c_[R[:, 0], R[:, 1], t]


class TestTracker(unittest.TestCase):
    def _ready(self, refs=REFS, bots=None, led_mode="chroma"):
        # the synthetic renders are exactly grey, which 'auto' would (rightly) take for a mono camera
        tr = Tracker(refs, bots or {10: BotMarker(10, (0.012, 0.0), SIZE)}, ref_frames=3, ref_size_m=SIZE,
                     led_mode=led_mode)
        img = scene([])
        for k in range(3):
            tr.process(img, k * 0.02, k)
        return tr

    def test_h_published_last(self):
        """Other threads wait for tracker.H: it must appear only after the checks, warnings and led_mode."""
        import graspion_calib.tracker as T
        seen = []
        real = T.led_mode_of

        def spy(img):                                   # called inside _fit_h, before H may be published
            seen.append(tr.H)
            return real(img)
        T.led_mode_of = spy
        try:
            tr = Tracker(REFS, {10: BotMarker(10, (0.012, 0.0), SIZE)}, ref_frames=3, ref_size_m=SIZE)
            for k in range(3):
                tr.process(scene([]), k * 0.02, k)
        finally:
            T.led_mode_of = real
        self.assertEqual(seen, [None])
        self.assertIsNotNone(tr.H)
        self.assertNotEqual(tr.led_mode, "auto")

    def test_reference_off_warning_is_rate_limited(self):
        tr = self._ready()
        H2 = camera_h()
        H2[0, 2] += 20.0                                # move the image: every reference now looks ~31 mm off
        img = scene([], H=H2)
        n0 = len(tr.warnings)
        for k in range(650):
            tr.process(img, 1.0 + k / 60, 10 + k)
        per_ref = len(tr.warnings) - n0
        self.assertEqual(per_ref, 4 * 3)                # frames 1, 301 and 601 for each of the 4 references
        self.assertEqual(set(tr.ref_off_frames.values()), {650})

    def test_pose_accuracy(self):
        tr = self._ready()
        self.assertIsNotNone(tr.H)
        self.assertTrue(math.isnan(tr.reproj_rms_m))                       # 4 references: exact fit, no check
        for chk in tr.ref_check.values():                                  # the actual survey check passes
            self.assertLess(abs(chk["side_err"]), 0.03)
            self.assertLess(max(chk["aspect_err"], chk["diag_err"]), chk["tol"])
        self.assertEqual(tr.warnings, [])
        self.assertIn("exact 4-point fit", tr.reference_summary())
        rng = np.random.default_rng(0)
        errs = []
        for k in range(10):
            x, y, th = rng.uniform(0.2, 0.8), rng.uniform(0.2, 0.8), rng.uniform(-math.pi, math.pi)
            dets = tr.process(scene([(10, x, y, th)]), 1.0 + k, 10 + k)
            d = [d for d in dets if d.id == 10]
            self.assertEqual(len(d), 1)
            d = d[0]
            self.assertLess(math.hypot(d.x - x, d.y - y), 1.5e-3, f"position error at {x:.2f},{y:.2f}")
            errs.append(abs(math.remainder(d.theta - th, 2 * math.pi)))
            self.assertLess(errs[-1], math.radians(2.5))                  # a 19 px marker: ~1 deg noise
            self.assertAlmostEqual(d.side_m, SIZE, delta=0.002)
        self.assertLess(np.mean(errs), math.radians(1.0))

    def test_ccw_rotation_gives_positive_omega(self):
        tr = self._ready()
        ths = []
        for k in range(6):
            dets = tr.process(scene([(10, 0.5, 0.5, 0.2 * k)]), k, k)
            ths.append([d.theta for d in dets if d.id == 10][0])
        self.assertTrue(np.all(np.diff(np.unwrap(ths)) > 0.15))

    def test_mirrored_reference_layout_rejected(self):
        mirrored = {i: (x, 1.0 - y) for i, (x, y) in REFS.items()}   # y copied from the image direction
        tr = Tracker(mirrored, {}, ref_frames=2)
        img = scene([])
        with self.assertRaises(FrameHandednessError):
            for k in range(2):
                tr.process(img, k, k)

    def test_led_sampling(self):
        # the LED must sit outside the marker and its white quiet zone (here 15 + 10 mm from the centre)
        tr = self._ready(bots={10: BotMarker(10, (0.035, 0.0), SIZE)})
        x, y, th = 0.4, 0.6, 0.7
        led_w = (x + 0.035 * math.cos(th), y + 0.035 * math.sin(th))
        off = [d for d in tr.process(scene([(10, x, y, th)]), 5, 5) if d.id == 10][0]
        on = [d for d in tr.process(scene([(10, x, y, th)], leds=[(*led_w, (220, 0, 0), 3)]), 6, 6) if d.id == 10][0]
        red = [d for d in tr.process(scene([(10, x, y, th)], leds=[(*led_w, (0, 0, 220), 3)]), 7, 7) if d.id == 10][0]
        self.assertGreater(on.led_b - off.led_b, 40)
        self.assertGreater(red.led_r - off.led_r, 40)
        self.assertLess(red.led_b - off.led_b, 40)

    def test_reference_survey_error_detected(self):
        """A mistyped reference coordinate warps the fitted frame; the 4-point RMS cannot see it, the reference
        markers' shapes can (gross errors: 60 mm references, ~36 px, and a 0.1 m typo)."""
        big = 0.06
        img = render_scene(camera_h(), [(i, x, y, 0.0, big) for i, (x, y) in REFS.items()])
        good = Tracker(REFS, {}, ref_frames=2, ref_size_m=big, led_mode="chroma")
        bad = Tracker({**REFS, 2: (0.95, 0.85)}, {}, ref_frames=2, ref_size_m=big, led_mode="chroma")
        for k in range(2):
            good.process(img, k, k)
            bad.process(img, k, k)
        self.assertTrue(math.isnan(bad.reproj_rms_m))                      # what used to print as "0.00 mm"
        self.assertEqual(good.warnings, [])
        self.assertTrue(any("distorted" in w for _, w in bad.warnings), bad.warnings)
        chk = bad.ref_check[2]
        self.assertGreater(max(abs(chk["side_err"]), chk["aspect_err"], chk["diag_err"]), chk["tol"])
        cm = Tracker({i: (100 * x, 100 * y) for i, (x, y) in REFS.items()}, {}, ref_frames=2, ref_size_m=big,
                     led_mode="chroma")                                    # survey typed in cm
        for k in range(2):
            cm.process(img, k, k)
        self.assertTrue(any("reference_size_m" in w for _, w in cm.warnings), cm.warnings)
        # with a fifth reference the reprojection residual is a real check again
        five = dict(REFS)
        five[4] = (0.5, 0.95)
        img = render_scene(camera_h(), [(i, x, y, 0.0, SIZE) for i, (x, y) in five.items()])
        tr5 = Tracker(five, {}, ref_frames=2, ref_size_m=SIZE, led_mode="chroma")
        for k in range(2):
            tr5.process(img, k, k)
        self.assertLess(tr5.reproj_rms_m, 1e-3)
        five[4] = (0.5, 0.90)
        tr5b = Tracker(five, {}, ref_frames=2, ref_size_m=SIZE, led_mode="chroma")
        for k in range(2):
            tr5b.process(img, k, k)
        self.assertGreater(tr5b.reproj_rms_m, 0.01)

    def test_mono_camera_logs_brightness(self):
        """Grey frames (3-channel or 2-D) have zero chroma; the tracker switches to brightness by itself."""
        x, y, th = 0.4, 0.6, 0.7
        led_w = (x + 0.035 * math.cos(th), y + 0.035 * math.sin(th))
        mk = [(i, mx, my, 0.0, SIZE) for i, (mx, my) in REFS.items()]

        def render(on):
            return render_scene(camera_h(), mk + [(10, x, y, th, SIZE)], background=120,
                                leds=[(*led_w, (255, 255, 255), 3)] if on else ())

        for to_2d in (False, True):
            conv = (lambda im: cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)) if to_2d else (lambda im: im)
            tr = Tracker(REFS, {10: BotMarker(10, (0.035, 0.0), SIZE)}, ref_frames=2)
            for k in range(2):
                tr.process(conv(render(False)), k, k)
            self.assertEqual(tr.led_mode, "luma")
            self.assertTrue(any("mono camera" in w for _, w in tr.warnings))
            off = [d for d in tr.process(conv(render(False)), 5, 5) if d.id == 10][0]
            on = [d for d in tr.process(conv(render(True)), 6, 6) if d.id == 10][0]
            self.assertGreater(on.led_b - off.led_b, 40)
            self.assertTrue(math.isnan(on.led_r))
        # a colour frame is recognised as such
        img = render(False)
        img[..., 0] = np.clip(img[..., 0].astype(int) + 9, 0, 255)
        self.assertEqual(led_mode_of(img), "chroma")

    def test_led_height_parallax(self):
        """The LED sits 30 mm below the marker plane, 0.45 m off the camera axis: ~9 px from where the
        marker-plane homography puts it. With led_height (and K) the patch lands on it again."""
        K, R, t, Hw = pinhole()
        x, y, th, dz = 0.85, 0.80, 0.4, -0.03
        led3 = np.array([x + 0.035 * math.cos(th), y + 0.035 * math.sin(th), dz])
        pc = K @ (R @ led3 + t)
        px = pc[:2] / pc[2]
        w = np.linalg.solve(Hw, np.r_[px, 1.0])                            # plane point on the same ray
        mk = [(i, mx, my, 0.0, SIZE) for i, (mx, my) in REFS.items()]
        dark = render_scene(Hw, mk + [(10, x, y, th, SIZE)])
        lit = render_scene(Hw, mk + [(10, x, y, th, SIZE)], leds=[(w[0] / w[2], w[1] / w[2], (220, 0, 0), 3)])

        def rise(**kw):
            und = kw.pop("und", Undistorter(K, np.zeros(5)))
            tr = Tracker(REFS, {10: BotMarker(10, (0.035, 0.0), SIZE, led_height=kw.pop("led_height", dz))}, und,
                         ref_frames=2, led_mode="chroma", **kw)
            for k in range(2):
                tr.process(dark, k, k)
            off = [d for d in tr.process(dark, 5, 5) if d.id == 10][0]
            on = [d for d in tr.process(lit, 6, 6) if d.id == 10][0]
            return on.led_b - off.led_b

        self.assertLess(rise(led_height=0.0), 20)                          # marker-plane mapping misses the LED
        self.assertGreater(rise(), 40)                                     # 3-D projection with K
        self.assertGreater(rise(und=Undistorter(), camera_height_m=1.0), 40)   # no K: scale about the centre

    def test_wrong_size_rejected(self):
        tr = self._ready()
        H = camera_h()
        img = render_scene(H, [(i, x, y, 0.0, SIZE) for i, (x, y) in REFS.items()] + [(10, 0.5, 0.5, 0.0, 0.05)])
        self.assertEqual([d for d in tr.process(img, 9, 9) if d.id == 10], [])

    def test_distortion_round_trip(self):
        K = np.array([[900.0, 0, 640], [0, 900.0, 360], [0, 0, 1]])
        u = Undistorter(K, np.array([-0.12, 0.05, 0.001, -0.0005, 0.0]))
        pts = np.array([[100.0, 80.0], [640.0, 360.0], [1200.0, 650.0], [300.0, 600.0]])
        back = u.undistort(u.distort(pts))
        np.testing.assert_allclose(back, pts, atol=0.05)

    def test_marker_corner_order(self):
        th = math.pi / 2
        c = marker_corners_world(0.0, 0.0, th, 0.02)                 # 'up' = +y
        np.testing.assert_allclose(c[0], [-0.01, 0.01], atol=1e-12)  # TL
        np.testing.assert_allclose(c[1], [0.01, 0.01], atol=1e-12)   # TR
        # theta (and marker_yaw) is the 'up' direction; the corner 0 -> 1 edge points 90 deg clockwise of it
        e01 = c[1] - c[0]
        self.assertAlmostEqual(math.atan2(e01[1], e01[0]), th - math.pi / 2, delta=1e-12)


if __name__ == "__main__":
    unittest.main()
