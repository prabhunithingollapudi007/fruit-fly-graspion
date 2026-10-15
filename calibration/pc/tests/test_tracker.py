"""Tracker on rendered scenes: homography at marker height, heading, handedness check, LED sampling, intrinsics."""
import math
import unittest

import numpy as np

from graspion_calib.tracker import (BotMarker, FrameHandednessError, Tracker, Undistorter, marker_corners_world,
                                    render_scene)

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


class TestTracker(unittest.TestCase):
    def _ready(self, refs=REFS, bots=None):
        tr = Tracker(refs, bots or {10: BotMarker(10, (0.012, 0.0), SIZE)}, ref_frames=3)
        img = scene([])
        for k in range(3):
            tr.process(img, k * 0.02, k)
        return tr

    def test_pose_accuracy(self):
        tr = self._ready()
        self.assertIsNotNone(tr.H)
        self.assertLess(tr.reproj_rms_m, 1e-4)
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
        c = marker_corners_world(0.0, 0.0, math.pi / 2, 0.02)       # 'up' = +y
        np.testing.assert_allclose(c[0], [-0.01, 0.01], atol=1e-12)  # TL
        np.testing.assert_allclose(c[1], [0.01, 0.01], atol=1e-12)   # TR


class TestIntrinsics(unittest.TestCase):
    def test_synthetic_charuco(self):
        from graspion_calib import intrinsics
        board = intrinsics.make_board()
        K = np.array([[950.0, 0, 640], [0, 950.0, 360], [0, 0, 1]])
        views = intrinsics.synthetic_views(board, K, n=12)
        res = intrinsics.calibrate(board, views)
        self.assertGreaterEqual(res["views"], 8)
        self.assertLess(res["rms_px"], 1.0)
        self.assertAlmostEqual(res["K"][0, 0] / 950.0, 1.0, delta=0.03)
        self.assertAlmostEqual(res["K"][1, 1] / 950.0, 1.0, delta=0.03)


if __name__ == "__main__":
    unittest.main()
