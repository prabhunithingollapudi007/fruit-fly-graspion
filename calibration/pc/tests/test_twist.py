"""Twist / ICR fit on EllipticalBot2D-style trajectories.

The kinematics replicate tilemat_control_stack .../acados_simple/Ellipse2dObj.py (EllipticalBot2D.move_bot:
rotate about a CoR offset laterally from the CoM by d_cw / d_ccw, then translate by v along the heading), so
the CoM body twist is (v + d |omega|, 0, omega). Set GRASPION_TILEMAT_SIM to that folder to also run the
user's own class (skipped if it, or its matplotlib import, is unavailable).
"""
import math
import os
import sys
import unittest

import numpy as np

from graspion_calib.geometry import BotGeometry, icr_body, transfer_twist
from graspion_calib.twist import fit_twist

D_COR = (0.0145, 0.0161)


def elliptical_bot_path(v, w, T=3.0, fps=60.0, d_cor=D_COR, D_r=0.0, pos_noise=3e-4, th_noise=0.005, seed=0,
                        th0=0.3, substeps=10):
    """EllipticalBot2D.move_bot, integrated with substeps per camera frame; returns noisy samples of the CoM."""
    rng = np.random.default_rng(seed)
    x = y = 0.0
    th = th0
    dt = 1.0 / fps / substeps
    ts, xs, ys, hs = [], [], [], []
    for k in range(int(T * fps)):
        for _ in range(substeps):
            wk = w
            c, s = math.cos(th), math.sin(th)
            if wk > 0:
                cx, cy = x - d_cor[1] * s, y + d_cor[1] * c
            else:
                cx, cy = x + d_cor[0] * s, y - d_cor[0] * c
            a = wk * dt
            rx, ry = x - cx, y - cy
            x = cx + math.cos(a) * rx - math.sin(a) * ry + v * c * dt
            y = cy + math.sin(a) * rx + math.cos(a) * ry + v * s * dt
            th += a + math.sqrt(2 * D_r * dt) * rng.normal()
        ts.append((k + 1) / fps)
        xs.append(x + rng.normal(0, pos_noise))
        ys.append(y + rng.normal(0, pos_noise))
        hs.append(th + rng.normal(0, th_noise))
    return np.array(ts), np.array(xs), np.array(ys), np.array(hs)


CASES = [(0.06, 1.0), (0.06, -1.0), (0.08, 0.05), (0.0, 3.2), (0.09, -2.5), (0.11, 0.0), (0.03, -3.5)]


class TestTwist(unittest.TestCase):
    def test_elliptical_bot_twist(self):
        for i, (v, w) in enumerate(CASES):
            t, x, y, th = elliptical_bot_path(v, w, seed=i)
            f = fit_twist(t, x, y, th)
            d = D_COR[1] if w > 0 else D_COR[0]
            self.assertAlmostEqual(f["u1"], v + d * abs(w), delta=1e-3, msg=f"u1 v={v} w={w}")   # 1 mm/s
            self.assertAlmostEqual(f["u2"], 0.0, delta=1e-3, msg=f"u2 v={v} w={w}")
            self.assertAlmostEqual(f["omega"], w, delta=0.01, msg=f"omega v={v} w={w}")

    def test_heading_diffusion_does_not_bias_u(self):
        errs = []
        for i in range(10):
            t, x, y, th = elliptical_bot_path(0.08, -0.8, D_r=0.03, seed=100 + i)
            f = fit_twist(t, x, y, th)
            errs.append(f["u1"] - (0.08 + D_COR[0] * 0.8))
        self.assertLess(abs(np.mean(errs)), 1.5e-3)

    def test_icr_and_marker_offset(self):
        v, w = 0.05, 2.0
        t, x, y, th = elliptical_bot_path(v, w, pos_noise=1e-5, th_noise=1e-4)
        g = BotGeometry(10, marker_offset=(0.006, -0.003), marker_yaw=0.05)
        mx, my, mth = g.marker_from_body(x, y, th)
        fm = fit_twist(t, mx, my, mth)                        # twist of the marker point
        bx, by, bth = g.body_from_marker(mx, my, mth)
        fb = fit_twist(t, bx, by, bth)                        # back at the CoM
        u_exp = transfer_twist(v + D_COR[1] * w, 0.0, w, g.marker_offset)
        # the marker twist is expressed in the marker's (yawed) frame
        c, s = math.cos(g.marker_yaw), math.sin(g.marker_yaw)
        u1m, u2m = c * u_exp[0] + s * u_exp[1], -s * u_exp[0] + c * u_exp[1]
        self.assertAlmostEqual(fm["u1"], u1m, delta=2e-4)
        self.assertAlmostEqual(fm["u2"], u2m, delta=2e-4)
        self.assertAlmostEqual(fb["u1"], v + D_COR[1] * w, delta=2e-4)
        self.assertAlmostEqual(fb["u2"], 0.0, delta=2e-4)
        icx, icy = icr_body(fb["u1"], fb["u2"], fb["omega"])
        self.assertAlmostEqual(icx, 0.0, delta=1e-3)
        self.assertAlmostEqual(icy, (v + D_COR[1] * w) / w, delta=1e-3)

    def test_outlier_rejection(self):
        t, x, y, th = elliptical_bot_path(0.07, 0.5, seed=5)
        x2, th2 = x.copy(), th.copy()
        x2[50] += 0.05
        th2[90] += math.pi                                    # a flipped marker detection
        f = fit_twist(t, x2, y, th2)
        self.assertGreaterEqual(f["n_rejected"], 2)
        self.assertAlmostEqual(f["u1"], 0.07 + D_COR[1] * 0.5, delta=1.5e-3)

    @unittest.skipUnless(os.environ.get("GRASPION_TILEMAT_SIM"), "set GRASPION_TILEMAT_SIM to run the user's plant")
    def test_users_elliptical_bot2d(self):
        sys.path.insert(0, os.environ["GRASPION_TILEMAT_SIM"])
        try:
            from Ellipse2dObj import EllipticalBot2D   # read-only import of the user's plant
        except ImportError as e:
            self.skipTest(f"cannot import Ellipse2dObj: {e}")
        for v, w in CASES:
            bot = EllipticalBot2D(r=np.array([0.0, 0.0]), theta_rad=0.3, vmag=0.0, w=0.0, a=0.03, b=0.015,
                                  npoints=5, dist_rotPoint_to_CoM=np.array(D_COR))
            dt, ts, xs, ys, hs, th = 1 / 600, [], [], [], [], 0.3
            for k in range(1800):
                bot.velocity = v * np.array([np.cos(bot.theta), np.sin(bot.theta)])
                bot.ang_velocity = w
                bot.move_bot(dt)
                th += w * dt
                if k % 10 == 9:
                    ts.append((k + 1) * dt); xs.append(bot.pos[0]); ys.append(bot.pos[1]); hs.append(th)
            f = fit_twist(np.array(ts), np.array(xs), np.array(ys), np.array(hs))
            d = D_COR[1] if w > 0 else D_COR[0]
            self.assertAlmostEqual(f["u1"], v + d * abs(w), delta=1e-3)
            self.assertAlmostEqual(f["omega"], w, delta=0.01)


if __name__ == "__main__":
    unittest.main()
