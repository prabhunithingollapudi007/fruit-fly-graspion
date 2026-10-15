"""ChArUco camera intrinsics (K, dist) -> .npz for the tracker. UNTESTED with a real camera (synthetic test only).

Print the board flat (check the printed square size with calipers and pass the measured value), then capture
15-30 views covering the whole image at different tilts.
"""
from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np


def make_board(squares=(7, 5), square_m=0.03, marker_m=0.022, dictionary="DICT_5X5_100"):
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
    return cv2.aruco.CharucoBoard(squares, square_m, marker_m, d)


def board_image(board, px_per_square=150, margin_px=40):
    sx, sy = board.getChessboardSize()
    return board.generateImage((sx * px_per_square + 2 * margin_px, sy * px_per_square + 2 * margin_px), None,
                               margin_px)


def detect(board, images, min_corners=8):
    det = cv2.aruco.CharucoDetector(board)
    obj, img, size = [], [], None
    for im in images:
        gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY) if im.ndim == 3 else im
        size = gray.shape[::-1]
        cc, ci, _, _ = det.detectBoard(gray)
        if ci is None or len(ci) < min_corners:
            continue
        o, p = board.matchImagePoints(cc, ci)
        obj.append(o)
        img.append(p)
    return obj, img, size


def calibrate(board, images, fix_k3=True):
    obj, img, size = detect(board, images)
    if len(obj) < 5:
        raise RuntimeError(f"only {len(obj)} usable views; need >= 5 (15-30 recommended)")
    flags = cv2.CALIB_FIX_K3 if fix_k3 else 0
    rms, K, dist, _, _ = cv2.calibrateCamera(obj, img, size, None, None, flags=flags)
    return {"rms_px": float(rms), "K": K, "dist": dist.ravel(), "views": len(obj), "size": size}


def calibrate_folder(folder, out_npz, **board_kw):
    board = make_board(**board_kw)
    images = [cv2.imread(str(p)) for p in sorted(Path(folder).glob("*")) if p.suffix.lower() in (".png", ".jpg", ".jpeg", ".bmp")]
    res = calibrate(board, [im for im in images if im is not None])
    np.savez(out_npz, K=res["K"], dist=res["dist"], rms_px=res["rms_px"], size=np.array(res["size"]))
    return res


def synthetic_views(board, K, n=12, size=(1280, 720), seed=0, px_per_square=120):
    """Render the board through a pinhole camera at random poses (no distortion): for the self-test."""
    rng = np.random.default_rng(seed)
    sx, sy = board.getChessboardSize()
    sq = board.getSquareLength()
    margin = px_per_square // 2
    tex = board_image(board, px_per_square, margin)
    th, tw = tex.shape[:2]
    # texture pixel -> board metres (board origin at the first inner corner's square corner)
    s = sq / px_per_square
    T = np.array([[s, 0, -margin * s], [0, s, -margin * s], [0, 0, 1]])
    views = []
    for _ in range(n):
        rvec = rng.normal(0, 0.35, 3)
        tvec = np.array([-0.5 * sx * sq + rng.normal(0, 0.03), -0.5 * sy * sq + rng.normal(0, 0.03),
                         rng.uniform(0.35, 0.6)])
        R, _ = cv2.Rodrigues(rvec)
        Hb = K @ np.c_[R[:, 0], R[:, 1], tvec]                  # board plane (z = 0) -> image
        M = Hb @ T
        im = cv2.warpPerspective(cv2.cvtColor(tex, cv2.COLOR_GRAY2BGR) if tex.ndim == 2 else tex, M, size,
                                 borderValue=(128, 128, 128))
        views.append(im)
    return views
