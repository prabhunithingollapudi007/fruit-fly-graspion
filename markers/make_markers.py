"""Printable sheet of the reference ArUco markers for the calibration rig.

Reads the reference IDs, corners and printed size from a rig config and writes a vector A4 PDF (exact
scale when printed at 100 % / "Actual size") plus a PNG preview of the same layout.

    python make_markers.py                                  # calibration/pc/configs/rig_example.json, IDs 4 5 extra
    python make_markers.py --rig my_rig.json --extra        # only the config's references

50 mm is the default size. With the lab's camera (1.63-1.85 px/mm, calibration/README.md "This rig's camera")
that is 13-15 px per marker cell, so the references still decode with binning or a coarser camera: in a
simulated view at ~0.63 px/mm, 20 mm markers were found in 42 % of frames, 35 mm and up in >= 99 %.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
A4 = (210.0, 297.0)                      # mm
PT = 72 / 25.4                           # PDF points per mm
TOP = 48.0                               # top of the first row of markers, mm from the page top


def marker_bits(mid, dictionary):
    """6x6 cell matrix (1 = white) of a 4x4 marker, including its 1-cell black border."""
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary))
    return (cv2.aruco.generateImageMarker(d, int(mid), 6) > 127).astype(int)


def corner(pos, refs):
    """Corner name of a reference as the camera sees the arena (x right, y up), from the centroid of all of them."""
    cx, cy = np.mean([p for _, p in refs if p is not None], axis=0)
    return f"{'bottom' if pos[1] < cy else 'top'}-{'left' if pos[0] < cx else 'right'} corner"


def layout(refs, size, dictionary):
    """Shapes in mm, origin top-left: ('rect', x, y, w, h, grey), ('dash', x, y, w, h), ('text', x, y, pt, str)."""
    margin = max(5.0, size / 6)                                  # white quiet zone: one cell, at least 5 mm
    cut = size + 2 * margin
    cols = int((A4[0] - 30) // (cut + 10))
    pitch_x, pitch_y = (A4[0] - 30) / cols, cut + 11                 # 11 mm: the label under each marker
    if TOP + -(-len(refs) // cols) * pitch_y > A4[1] - 12:
        raise SystemExit(f"{len(refs)} markers of {size:g} mm do not fit on one A4 page")
    notes = ["Print at 100 % / Actual size. A few % of printer scaling does no harm: the tracker uses the",
             "centres you survey, and the printed size only feeds a 10 % sanity check (reference_size_m).",
             "Cut on the dashed line, keeping the white border. Glue each flat on a spacer, its face level",
             "with the bot-cap markers. Put it in the corner named below, as the camera sees the arena,",
             "then measure its centre into reference_markers. Rotation does not matter."]
    shapes = [("text", 15, 15, 14, f"GRASPion calibration: reference markers ({dictionary}, {size:g} mm)")]
    shapes += [("text", 15, 22 + 4.5 * i, 9, s) for i, s in enumerate(notes)]
    for k, (mid, pos) in enumerate(refs):
        x0 = 15 + (k % cols) * pitch_x + (pitch_x - cut) / 2
        y0 = TOP + (k // cols) * pitch_y
        shapes.append(("dash", x0, y0, cut, cut))
        bx, by, c = x0 + margin, y0 + margin, size / 6
        shapes.append(("rect", bx, by, size, size, 0))
        for r, row in enumerate(marker_bits(mid, dictionary)):     # white cells, merged into runs per row
            j = 0
            while j < 6:
                if row[j]:
                    n = 1
                    while j + n < 6 and row[j + n]:
                        n += 1
                    shapes.append(("rect", bx + j * c, by + r * c, n * c, c, 1))
                    j += n
                else:
                    j += 1
        where = corner(pos, refs) if pos is not None else "optional extra: along an edge, survey it too"
        shapes.append(("text", x0, y0 + cut + 5, 10, f"ID {mid}   {where}"))
    return shapes


def pdf(shapes, path):
    def esc(s):
        return s.replace("\\", "\\\\").replace("(", "\\(").replace(")", "\\)")
    ops, H = [], A4[1]
    for s in shapes:
        if s[0] == "rect":
            _, x, y, w, h, g = s
            ops.append(f"{g} g {x * PT:.3f} {(H - y - h) * PT:.3f} {w * PT:.3f} {h * PT:.3f} re f")
        elif s[0] == "dash":
            _, x, y, w, h = s
            ops.append(f"0.6 G 0.4 w [3 3] 0 d {x * PT:.3f} {(H - y - h) * PT:.3f} {w * PT:.3f} {h * PT:.3f} re S "
                       f"[] 0 d")
        else:
            _, x, y, size, txt = s
            ops.append(f"0 g BT /F1 {size} Tf {x * PT:.3f} {(H - y) * PT:.3f} Td ({esc(txt)}) Tj ET")
    content = "\n".join(ops).encode("latin-1")
    objs = [b"<< /Type /Catalog /Pages 2 0 R >>",
            b"<< /Type /Pages /Kids [3 0 R] /Count 1 >>",
            f"<< /Type /Page /Parent 2 0 R /MediaBox [0 0 {A4[0] * PT:.3f} {A4[1] * PT:.3f}] "
            f"/Resources << /Font << /F1 5 0 R >> >> /Contents 4 0 R >>".encode(),
            b"<< /Length %d >>\nstream\n" % len(content) + content + b"\nendstream",
            b"<< /Type /Font /Subtype /Type1 /BaseFont /Helvetica >>"]
    out, offs = bytearray(b"%PDF-1.4\n"), []
    for i, o in enumerate(objs, 1):
        offs.append(len(out))
        out += b"%d 0 obj\n" % i + o + b"\nendobj\n"
    xref = len(out)
    out += b"xref\n0 %d\n0000000000 65535 f \n" % (len(objs) + 1)
    out += b"".join(b"%010d 00000 n \n" % o for o in offs)
    out += b"trailer\n<< /Size %d /Root 1 0 R >>\nstartxref\n%d\n%%%%EOF\n" % (len(objs) + 1, xref)
    path.write_bytes(bytes(out))


def png(shapes, path, dpi=300):
    """Raster preview of the same shapes (text left out); also what the self-check detects markers on."""
    px = dpi / 25.4
    img = np.full((round(A4[1] * px), round(A4[0] * px)), 255, np.uint8)
    for s in shapes:
        if s[0] == "rect":
            _, x, y, w, h, g = s
            img[round(y * px):round((y + h) * px), round(x * px):round((x + w) * px)] = 255 * g
        elif s[0] == "dash":
            _, x, y, w, h = s
            cv2.rectangle(img, (round(x * px), round(y * px)), (round((x + w) * px), round((y + h) * px)), 150, 1)
    cv2.imwrite(str(path), img)
    return img


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--rig", default=HERE.parent / "calibration/pc/configs/rig_example.json", type=Path)
    ap.add_argument("--size-mm", type=float, help="black-square side (default: camera.reference_size_m)")
    ap.add_argument("--extra", type=int, nargs="*", default=[4, 5],
                    help="extra reference IDs to print (default 4 5: a 5th reference makes the fit checkable)")
    ap.add_argument("--dictionary", default="DICT_4X4_50")
    ap.add_argument("--out", default=HERE / "reference_markers", type=Path, help="output path without suffix")
    a = ap.parse_args()

    rig = json.loads(a.rig.read_text())
    size = a.size_mm or rig["camera"]["reference_size_m"] * 1000
    refs = [(int(k), v) for k, v in sorted(rig["reference_markers"].items(), key=lambda kv: int(kv[0]))]
    refs += [(i, None) for i in a.extra if i not in dict(refs)]
    shapes = layout(refs, size, a.dictionary)
    pdf(shapes, a.out.with_suffix(".pdf"))
    img = png(shapes, a.out.with_suffix(".png"))

    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, a.dictionary)))
    _, ids, _ = det.detectMarkers(img)
    found = sorted(int(i) for i in ids.ravel()) if ids is not None else []
    want = sorted(i for i, _ in refs)
    print(f"wrote {a.out.with_suffix('.pdf')} and .png: IDs {want}, {size:g} mm; preview detects {found}")
    if found != want:
        raise SystemExit("self-check failed: the preview does not decode to the requested IDs")


if __name__ == "__main__":
    main()
