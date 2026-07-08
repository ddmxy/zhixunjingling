#!/usr/bin/env python3
"""Offline calibrate box-edge line detection from photos in 边缘实拍图片/.

Usage (PC or RDK):
  pip install opencv-python numpy
  python3 box_edge_calibrate.py
  python3 box_edge_calibrate.py --folder /path/to/边缘实拍图片

For each image, finds the strongest near-horizontal line in the lower half
(box opening rim). Prints suggested edge_line_target_y (normalized 0=top, 1=bottom).

Label photos by distance when saving, e.g.:
  far_30cm.jpg  mid_25cm.jpg  near_20cm.jpg  grab_ok_22cm.jpg
"""
from __future__ import annotations

import argparse
import glob
import os
import sys

try:
    import cv2
    import numpy as np
except ImportError:
    print("pip install opencv-python numpy")
    sys.exit(1)

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
DEFAULT_FOLDER = os.path.normpath(os.path.join(SCRIPT_DIR, "..", "边缘实拍图片"))
OUT_FOLDER = os.path.normpath(os.path.join(SCRIPT_DIR, "..", "边缘实拍图片", "calibrated"))


def detect_rim_line(bgr: np.ndarray) -> tuple[float | None, np.ndarray]:
    """Return normalized y of dominant horizontal edge (0=top), debug overlay."""
    h, w = bgr.shape[:2]
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(gray, 50, 150)
    # Box rim usually in lower 70% when camera looks down into recess
    roi = edges[int(h * 0.25) :, :]
    y0 = int(h * 0.25)
    lines = cv2.HoughLinesP(
        roi, 1, np.pi / 180, threshold=60, minLineLength=int(w * 0.25), maxLineGap=20
    )
    vis = bgr.copy()
    cv2.line(vis, (0, y0), (w, y0), (255, 0, 255), 1)
    best_y = None
    best_score = -1.0
    if lines is not None:
        for seg in lines:
            x1, y1, x2, y2 = seg[0]
            if abs(y2 - y1) > 8:
                continue
            length = abs(x2 - x1)
            y_mid = (y1 + y2) / 2 + y0
            # Prefer long, lower (closer to camera / nearer box) lines
            score = length * (1.0 + y_mid / h)
            if score > best_score:
                best_score = score
                best_y = y_mid
                cv2.line(vis, (x1, y1 + y0), (x2, y2 + y0), (0, 255, 0), 2)
    if best_y is not None:
        ny = best_y / h
        cv2.line(vis, (0, int(best_y)), (w, int(best_y)), (0, 0, 255), 2)
        cv2.putText(
            vis, f"rim y={ny:.3f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2
        )
        # Target band for "good grab distance" — tune after near_*.jpg samples
        target = 0.52
        cv2.line(vis, (0, int(target * h)), (w, int(target * h)), (255, 255, 0), 1)
        cv2.putText(vis, "target band", (10, int(target * h) - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)
        return ny, vis
    cv2.putText(vis, "NO EDGE", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
    return None, vis


def main() -> None:
    parser = argparse.ArgumentParser(description="Calibrate box edge from sample photos")
    parser.add_argument("--folder", default=DEFAULT_FOLDER)
    parser.add_argument("--show", action="store_true", help="Show each image window")
    args = parser.parse_args()

    patterns = ["*.jpg", "*.jpeg", "*.png", "*.JPG", "*.PNG"]
    paths: list[str] = []
    for pat in patterns:
        paths.extend(glob.glob(os.path.join(args.folder, pat)))
    paths = sorted(set(paths))

    if not paths:
        print(f"No images in: {args.folder}")
        print("Add photos (far/mid/near) then rerun.")
        sys.exit(1)

    os.makedirs(OUT_FOLDER, exist_ok=True)
    print(f"Processing {len(paths)} images from {args.folder}\n")
    print(f"{'file':<30} {'rim_y':>8}  note")
    print("-" * 55)

    results = []
    for path in paths:
        bgr = cv2.imread(path)
        if bgr is None:
            print(f"{os.path.basename(path):<30}  READ FAIL")
            continue
        ny, vis = detect_rim_line(bgr)
        name = os.path.basename(path)
        out_path = os.path.join(OUT_FOLDER, f"dbg_{name}")
        cv2.imwrite(out_path, vis)
        if ny is None:
            print(f"{name:<30}     —     no line found")
            continue
        note = "far" if ny < 0.42 else ("mid" if ny < 0.50 else "near")
        print(f"{name:<30} {ny:8.3f}  {note}")
        results.append((name, ny))

    if results:
        ys = [r[1] for r in results]
        print("-" * 55)
        print(f"rim_y range: {min(ys):.3f} .. {max(ys):.3f}")
        print(f"Suggested edge_line_target_y for VIS_ALIGN stop: {sum(ys)/len(ys):.3f}")
        print(f"Debug overlays saved to: {OUT_FOLDER}")
        print("\nInterpretation:")
        print("  rim_y smaller (upper in image) = car farther from box")
        print("  rim_y larger (lower in image)  = car closer")
        print("  Stop VIS_ALIGN when live rim_y >= target (after arm_look).")

    if args.show:
        for path in paths:
            dbg = os.path.join(OUT_FOLDER, f"dbg_{os.path.basename(path)}")
            if os.path.isfile(dbg):
                cv2.imshow("edge", cv2.imread(dbg))
                cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
