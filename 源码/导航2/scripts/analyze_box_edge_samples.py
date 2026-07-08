#!/usr/bin/env python3
"""Offline analysis of box-edge sample photos (arm_look camera).

Detects the **cardboard box boundary** (vertical wall + bottom rim), not floor seams.
Run on PC or RDK:
  python3 analyze_box_edge_samples.py --input ~/边缘实拍图片 --output ~/edge_analysis
"""
from __future__ import annotations

import argparse
import csv
from pathlib import Path

import cv2
import numpy as np


def _roi_bounds(h: int, w: int) -> tuple[int, int, int, int]:
    y1, y2 = int(h / 16), int(h / 16 * 12)
    x1, x2 = int(w / 16 * 4), int(w / 16 * 15)
    return x1, y1, x2, y2


def _segment_cardboard(img: np.ndarray, roi: tuple[int, int, int, int]) -> np.ndarray:
    x1, y1, x2, y2 = roi
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # Tan/brown cardboard (exclude green block and gray floor)
    cardboard = cv2.inRange(hsv, (5, 20, 55), (42, 210, 245))
    green = cv2.inRange(hsv, (50, 46, 24), (80, 255, 255))
    cardboard = cv2.bitwise_and(cardboard, cv2.bitwise_not(green))

    roi_mask = np.zeros_like(cardboard)
    roi_mask[y1:y2, x1:x2] = 255
    cardboard = cv2.bitwise_and(cardboard, roi_mask)

    k = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
    cardboard = cv2.morphologyEx(cardboard, cv2.MORPH_CLOSE, k, iterations=2)
    cardboard = cv2.morphologyEx(cardboard, cv2.MORPH_OPEN, k, iterations=1)
    return cardboard


def _pick_box_contour(mask: np.ndarray, roi: tuple[int, int, int, int]) -> np.ndarray | None:
    x1, y1, x2, y2 = roi
    roi_area = (x2 - x1) * (y2 - y1)
    cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    best = None
    best_score = -1.0
    for c in cnts:
        area = float(cv2.contourArea(c))
        if area < roi_area * 0.025:
            continue
        bx, by, bw, bh = cv2.boundingRect(c)
        # Box is usually on the right; reject blobs hugging the left ROI border (floor glare)
        cx = bx + 0.5 * bw
        if cx < x1 + 0.35 * (x2 - x1):
            continue
        if bh < 0.12 * (y2 - y1):
            continue
        score = area * (0.5 + cx / (x2 + 1))
        if score > best_score:
            best_score = score
            best = c
    return best


def _refine_wall_x(img: np.ndarray, x_est: int, y_top: int, y_bot: int) -> int:
    """Snap wall x to floor->card://cardboard transition just right of the mask estimate."""
    h, w = img.shape[:2]
    y_top = max(0, y_top)
    y_bot = min(h, y_bot)
    if y_bot - y_top < 8:
        return x_est
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.float32)
    xs_ref: list[int] = []
    for y in range(y_top, y_bot, 2):
        x0 = max(0, x_est - 6)
        x1 = min(w, x_est + 32)
        profile = gray[y, x0:x1]
        if profile.size < 4:
            continue
        diff = np.diff(profile)
        # Strongest brightening step = enter cardboard from floor/shadow on the left
        xs_ref.append(x0 + 1 + int(np.argmax(diff)))
    if len(xs_ref) >= 6:
        return int(np.median(xs_ref))
    return x_est


def _left_boundary(mask: np.ndarray, bx: int, by: int, bw: int, bh: int) -> tuple[np.ndarray, np.ndarray]:
    """Per-row leftmost cardboard pixel -> smooth vertical wall profile."""
    y_end = min(by + bh, mask.shape[0])
    xs: list[int] = []
    ys: list[int] = []
    x_end = min(bx + bw + 8, mask.shape[1])
    for y in range(by, y_end):
        row = mask[y, bx:x_end]
        cols = np.flatnonzero(row > 0)
        if cols.size:
            xs.append(bx + int(cols[0]))
            ys.append(y)
    if not xs:
        empty = np.array([], dtype=np.int32)
        return empty, empty
    xs_arr = np.array(xs, dtype=np.float32)
    ys_arr = np.array(ys, dtype=np.int32)
    # Median filter along y to kill single-row spikes (floor bleed / noise)
    k = 7
    if len(xs_arr) >= k:
        pad = k // 2
        padded = np.pad(xs_arr, (pad, pad), mode="edge")
        smoothed = np.array(
            [np.median(padded[i : i + k]) for i in range(len(xs_arr))], dtype=np.float32
        )
        xs_arr = smoothed
    return xs_arr, ys_arr


def _fit_vertical_wall(xs: np.ndarray, ys: np.ndarray, y_top: int, y_bot: int) -> tuple[int, int, int]:
    """Fit a straight vertical wall inside [y_top, y_bot]; prefer upper 65% for x."""
    if len(xs) < 6:
        return int(np.median(xs)) if len(xs) else 0, y_top, y_bot

    y_cut = y_top + int(0.65 * max(1, y_bot - y_top))
    upper = xs[ys <= y_cut]
    x_wall = int(np.median(upper if len(upper) >= 6 else xs))

    # Huber line fit; clamp to near-vertical
    pts = np.column_stack([xs, ys.astype(np.float32)])
    vx, vy, x0, y0 = cv2.fitLine(pts, cv2.DIST_HUBER, 0, 0.01, 0.01).flatten()
    if abs(float(vy)) > 1e-3 and abs(float(vx / vy)) < 0.08:
        x_at = int(x0 + (y_top - y0) * (vx / vy))
        x_wall = int(0.6 * x_wall + 0.4 * x_at)
    return x_wall, y_top, y_bot


def _scan_horizontal_rim(
    mask: np.ndarray, bx: int, by: int, bw: int, bh: int, *, top: bool
) -> tuple[int | None, tuple[int, int, int, int] | None]:
    """Top/bottom horizontal rim from column-wise boundary scan."""
    x_end = min(bx + bw, mask.shape[1])
    y_lo = by
    y_hi = min(by + bh, mask.shape[0])
    rim_ys: list[int] = []
    rim_xs: list[int] = []
    step = max(1, bw // 24)
    for x in range(bx, x_end, step):
        col = mask[y_lo:y_hi, x]
        idx = np.flatnonzero(col > 0)
        if idx.size == 0:
            continue
        y = y_lo + (int(idx[0]) if top else int(idx[-1]))
        rim_ys.append(y)
        rim_xs.append(x)

    if len(rim_ys) < 5:
        return None, None

    y_rim = int(np.median(rim_ys))
    x_left = int(np.min(rim_xs))
    x_right = int(np.max(rim_xs))
    span = x_right - x_left
    if span < max(20, int(bw * 0.25)):
        return None, None
    return y_rim, (x_left, y_rim, x_right, y_rim)


def _extract_box_edges(mask: np.ndarray, contour: np.ndarray, h: int, w: int, img: np.ndarray) -> dict:
    bx, by, bw, bh = cv2.boundingRect(contour)
    # Slight erode removes floor bleed on the left of the mask
    k3 = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    tight = cv2.erode(mask, k3, iterations=1)
    if cv2.countNonZero(tight) < 0.5 * cv2.countNonZero(mask):
        tight = mask
    xs, ys = _left_boundary(tight, bx, by, bw, bh)

    top_y, top_line = _scan_horizontal_rim(tight, bx, by, bw, bh, top=True)
    bot_y, bot_line = _scan_horizontal_rim(tight, bx, by, bw, bh, top=False)

    rim_source = "none"
    active_rim = None
    edge_y_norm = float("nan")

    if top_y is not None:
        rim_source = "top"
        active_rim = top_line
        edge_y_norm = top_y / h
        y_wall_top = top_y
        # Vertical wall stops above floor contact; bottom rim only used as fallback metric
        y_wall_bot = top_y + int(0.72 * max(bh, 1))
        y_wall_bot = min(y_wall_bot, by + bh)
    elif bot_y is not None:
        rim_source = "bottom"
        active_rim = bot_line
        edge_y_norm = bot_y / h
        y_wall_top = by
        y_wall_bot = bot_y
    else:
        y_wall_top = by
        y_wall_bot = by + bh

    if len(xs) >= 6:
        x_wall, y1, y2 = _fit_vertical_wall(xs, ys, y_wall_top, y_wall_bot)
        x_wall = _refine_wall_x(img, x_wall, y1, y2)
        vertical = (x_wall, y1, x_wall, y2)
        edge_x_norm = x_wall / w
    else:
        vertical = None
        edge_x_norm = float("nan")

    return {
        "vertical": vertical,
        "top_rim": top_line,
        "bottom_rim": bot_line if rim_source == "bottom" else None,
        "active_rim": active_rim,
        "rim_source": rim_source,
        "edge_y_norm": edge_y_norm,
        "edge_x_norm": edge_x_norm,
        "bbox": (bx, by, bw, bh),
    }


def _detect_green(img: np.ndarray) -> tuple[float, float, float]:
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    green = cv2.inRange(hsv, (50, 46, 24), (80, 255, 255))
    green = cv2.erode(green, None, iterations=2)
    cnts, _ = cv2.findContours(green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return float("nan"), float("nan"), 0.0
    c = max(cnts, key=cv2.contourArea)
    area = float(cv2.contourArea(c))
    if area <= 200:
        return float("nan"), float("nan"), area
    m = cv2.moments(c)
    return m["m10"] / m["m01"], m["m01"] / m["m00"], area


def detect_box_edge(img: np.ndarray) -> dict:
    h, w = img.shape[:2]
    roi = _roi_bounds(h, w)
    mask = _segment_cardboard(img, roi)
    contour = _pick_box_contour(mask, roi)

    gx, gy, g_area = _detect_green(img)

    if contour is None:
        return {
            "edge_y_norm": float("nan"),
            "edge_x_norm": float("nan"),
            "edge_band": "unknown",
            "green_cx_norm": gx / w if g_area > 200 else float("nan"),
            "green_cy_norm": gy / h if g_area > 200 else float("nan"),
            "green_area": g_area,
            "vertical": None,
            "top_rim": None,
            "bottom_rim": None,
            "active_rim": None,
            "rim_source": "none",
            "bbox": None,
            "mask": mask,
        }

    edges = _extract_box_edges(mask, contour, h, w, img)
    x_norm = edges["edge_x_norm"]
    y_norm = edges["edge_y_norm"]
    # Distance: prefer top-rim height; fall back to wall x when rim missing
    if edges["rim_source"] == "top" and not np.isnan(y_norm):
        if y_norm < 0.34:
            band = "far"
        elif y_norm < 0.48:
            band = "mid"
        else:
            band = "near"
    elif not np.isnan(x_norm):
        if x_norm > 0.68:
            band = "far"
        elif x_norm > 0.50:
            band = "mid"
        else:
            band = "near"
    else:
        band = "unknown"

    return {
        "edge_y_norm": y_norm,
        "edge_x_norm": x_norm,
        "edge_band": band,
        "rim_source": edges["rim_source"],
        "green_cx_norm": gx / w if g_area > 200 else float("nan"),
        "green_cy_norm": gy / h if g_area > 200 else float("nan"),
        "green_area": g_area,
        "vertical": edges["vertical"],
        "top_rim": edges["top_rim"],
        "bottom_rim": edges["bottom_rim"],
        "active_rim": edges["active_rim"],
        "bbox": edges["bbox"],
        "mask": mask,
    }


def draw_overlay(img: np.ndarray, info: dict) -> np.ndarray:
    out = img.copy()
    h, w = out.shape[:2]
    x1, y1, x2, y2 = _roi_bounds(h, w)
    cv2.rectangle(out, (x1, y1), (x2, y2), (0, 255, 0), 2)
    cv2.line(out, (0, int(h * 0.45)), (w, int(h * 0.45)), (255, 255, 0), 1)

    if info["vertical"]:
        vx1, vy1, vx2, vy2 = info["vertical"]
        cv2.line(out, (vx1, vy1), (vx2, vy2), (0, 0, 255), 3)
    if info["top_rim"]:
        tx1, ty1, tx2, ty2 = info["top_rim"]
        cv2.line(out, (tx1, ty1), (tx2, ty2), (255, 200, 0), 2)  # cyan-ish top rim
    if info["bottom_rim"]:
        bx1, by1, bx2, by2 = info["bottom_rim"]
        cv2.line(out, (bx1, by1), (bx2, by2), (255, 0, 255), 2)

    if info["vertical"] or info["active_rim"]:
        src = info.get("rim_source", "?")
        y_txt = f"{info['edge_y_norm']:.2f}" if not np.isnan(info["edge_y_norm"]) else "?"
        x_txt = f"{info['edge_x_norm']:.2f}" if not np.isnan(info["edge_x_norm"]) else "?"
        label = f"rim={src} wall_x={x_txt} rim_y={y_txt} {info['edge_band']}"
        cv2.putText(out, label, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

    if info["green_area"] > 200:
        cv2.circle(
            out,
            (int(info["green_cx_norm"] * w), int(info["green_cy_norm"] * h)),
            8,
            (0, 255, 0),
            -1,
        )
    return out


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--input",
        default=str(Path(__file__).resolve().parents[2] / "边缘实拍图片"),
        help="folder with sample jpg/png",
    )
    parser.add_argument("--output", default=str(Path(__file__).resolve().parent / "edge_analysis"))
    args = parser.parse_args()

    in_dir = Path(args.input)
    out_dir = Path(args.output)
    out_dir.mkdir(parents=True, exist_ok=True)

    files = sorted(list(in_dir.glob("*.jpg")) + list(in_dir.glob("*.png")))
    if not files:
        raise SystemExit(f"No images in {in_dir}")

    rows = []
    for fp in files:
        data = fp.read_bytes()
        arr = np.frombuffer(data, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
        if img is None:
            continue
        info = detect_box_edge(img)
        overlay = draw_overlay(img, info)
        cv2.imwrite(str(out_dir / f"{fp.stem}_edge.jpg"), overlay)
        rows.append(
            {
                "file": fp.name,
                "edge_y_norm": f"{info['edge_y_norm']:.3f}" if info["vertical"] else "",
                "edge_x_norm": f"{info['edge_x_norm']:.3f}" if info["vertical"] else "",
                "band": info["edge_band"],
                "rim_source": info.get("rim_source", ""),
                "green_cx": f"{info['green_cx_norm']:.3f}" if info["green_area"] > 200 else "",
                "green_cy": f"{info['green_cy_norm']:.3f}" if info["green_area"] > 200 else "",
            }
        )

    csv_path = out_dir / "summary.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(
            f,
            fieldnames=["file", "edge_y_norm", "edge_x_norm", "band", "rim_source", "green_cx", "green_cy"],
        )
        writer.writeheader()
        writer.writerows(rows)

    bands: dict[str, int] = {}
    for r in rows:
        bands[r["band"]] = bands.get(r["band"], 0) + 1
    print(f"Processed {len(rows)} images -> {out_dir}")
    print("Distance bands (box wall x, smaller=closer):", bands)
    print("Overlays: red=vertical wall, orange=top rim (priority), magenta=bottom rim (fallback)")
    print(f"CSV: {csv_path}")


if __name__ == "__main__":
    main()
