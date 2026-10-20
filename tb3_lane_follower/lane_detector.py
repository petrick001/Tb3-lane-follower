"""Pure-OpenCV lane detector (no ROS imports, so it can be tested offline).

Basic classical algorithm:
  1. Per-frame brightness normalisation (so lights-on and lights-off both work)
  2. HSV threshold -> yellow mask and white mask, only below the horizon (ROI)
  3. Remove big bright blobs (windows / glare): painted lines are thin
  4. Look at a horizontal band near the bottom; line x = median x of its pixels
  5. Lane centre = midpoint of both lines (or one line +/- half lane width)
  6. error = lane centre offset from image centre, normalised to [-1, 1]
     (error > 0  ->  lane centre is to the RIGHT of the robot -> turn right)
"""
from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np


@dataclass
class DetectorParams:
    proc_width: int = 320                          # image is resized to this width
    yellow_h: Tuple[int, int] = (15, 40)           # OpenCV hue range (0-179)
    yellow_s_min: int = 70
    yellow_v_min: int = 70                         # on the normalised V channel
    white_s_max: int = 70
    white_v_min: int = 165                         # on the normalised V channel
    white_abs_v_min: int = 55                      # raw V floor: rejects noise in black areas
    min_contrast: int = 30                         # min (p99.5 - p5) of raw V to trust normalisation
    blob_size: int = 31                            # white blobs thicker than this (px) = glare, removed
    roi_top: float = 0.55                          # ignore everything above (fraction of height)
    band_top: float = 0.66                         # measurement band (fractions of height)
    band_bottom: float = 0.97
    min_pixels: int = 40                           # min mask pixels in band to trust a line
    half_lane: float = 0.28                        # half lane width / image width (single-line fallback)
    yellow_on_left: bool = True                    # False when driving the 'inverse' direction


@dataclass
class LaneResult:
    error: Optional[float]      # -1 .. +1 ; None = lane lost
    yellow_x: Optional[float]   # processed-image pixels
    white_x: Optional[float]
    mode: str                   # both | yellow | white | none
    debug: Optional[np.ndarray] = None


def _line_x(mask_band: np.ndarray, min_pixels: int) -> Optional[float]:
    """x of the line in the band: median x of the LARGEST connected piece.
    (a painted line is one long piece; noise / tile edges are small scattered pieces)"""
    n, lab, stats, _ = cv2.connectedComponentsWithStats(mask_band, connectivity=8)
    if n <= 1:
        return None
    areas = stats[1:, cv2.CC_STAT_AREA]
    best = int(np.argmax(areas)) + 1
    if areas[best - 1] < min_pixels:
        return None
    ys, xs = np.nonzero(lab == best)
    return float(np.median(xs))


def _remove_blobs(mask: np.ndarray, size: int) -> np.ndarray:
    """Remove regions that are too thick to be a painted line."""
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (size, size))
    core = cv2.erode(mask, k)
    if not core.any():
        return mask
    blobs = cv2.dilate(core, k)
    # grow back through the original mask so the whole blob is removed
    _, lab = cv2.connectedComponents(mask)
    bad = np.unique(lab[blobs > 0])
    bad = bad[bad > 0]
    out = mask.copy()
    out[np.isin(lab, bad)] = 0
    return out


def detect(bgr: np.ndarray, p: DetectorParams, draw: bool = True) -> LaneResult:
    h0, w0 = bgr.shape[:2]
    scale = p.proc_width / float(w0)
    img = cv2.resize(bgr, (p.proc_width, int(round(h0 * scale))), interpolation=cv2.INTER_AREA)
    h, w = img.shape[:2]
    top = int(h * p.roi_top)

    hsv = cv2.cvtColor(cv2.GaussianBlur(img, (3, 3), 0), cv2.COLOR_BGR2HSV)
    H, S, V = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]

    # --- brightness normalisation on the region below the horizon -------------
    roi_v = V[top:].astype(np.float32)
    lo, hi = np.percentile(roi_v, (5, 99.5))
    low_contrast = (hi - lo) < p.min_contrast
    if low_contrast:
        vn = V.astype(np.float32)
    else:
        vn = np.clip((V.astype(np.float32) - lo) / (hi - lo) * 255.0, 0, 255)

    yellow = ((H >= p.yellow_h[0]) & (H <= p.yellow_h[1]) &
              (S >= p.yellow_s_min) & (vn >= p.yellow_v_min) &
              (V >= p.white_abs_v_min)).astype(np.uint8) * 255
    white = ((S <= p.white_s_max) & (vn >= p.white_v_min) &
             (V >= p.white_abs_v_min)).astype(np.uint8) * 255
    if low_contrast:                      # nothing stands out -> no white line to trust
        white[:] = 0

    yellow[:top, :] = 0
    white[:top, :] = 0
    k = np.ones((3, 3), np.uint8)
    yellow = cv2.morphologyEx(yellow, cv2.MORPH_OPEN, k)
    white = cv2.morphologyEx(white, cv2.MORPH_OPEN, k)
    white = _remove_blobs(white, p.blob_size)

    b0, b1 = int(h * p.band_top), int(h * p.band_bottom)
    yx = _line_x(yellow[b0:b1], p.min_pixels)
    wx = _line_x(white[b0:b1], p.min_pixels)

    # sanity: left/right order must match the driving direction
    if yx is not None and wx is not None:
        ordered = yx < wx if p.yellow_on_left else wx < yx
        if not ordered:
            if np.count_nonzero(yellow[b0:b1]) >= np.count_nonzero(white[b0:b1]):
                wx = None
            else:
                yx = None

    half = p.half_lane * w
    sgn = 1.0 if p.yellow_on_left else -1.0
    if yx is not None and wx is not None:
        centre, mode = (yx + wx) / 2.0, "both"
    elif yx is not None:
        centre, mode = yx + sgn * half, "yellow"
    elif wx is not None:
        centre, mode = wx - sgn * half, "white"
    else:
        centre, mode = None, "none"

    error = None if centre is None else float(np.clip((centre - w / 2.0) / (w / 2.0), -1.0, 1.0))

    dbg = None
    if draw:
        dbg = img.copy()
        dbg[yellow > 0] = (0, 0, 255)       # yellow line drawn red
        dbg[white > 0] = (255, 120, 0)      # white line drawn blue
        cv2.rectangle(dbg, (0, b0), (w - 1, b1), (0, 255, 0), 1)
        cv2.line(dbg, (w // 2, b0), (w // 2, b1), (255, 255, 255), 1)
        if centre is not None:
            cv2.circle(dbg, (int(centre), (b0 + b1) // 2), 5, (0, 255, 0), -1)
        txt = "--" if error is None else "%+.2f" % error
        cv2.putText(dbg, "%s err=%s" % (mode, txt), (5, 18),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
    return LaneResult(error, yx, wx, mode, dbg)
