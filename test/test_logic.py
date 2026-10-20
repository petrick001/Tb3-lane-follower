"""Offline tests (no ROS needed):  python3 -m pytest test/ -q"""
import numpy as np
import cv2

from tb3_lane_follower.controller import ControlParams, PDController
from tb3_lane_follower.lane_detector import DetectorParams, detect


def synthetic(yx, wx, w=320, h=240):
    """black floor, yellow line at x=yx and white line at x=wx in the lower part."""
    img = np.zeros((h, w, 3), np.uint8)
    if yx is not None:
        cv2.line(img, (yx, h), (yx, int(h * 0.6)), (0, 220, 255), 8)   # BGR yellow
    if wx is not None:
        cv2.line(img, (wx, h), (wx, int(h * 0.6)), (255, 255, 255), 8)
    return img


def test_centered_lane_gives_zero_error():
    r = detect(synthetic(60, 260), DetectorParams())
    assert r.mode == 'both' and abs(r.error) < 0.05


def test_lane_right_of_robot_gives_positive_error():
    r = detect(synthetic(150, 310), DetectorParams())
    assert r.error > 0.2


def test_lane_left_of_robot_gives_negative_error():
    r = detect(synthetic(10, 170), DetectorParams())
    assert r.error < -0.2


def test_single_line_fallback():
    assert detect(synthetic(60, None), DetectorParams()).mode == 'yellow'
    assert detect(synthetic(None, 260), DetectorParams()).mode == 'white'


def test_nothing_visible():
    r = detect(synthetic(None, None), DetectorParams())
    assert r.mode == 'none' and r.error is None


def test_controller_turns_toward_lane():
    c = PDController(ControlParams())
    v, w = c.update(+0.5, 0.05)          # lane to the right -> negative w (turn right)
    assert w < 0 and v > 0
    c.reset()
    v, w = c.update(-0.5, 0.05)
    assert w > 0


def test_controller_slows_in_corrections_and_clamps():
    p = ControlParams()
    c = PDController(p)
    v0, _ = c.update(0.0, 0.05)
    c.reset()
    v1, w1 = c.update(1.0, 0.05)
    assert v1 < v0 and abs(w1) <= p.w_max
