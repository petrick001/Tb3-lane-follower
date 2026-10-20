"""PD steering controller (pure Python, unit-testable)."""
from dataclasses import dataclass
from typing import Tuple


@dataclass
class ControlParams:
    kp: float = 1.2             # rad/s per unit of error
    kd: float = 0.15            # derivative gain
    v_max: float = 0.10         # m/s on straights (start slow, raise once it works)
    v_min: float = 0.04         # m/s in sharp corrections
    w_max: float = 1.0          # rad/s clamp
    slowdown: float = 0.8       # 0..1 : how much speed is lost at |error| = 1
    smooth: float = 0.5         # 0..1 : EMA weight on the NEW error (1 = no smoothing)


class PDController:
    def __init__(self, p: ControlParams):
        self.p = p
        self.prev_err = None
        self.filt_err = 0.0

    def reset(self):
        self.prev_err = None
        self.filt_err = 0.0

    def update(self, error: float, dt: float) -> Tuple[float, float]:
        """error > 0 means the lane centre is right of the robot -> turn right (negative w)."""
        p = self.p
        self.filt_err = p.smooth * error + (1.0 - p.smooth) * self.filt_err
        e = self.filt_err
        de = 0.0 if (self.prev_err is None or dt <= 1e-3) else (e - self.prev_err) / dt
        self.prev_err = e
        w = -(p.kp * e + p.kd * de)
        w = max(-p.w_max, min(p.w_max, w))
        v = p.v_max * (1.0 - p.slowdown * min(1.0, abs(e)))
        v = max(p.v_min, v)
        return v, w
