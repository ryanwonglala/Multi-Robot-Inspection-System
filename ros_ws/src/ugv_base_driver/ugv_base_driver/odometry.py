"""Pure differential-drive odometry used by the serial ROS node."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class OdometryUpdate:
    """One accepted encoder update."""

    x: float
    y: float
    yaw: float
    linear_velocity: float
    angular_velocity: float
    left_delta: float
    right_delta: float


class DifferentialOdometry:
    """Integrate cumulative left/right distance counters.

    The UGV controller reports cumulative ``odl`` and ``odr`` counters. On the
    commissioned chassis one count is approximately one centimetre; keeping the
    scale configurable allows later calibration without changing this class.
    """

    def __init__(
        self,
        meters_per_tick: float,
        track_width: float,
        max_tick_jump: int = 100,
    ):
        if meters_per_tick <= 0.0:
            raise ValueError('meters_per_tick must be positive')
        if track_width <= 0.0:
            raise ValueError('track_width must be positive')
        if max_tick_jump <= 0:
            raise ValueError('max_tick_jump must be positive')
        self.meters_per_tick = float(meters_per_tick)
        self.track_width = float(track_width)
        self.max_tick_jump = int(max_tick_jump)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self._left = None
        self._right = None
        self._time = None

    def update(
        self,
        left_ticks: int,
        right_ticks: int,
        monotonic_time: float,
    ) -> OdometryUpdate | None:
        """Integrate a sample, or re-baseline and return ``None``."""
        left_ticks = int(left_ticks)
        right_ticks = int(right_ticks)
        monotonic_time = float(monotonic_time)

        if self._left is None:
            self._set_baseline(left_ticks, right_ticks, monotonic_time)
            return None

        delta_left_ticks = left_ticks - self._left
        delta_right_ticks = right_ticks - self._right
        elapsed = monotonic_time - self._time
        self._set_baseline(left_ticks, right_ticks, monotonic_time)

        if elapsed <= 0.0:
            return None
        if (
            abs(delta_left_ticks) > self.max_tick_jump
            or abs(delta_right_ticks) > self.max_tick_jump
        ):
            return None

        left_delta = delta_left_ticks * self.meters_per_tick
        right_delta = delta_right_ticks * self.meters_per_tick
        distance = 0.5 * (left_delta + right_delta)
        yaw_delta = (right_delta - left_delta) / self.track_width
        middle_yaw = self.yaw + 0.5 * yaw_delta
        self.x += distance * math.cos(middle_yaw)
        self.y += distance * math.sin(middle_yaw)
        self.yaw = normalize_angle(self.yaw + yaw_delta)

        return OdometryUpdate(
            x=self.x,
            y=self.y,
            yaw=self.yaw,
            linear_velocity=distance / elapsed,
            angular_velocity=yaw_delta / elapsed,
            left_delta=left_delta,
            right_delta=right_delta,
        )

    def _set_baseline(self, left: int, right: int, sample_time: float):
        self._left = left
        self._right = right
        self._time = sample_time


def normalize_angle(angle: float) -> float:
    """Wrap radians to [-pi, pi]."""
    return math.atan2(math.sin(angle), math.cos(angle))
