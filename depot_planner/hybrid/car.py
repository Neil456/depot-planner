"""Kinematic bicycle car model and its footprint.

Coordinates follow the rest of the project: ``x`` grows right, ``y`` grows
*down*, and the heading ``theta`` is measured from the ``+x`` axis towards
``+y``. The reference point is the centre of the rear axle.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Iterable

import numpy as np

from depot_planner.config import load_config

Pose = tuple[float, float, float]

EPSILON = 1e-9


def wrap_angle(theta: float) -> float:
    """Wrap an angle to ``(-pi, pi]``."""
    wrapped = math.fmod(theta, 2.0 * math.pi)
    if wrapped > math.pi:
        wrapped -= 2.0 * math.pi
    elif wrapped <= -math.pi:
        wrapped += 2.0 * math.pi
    return wrapped


def angle_difference(a: float, b: float) -> float:
    """Shortest signed difference ``a - b``, wrapped to ``(-pi, pi]``."""
    return wrap_angle(a - b)


@dataclass(frozen=True)
class CarModel:
    """Geometry and kinematics of the ego car."""

    wheelbase: float = 2.7
    length: float = 4.5
    width: float = 1.9
    max_steer: float = 0.6
    rear_overhang: float = 0.9
    n_discs: int = 4

    @classmethod
    def from_config(cls, config: dict[str, Any] | None = None) -> "CarModel":
        cfg = (config if config is not None else load_config("hybrid"))["car"]
        return cls(
            wheelbase=float(cfg["wheelbase"]),
            length=float(cfg["length"]),
            width=float(cfg["width"]),
            max_steer=float(cfg["max_steer"]),
            rear_overhang=float(cfg["rear_overhang"]),
            n_discs=int(cfg["n_discs"]),
        )

    # ----------------------------------------------------------------- shape

    @property
    def front_overhang(self) -> float:
        return self.length - self.rear_overhang

    @property
    def min_turning_radius(self) -> float:
        return self.wheelbase / math.tan(self.max_steer)

    @property
    def max_curvature(self) -> float:
        return 1.0 / self.min_turning_radius

    @property
    def disc_radius(self) -> float:
        """Radius of each covering disc: the half-diagonal of one footprint slice."""
        half_slice = self.length / (2.0 * self.n_discs)
        return math.hypot(half_slice, self.width / 2.0)

    @property
    def disc_offsets(self) -> tuple[float, ...]:
        """Longitudinal offsets of the disc centres from the rear axle."""
        slice_length = self.length / self.n_discs
        return tuple(
            -self.rear_overhang + (index + 0.5) * slice_length for index in range(self.n_discs)
        )

    def disc_centres(self, x: float, y: float, theta: float) -> np.ndarray:
        """``(n_discs, 2)`` array of disc centres for a pose."""
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        offsets = np.asarray(self.disc_offsets, dtype=float)
        return np.stack([x + offsets * cos_t, y + offsets * sin_t], axis=1)

    def corners(self, x: float, y: float, theta: float) -> np.ndarray:
        """``(4, 2)`` array of footprint corners, anticlockwise from the rear right."""
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        half = self.width / 2.0
        local = np.array([
            [-self.rear_overhang, -half],
            [self.front_overhang, -half],
            [self.front_overhang, half],
            [-self.rear_overhang, half],
        ])
        rotation = np.array([[cos_t, -sin_t], [sin_t, cos_t]])
        return local @ rotation.T + np.array([x, y])

    # ------------------------------------------------------------ kinematics

    def step(self, pose: Pose, steer: float, distance: float) -> Pose:
        """Exact integration of one constant-steering arc of signed ``distance``."""
        x, y, theta = pose
        curvature = math.tan(steer) / self.wheelbase
        if abs(curvature) < EPSILON:
            return x + distance * math.cos(theta), y + distance * math.sin(theta), theta
        new_theta = theta + curvature * distance
        new_x = x + (math.sin(new_theta) - math.sin(theta)) / curvature
        new_y = y - (math.cos(new_theta) - math.cos(theta)) / curvature
        return new_x, new_y, wrap_angle(new_theta)

    def sample_arc(self, pose: Pose, steer: float, distance: float, substeps: int) -> list[Pose]:
        """Poses along one arc, excluding the start and including the end."""
        step_distance = distance / substeps
        poses: list[Pose] = []
        current = pose
        for _ in range(substeps):
            current = self.step(current, steer, step_distance)
            poses.append(current)
        return poses

    def steer_angles(self, fractions: Iterable[float]) -> tuple[float, ...]:
        return tuple(float(f) * self.max_steer for f in fractions)
