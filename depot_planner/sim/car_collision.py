"""Independent collision checker for car-shaped paths.

Like :mod:`depot_planner.sim.collision` this shares no code with the planner.
Where the planner covers the car with discs and queries a sampled distance
field, this checker uses the **exact** car rectangle and the **exact** obstacle
rectangles, and separates them with the separating-axis test. It is therefore a
genuine second opinion: the planner's disc model is conservative, so anything
this checker rejects is a planner bug.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np

Pose = tuple[float, float, float]
Rectangle = tuple[float, float, float, float]

CONTACT_TOLERANCE = 1e-9


@dataclass
class CarCollisionReport:
    """Verdict on one interpolated car path."""

    ok: bool
    pose: Pose | None = None
    index: int | None = None
    obstacle: Rectangle | None = None

    def describe(self) -> str:
        if self.ok:
            return "no collision"
        x, y, theta = self.pose
        return (
            f"car at ({x:.2f}, {y:.2f}, {math.degrees(theta):.1f} deg) overlaps "
            f"obstacle {tuple(round(v, 2) for v in self.obstacle)} at sample {self.index}"
        )


def car_corners(pose: Pose, length: float, width: float, rear_overhang: float) -> np.ndarray:
    """The four corners of the car rectangle, derived here from first principles."""
    x, y, theta = pose
    cos_t, sin_t = math.cos(theta), math.sin(theta)
    half = width / 2.0
    front = length - rear_overhang
    local = ((-rear_overhang, -half), (front, -half), (front, half), (-rear_overhang, half))
    return np.array([(x + lx * cos_t - ly * sin_t, y + lx * sin_t + ly * cos_t) for lx, ly in local])


def _overlaps(corners: np.ndarray, rectangle: Rectangle) -> bool:
    """Separating-axis test between a rotated rectangle and an axis-aligned one."""
    x0, y0, x1, y1 = rectangle
    box = np.array([(x0, y0), (x1, y0), (x1, y1), (x0, y1)])

    axes = [(1.0, 0.0), (0.0, 1.0)]
    for index in range(2):  # two unique edge normals of the rotated rectangle
        edge = corners[index + 1] - corners[index]
        axes.append((-edge[1], edge[0]))

    for ax, ay in axes:
        norm = math.hypot(ax, ay)
        if norm < 1e-12:
            continue
        ax, ay = ax / norm, ay / norm
        car_projection = corners[:, 0] * ax + corners[:, 1] * ay
        box_projection = box[:, 0] * ax + box[:, 1] * ay
        if (car_projection.max() <= box_projection.min() + CONTACT_TOLERANCE
                or box_projection.max() <= car_projection.min() + CONTACT_TOLERANCE):
            return False
    return True


def densify(poses: Sequence[Pose], step: float = 0.05) -> list[Pose]:
    """Interpolate a path so consecutive samples are at most ``step`` metres apart."""
    if len(poses) < 2:
        return list(poses)
    dense: list[Pose] = [tuple(poses[0])]
    for start, end in zip(poses, poses[1:]):
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        turn = abs(math.atan2(math.sin(end[2] - start[2]), math.cos(end[2] - start[2])))
        pieces = max(1, int(math.ceil(max(distance, turn) / step)))
        delta_theta = math.atan2(math.sin(end[2] - start[2]), math.cos(end[2] - start[2]))
        for piece in range(1, pieces + 1):
            fraction = piece / pieces
            dense.append((
                start[0] + fraction * (end[0] - start[0]),
                start[1] + fraction * (end[1] - start[1]),
                start[2] + fraction * delta_theta,
            ))
    return dense


def check_car_path(
    poses: Sequence[Pose],
    obstacles: Sequence[Rectangle],
    length: float,
    width: float,
    rear_overhang: float,
    step: float = 0.05,
) -> CarCollisionReport:
    """Densely interpolate ``poses`` and reject any sample whose body overlaps an obstacle."""
    for index, pose in enumerate(densify(poses, step)):
        corners = car_corners(pose, length, width, rear_overhang)
        for rectangle in obstacles:
            if _overlaps(corners, rectangle):
                return CarCollisionReport(False, pose, index, rectangle)
    return CarCollisionReport(True)


def check_plan(scenario, result, step: float = 0.05) -> CarCollisionReport:
    """Convenience wrapper for a :class:`ParkingScenario` and a hybrid A* result."""
    car = scenario.hybrid_config["car"]
    return check_car_path(
        result.poses, scenario.obstacles,
        float(car["length"]), float(car["width"]), float(car["rear_overhang"]), step,
    )
