"""Continuous collision checking for the car against a Euclidean distance field."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Iterable, Sequence

import numpy as np
from scipy.ndimage import distance_transform_edt

from depot_planner.config import load_config
from depot_planner.hybrid.car import CarModel, Pose

Rectangle = tuple[float, float, float, float]  # x0, y0, x1, y1 in metres


@dataclass
class DistanceField:
    """A conservative lower bound on the distance to the nearest obstacle.

    ``distance_transform_edt`` measures from cell *centre* to cell *centre*, and
    lookups snap a query point to the cell containing it. Both introduce error,
    each bounded by half a cell diagonal, so the raw transform can report up to
    one cell diagonal more clearance than really exists — enough for a planned
    path to clip an obstacle corner. Subtracting ``sqrt(2)`` cells makes the
    stored value a true lower bound on the distance from any point in a cell to
    any point of the (conservatively rasterised) obstacle region.

    Points outside the field are treated as obstacles (distance 0), which keeps
    the car inside the modelled world.
    """

    distance: np.ndarray       # (rows, cols), metres
    resolution: float          # metres per sample
    width_m: float
    height_m: float

    @classmethod
    def from_rectangles(
        cls,
        rectangles: Sequence[Rectangle],
        width_m: float,
        height_m: float,
        resolution: float = 0.1,
    ) -> "DistanceField":
        cols = int(round(width_m / resolution))
        rows = int(round(height_m / resolution))
        occupied = np.zeros((rows, cols), dtype=bool)
        for x0, y0, x1, y1 in rectangles:
            i0 = max(0, int(math.floor(min(y0, y1) / resolution)))
            i1 = min(rows, int(math.ceil(max(y0, y1) / resolution)))
            j0 = max(0, int(math.floor(min(x0, x1) / resolution)))
            j1 = min(cols, int(math.ceil(max(x0, x1) / resolution)))
            if i1 > i0 and j1 > j0:
                occupied[i0:i1, j0:j1] = True
        free = ~occupied
        cells = distance_transform_edt(free)
        distance = np.maximum(cells - math.sqrt(2.0), 0.0) * resolution
        return cls(distance, resolution, width_m, height_m)

    @property
    def shape(self) -> tuple[int, int]:
        return self.distance.shape

    def distance_at(self, xs: np.ndarray, ys: np.ndarray) -> np.ndarray:
        """Nearest-sample distance lookup; 0 outside the field."""
        xs = np.asarray(xs, dtype=float)
        ys = np.asarray(ys, dtype=float)
        rows, cols = self.distance.shape
        j = np.floor(xs / self.resolution).astype(int)
        i = np.floor(ys / self.resolution).astype(int)
        inside = (i >= 0) & (i < rows) & (j >= 0) & (j < cols)
        out = np.zeros(xs.shape, dtype=float)
        if inside.any():
            out[inside] = self.distance[i[inside], j[inside]]
        return out

    def occupied_mask(self) -> np.ndarray:
        return self.distance <= 0.0


class CarCollisionChecker:
    """Checks car poses by covering the footprint with discs."""

    def __init__(
        self,
        field: DistanceField,
        car: CarModel | None = None,
        config: dict[str, Any] | None = None,
    ) -> None:
        cfg = config if config is not None else load_config("hybrid")
        self.field = field
        self.car = car if car is not None else CarModel.from_config(cfg)
        self.margin = float(cfg["collision"]["safety_margin"])
        self.clearance = self.car.disc_radius + self.margin
        self.checks = 0

    def poses_are_free(self, poses: np.ndarray) -> np.ndarray:
        """Vectorised check of many poses at once.

        ``poses`` has shape ``(..., 3)``; the result is a boolean array of shape
        ``poses.shape[:-1]``. One distance-field lookup covers every disc of
        every pose, which is what keeps the search affordable.
        """
        array = np.asarray(poses, dtype=float)
        flat = array.reshape(-1, 3)
        self.checks += flat.shape[0]
        offsets = np.asarray(self.car.disc_offsets, dtype=float)[None, :]
        cos_t = np.cos(flat[:, 2])[:, None]
        sin_t = np.sin(flat[:, 2])[:, None]
        centres_x = flat[:, 0][:, None] + offsets * cos_t
        centres_y = flat[:, 1][:, None] + offsets * sin_t
        distances = self.field.distance_at(centres_x, centres_y)
        free = (distances >= self.clearance).all(axis=1)
        return free.reshape(array.shape[:-1])

    def is_free(self, pose: Pose) -> bool:
        """True if the car at ``pose`` clears every obstacle."""
        return bool(self.poses_are_free(np.asarray([pose], dtype=float))[0])

    def path_is_free(self, poses: Iterable[Pose]) -> bool:
        array = np.asarray(list(poses), dtype=float)
        if array.size == 0:
            return True
        return bool(self.poses_are_free(array).all())

    def first_collision(self, poses: Iterable[Pose]) -> int | None:
        array = np.asarray(list(poses), dtype=float)
        if array.size == 0:
            return None
        free = self.poses_are_free(array)
        bad = np.flatnonzero(~free)
        return int(bad[0]) if len(bad) else None
