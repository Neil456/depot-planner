"""Occupancy grid and driving cost map for the depot."""

from __future__ import annotations

from enum import IntEnum
from typing import Any

import numpy as np
from scipy.ndimage import distance_transform_edt

from depot_planner import core
from depot_planner.config import load_config


class CellType(IntEnum):
    """What a single grid cell is."""

    AISLE = 0      # drivable lane, cheap
    PARKING = 1    # drivable parking area, more expensive
    OBSTACLE = 2   # wall, curb or building: not drivable


class Grid:
    """A 2D occupancy grid with a driving cost map.

    Cells are addressed as ``(x, y)`` pairs; the underlying arrays are indexed
    ``[y, x]`` in the usual row/column order.
    """

    def __init__(
        self,
        cells: np.ndarray,
        resolution: float | None = None,
        config: dict[str, Any] | None = None,
    ) -> None:
        self.config = config if config is not None else load_config("grid")
        self.cells = np.asarray(cells, dtype=np.uint8)
        if self.cells.ndim != 2:
            raise ValueError("cells must be a 2D array")
        self.resolution = float(
            resolution if resolution is not None else self.config["resolution"]
        )
        self._cost_map: np.ndarray | None = None

    # ------------------------------------------------------------------ shape

    @property
    def height(self) -> int:
        """Number of rows."""
        return int(self.cells.shape[0])

    @property
    def width(self) -> int:
        """Number of columns."""
        return int(self.cells.shape[1])

    @property
    def shape(self) -> tuple[int, int]:
        """``(height, width)`` in cells."""
        return self.height, self.width

    def in_bounds(self, x: int, y: int) -> bool:
        """True if ``(x, y)`` is inside the grid."""
        return 0 <= x < self.width and 0 <= y < self.height

    # --------------------------------------------------------------- drivable

    @property
    def drivable(self) -> np.ndarray:
        """Boolean mask of cells a vehicle may occupy."""
        return self.cells != CellType.OBSTACLE

    def is_drivable(self, x: int, y: int) -> bool:
        """True if ``(x, y)`` is in bounds and not an obstacle."""
        return self.in_bounds(x, y) and bool(self.cells[y, x] != CellType.OBSTACLE)

    # -------------------------------------------------------------- cost maps

    def base_cost_map(self) -> np.ndarray:
        """Per-cell terrain cost, ``inf`` on obstacles, without the soft penalty."""
        costs = self.config["cell_cost"]
        cost = np.full(self.shape, np.inf, dtype=float)
        cost[self.cells == CellType.AISLE] = float(costs["aisle"])
        cost[self.cells == CellType.PARKING] = float(costs["parking"])
        return cost

    def obstacle_distance(self, backend: str | None = None) -> np.ndarray:
        """Distance in metres from every cell to the nearest non-drivable cell.

        Cells outside the grid are treated as free, which is the conservative
        choice: it never inflates the penalty of a cell near the border more
        than the walls themselves already do.

        The C++ core computes the exact transform in integer arithmetic, so it
        agrees with ``scipy.ndimage.distance_transform_edt`` to the last bit on
        every map that has at least one obstacle. A map with no obstacle at all
        has no nearest obstacle: the core reports infinity, while scipy returns
        an artefact of its own algorithm (see docs/DECISIONS.md).
        """
        drivable = self.drivable
        if core.resolve(backend) == "cpp":
            return core.require().obstacle_distance(
                np.ascontiguousarray(drivable), self.resolution
            )
        return distance_transform_edt(drivable) * self.resolution

    def proximity_penalty(self, backend: str | None = None) -> np.ndarray:
        """Soft extra cost that decays with distance from obstacles."""
        cfg = self.config["obstacle_penalty"]
        penalty = np.zeros(self.shape, dtype=float)
        if not cfg.get("enabled", True):
            return penalty
        radius = float(cfg["radius"])
        if radius <= 0.0:
            return penalty
        weight = float(cfg["weight"])
        exponent = float(cfg["exponent"])
        dist = self.obstacle_distance(backend)
        near = (dist > 0.0) & (dist < radius)
        penalty[near] = weight * (1.0 - dist[near] / radius) ** exponent
        return penalty

    def cost_map(self) -> np.ndarray:
        """Terrain cost plus obstacle-proximity penalty; ``inf`` on obstacles."""
        if self._cost_map is None:
            cost = self.base_cost_map()
            drivable = self.drivable
            cost[drivable] += self.proximity_penalty()[drivable]
            self._cost_map = cost
        return self._cost_map

    def min_cell_cost(self) -> float:
        """Cheapest drivable cell; the scale factor that keeps A*'s h admissible."""
        cost = self.cost_map()
        drivable = self.drivable
        if not drivable.any():
            return 0.0
        return float(cost[drivable].min())

    # ------------------------------------------------------------------ misc

    def copy(self) -> "Grid":
        """An independent copy sharing the same config."""
        return Grid(self.cells.copy(), self.resolution, self.config)

    def __repr__(self) -> str:  # pragma: no cover - debugging aid
        return f"Grid({self.width}x{self.height} @ {self.resolution} m)"
