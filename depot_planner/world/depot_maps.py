"""Deterministic generator for depot layouts.

A depot is a rectangular lot enclosed by walls, with a few horizontal driving
aisles, parking bands between them, vertical cross aisles connecting the
horizontal ones, curb islands inside the deeper parking bands, and entry/exit
gaps in the left and right walls.

Everything is driven by a ``numpy.random.Generator`` seeded from ``seed``, so
the same seed always produces the same layout.
"""

from __future__ import annotations

from typing import Any

import numpy as np

from depot_planner.config import load_config
from depot_planner.world.grid import CellType, Grid


def _band_depths(total: int, count: int) -> list[int]:
    """Split ``total`` rows into ``count`` bands as evenly as possible."""
    base, extra = divmod(total, count)
    return [base + (1 if i < extra else 0) for i in range(count)]


def depot_layout(seed: int = 0, config: dict[str, Any] | None = None) -> dict[str, Any]:
    """Return the structural description of a depot layout for ``seed``."""
    cfg = config if config is not None else load_config("depot")
    rng = np.random.default_rng(seed)

    width = int(cfg["width"])
    height = int(cfg["height"])
    wall = int(cfg["wall_thickness"])
    aisle_cfg = cfg["aisles"]
    cross_cfg = cfg["cross_aisles"]

    aisle_width = int(aisle_cfg["width"])
    n_aisles = int(rng.integers(int(aisle_cfg["min_count"]), int(aisle_cfg["max_count"]) + 1))
    n_cross = int(rng.integers(int(cross_cfg["min_count"]), int(cross_cfg["max_count"]) + 1))
    cross_width = int(cross_cfg["width"])

    interior_top = wall
    interior_height = height - 2 * wall
    spare = interior_height - n_aisles * aisle_width
    if spare < 0:
        raise ValueError("depot is too short for the requested aisles")

    # Parking bands sit above, between and below the aisles.
    bands = _band_depths(spare, n_aisles + 1)

    aisle_rows: list[tuple[int, int]] = []
    band_rows: list[tuple[int, int]] = []
    y = interior_top
    for index in range(n_aisles + 1):
        depth = bands[index]
        if depth > 0:
            band_rows.append((y, y + depth))
        y += depth
        if index < n_aisles:
            aisle_rows.append((y, y + aisle_width))
            y += aisle_width

    interior_left = wall
    interior_right = width - wall
    usable = interior_right - interior_left
    cross_cols: list[tuple[int, int]] = []
    for index in range(n_cross):
        centre = interior_left + int(round(usable * (index + 1) / (n_cross + 1)))
        x0 = max(interior_left, centre - cross_width // 2)
        x1 = min(interior_right, x0 + cross_width)
        cross_cols.append((x0, x1))

    return {
        "width": width,
        "height": height,
        "wall": wall,
        "aisle_rows": aisle_rows,
        "band_rows": band_rows,
        "cross_cols": cross_cols,
        "curb_offset": int(rng.integers(0, int(cfg["curb"]["gap_period"]))),
        "config": cfg,
    }


def generate_depot(seed: int = 0, config: dict[str, Any] | None = None) -> Grid:
    """Build a depot :class:`Grid` for ``seed``."""
    layout = depot_layout(seed, config)
    cfg = layout["config"]
    width, height, wall = layout["width"], layout["height"], layout["wall"]

    cells = np.full((height, width), CellType.PARKING, dtype=np.uint8)

    # Enclosing walls.
    cells[:wall, :] = CellType.OBSTACLE
    cells[height - wall :, :] = CellType.OBSTACLE
    cells[:, :wall] = CellType.OBSTACLE
    cells[:, width - wall :] = CellType.OBSTACLE

    # Curb islands down the middle of the deeper parking bands.
    curb_cfg = cfg["curb"]
    min_depth = int(curb_cfg["min_band_depth"])
    gap_period = int(curb_cfg["gap_period"])
    offset = layout["curb_offset"]
    for y0, y1 in layout["band_rows"]:
        if y1 - y0 < min_depth:
            continue
        curb_y = (y0 + y1) // 2
        for x in range(wall, width - wall):
            if (x + offset) % gap_period == 0:
                continue
            cells[curb_y, x] = CellType.OBSTACLE

    # Horizontal driving aisles.
    for y0, y1 in layout["aisle_rows"]:
        cells[y0:y1, wall : width - wall] = CellType.AISLE

    # Vertical cross aisles connect the horizontal ones (and cut the curbs).
    for x0, x1 in layout["cross_cols"]:
        cells[wall : height - wall, x0:x1] = CellType.AISLE

    # Entry gap on the left at the first aisle, exit gap on the right at the last.
    first_aisle = layout["aisle_rows"][0]
    last_aisle = layout["aisle_rows"][-1]
    cells[first_aisle[0] : first_aisle[1], :wall] = CellType.AISLE
    cells[last_aisle[0] : last_aisle[1], width - wall :] = CellType.AISLE

    return Grid(cells)


def aisle_cells(grid: Grid) -> np.ndarray:
    """``(N, 2)`` array of ``(x, y)`` aisle cells, useful for placing vehicles."""
    ys, xs = np.nonzero(grid.cells == CellType.AISLE)
    return np.stack([xs, ys], axis=1)


def drivable_cells(grid: Grid) -> np.ndarray:
    """``(N, 2)`` array of ``(x, y)`` drivable cells."""
    ys, xs = np.nonzero(grid.drivable)
    return np.stack([xs, ys], axis=1)


def connected_component(grid: Grid, seed_cell: tuple[int, int] | None = None) -> np.ndarray:
    """Boolean mask of drivable cells reachable from ``seed_cell`` (8-connected)."""
    from depot_planner.grid_astar.search import dijkstra_field

    if seed_cell is None:
        cells = aisle_cells(grid)
        if len(cells) == 0:
            cells = drivable_cells(grid)
        seed_cell = (int(cells[0][0]), int(cells[0][1]))
    field = dijkstra_field(grid.cost_map(), [seed_cell], drivable=grid.drivable)
    return np.isfinite(field)


def sample_start_goal_pairs(
    grid: Grid,
    count: int,
    rng: np.random.Generator,
    min_separation: float = 15.0,
) -> list[tuple[tuple[int, int], tuple[int, int]]]:
    """Sample ``count`` mutually reachable ``(start, goal)`` cell pairs."""
    reachable = connected_component(grid)
    ys, xs = np.nonzero(reachable)
    if len(xs) < 2:
        raise ValueError("map has no reachable cells to sample from")
    pairs: list[tuple[tuple[int, int], tuple[int, int]]] = []
    while len(pairs) < count:
        i, j = rng.integers(0, len(xs), size=2)
        start = (int(xs[i]), int(ys[i]))
        goal = (int(xs[j]), int(ys[j]))
        if np.hypot(start[0] - goal[0], start[1] - goal[1]) < min_separation:
            continue
        pairs.append((start, goal))
    return pairs
