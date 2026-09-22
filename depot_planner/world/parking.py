"""Seeded parking scenarios: static parked cars, one free spot, one goal pose.

Four types are produced:

``perpendicular_forward``  pull forward into a spot between parked cars
``perpendicular_reverse``  back into a spot (the goal heading requires reverse)
``parallel``               parallel park into a gap of varying length
``tight``                  any of the above with minimal clearances

There are no moving agents in this step; every obstacle is a static rectangle.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from depot_planner.config import deep_merge, load_config
from depot_planner.grid_astar.search import dijkstra_field
from depot_planner.hybrid.car import CarModel, Pose
from depot_planner.hybrid.collision import CarCollisionChecker, DistanceField, Rectangle

PARKING_TYPES: tuple[str, ...] = (
    "perpendicular_forward",
    "perpendicular_reverse",
    "parallel",
    "tight",
)

BASE_LAYOUTS: tuple[str, ...] = (
    "perpendicular_forward",
    "perpendicular_reverse",
    "parallel",
)

#: Harder variants, defined in ``configs/parking.yaml`` under ``hard:``.
HARD_PARKING_TYPES: tuple[str, ...] = (
    "parallel_minimal",
    "perpendicular_minimal",
)

ALL_PARKING_TYPES: tuple[str, ...] = PARKING_TYPES + HARD_PARKING_TYPES


class ParkingGenerationError(RuntimeError):
    """Raised when a parking scenario could not be generated."""


@dataclass
class ParkingScenario:
    """One parking problem: a walled lot, parked cars, a start pose and a goal pose."""

    name: str
    seed: int
    layout: str                 # the base layout actually used
    width_m: float
    height_m: float
    obstacles: list[Rectangle]
    start: Pose
    goal: Pose
    requires_reverse: bool
    metrics: dict[str, float] = field(default_factory=dict)
    config: dict[str, Any] = field(default_factory=dict)
    hybrid_config: dict[str, Any] = field(default_factory=dict)
    _field: DistanceField | None = field(default=None, repr=False)
    _heuristic: tuple[np.ndarray, np.ndarray, float] | None = field(default=None, repr=False)

    # -------------------------------------------------------------- geometry

    def distance_field(self) -> DistanceField:
        if self._field is None:
            resolution = float(self.hybrid_config["collision"]["field_resolution"])
            self._field = DistanceField.from_rectangles(
                self.obstacles, self.width_m, self.height_m, resolution
            )
        return self._field

    def checker(self, car: CarModel | None = None) -> CarCollisionChecker:
        return CarCollisionChecker(self.distance_field(), car, self.hybrid_config)

    def heuristic_grid(self) -> tuple[np.ndarray, np.ndarray, float]:
        """``(cost_map, drivable, resolution)`` of the coarse grid for the heuristic.

        A coarse cell is drivable when its centre is not inside an obstacle. That
        is the optimistic choice: the car's reference point sits inside its own
        footprint, so it can never be in an obstacle cell, and marking fewer
        cells as blocked keeps the resulting distance field a lower bound.
        """
        if self._heuristic is None:
            resolution = float(self.hybrid_config["heuristic"]["grid_resolution"])
            cols = max(1, int(round(self.width_m / resolution)))
            rows = max(1, int(round(self.height_m / resolution)))
            xs = (np.arange(cols) + 0.5) * resolution
            ys = (np.arange(rows) + 0.5) * resolution
            mesh_x, mesh_y = np.meshgrid(xs, ys)
            distances = self.distance_field().distance_at(mesh_x, mesh_y)
            drivable = distances > 0.0
            cost_map = np.where(drivable, 1.0, np.inf)
            self._heuristic = (cost_map, drivable, resolution)
        return self._heuristic

    def to_cell(self, x: float, y: float) -> tuple[int, int]:
        _, _, resolution = self.heuristic_grid()
        return int(x // resolution), int(y // resolution)

    def goal_distance_field(self) -> np.ndarray:
        """Obstacle-aware distance in metres from every coarse cell to the goal."""
        cost_map, drivable, resolution = self.heuristic_grid()
        goal_cell = self.to_cell(self.goal[0], self.goal[1])
        return dijkstra_field(cost_map, [goal_cell], drivable=drivable) * resolution

    def summary(self) -> dict[str, Any]:
        return {
            "parking_type": self.name,
            "layout": self.layout,
            "seed": self.seed,
            "size_m": (round(self.width_m, 2), round(self.height_m, 2)),
            "parked_cars": len(self.obstacles) - 4,
            "start": tuple(round(v, 3) for v in self.start),
            "goal": tuple(round(v, 3) for v in self.goal),
            "requires_reverse": self.requires_reverse,
            **{key: round(value, 3) for key, value in self.metrics.items()},
        }


def clearance_requirement(car: CarModel, hybrid_config: dict[str, Any]) -> float:
    """Distance a disc centre must keep from an obstacle for the planner to accept it.

    That is the disc radius, plus the configured safety margin, plus the
    conservatism the distance field builds in (``sqrt(2)`` cells).
    """
    collision = hybrid_config["collision"]
    return (
        car.disc_radius
        + float(collision["safety_margin"])
        + math.sqrt(2.0) * float(collision["field_resolution"])
    )


def minimum_parallel_gap(car: CarModel, hybrid_config: dict[str, Any]) -> float:
    """Shortest parallel gap in which the goal pose is collision free.

    The car sits centred in the gap; its outermost disc centre is
    ``length / (2 * n_discs)`` from the bumper, and that centre has to clear the
    neighbouring car by :func:`clearance_requirement`.
    """
    required = clearance_requirement(car, hybrid_config)
    end_offset = car.length / (2.0 * car.n_discs)
    return car.length + 2.0 * max(0.0, required - end_offset)


def minimum_spot_width(car: CarModel, hybrid_config: dict[str, Any]) -> float:
    """Narrowest perpendicular spot in which the goal pose is collision free.

    Disc centres lie on the car's centreline, which is ``width - car.width / 2``
    from the neighbouring car's near edge when every spot holds a centred car.
    """
    return clearance_requirement(car, hybrid_config) + car.width / 2.0


def resolve_parking_type(
    name: str,
    car: CarModel,
    config: dict[str, Any],
    hybrid_config: dict[str, Any],
) -> tuple[str, dict[str, Any]]:
    """Return ``(base_type, config)`` for a normal or hard parking type."""
    if name in PARKING_TYPES:
        return name, config
    hard = config.get("hard", {})
    if name not in hard:
        raise ValueError(f"unknown parking type {name!r}; expected one of {ALL_PARKING_TYPES}")
    spec = hard[name]
    base = str(spec["base"])
    if base not in PARKING_TYPES:
        raise ValueError(f"hard parking type {name!r} names an unknown base type {base!r}")

    overrides: dict[str, Any] = {}
    section = "perpendicular" if base.startswith("perpendicular") else "parallel"
    if spec.get("apply_tight_overrides"):
        tight = {k: v for k, v in config["tight"].items() if k in config[section]}
        overrides[section] = dict(tight)
    else:
        overrides[section] = {}

    if "gap_above_minimum" in spec:
        low, high = spec["gap_above_minimum"]
        minimum = minimum_parallel_gap(car, hybrid_config)
        overrides[section]["gap"] = [minimum + float(low), minimum + float(high)]
    if "spot_width_above_minimum" in spec:
        low, high = spec["spot_width_above_minimum"]
        minimum = minimum_spot_width(car, hybrid_config)
        overrides[section]["spot_width"] = [minimum + float(low), minimum + float(high)]
    return base, deep_merge(config, overrides)


# --------------------------------------------------------------------- pieces


def _sample_dimension(value: Any, rng: np.random.Generator) -> float:
    """A layout dimension may be a fixed number or a ``[low, high]`` band."""
    if isinstance(value, (list, tuple)):
        return float(rng.uniform(float(value[0]), float(value[1])))
    return float(value)


def _walls(width: float, height: float, thickness: float) -> list[Rectangle]:
    return [
        (0.0, 0.0, width, thickness),
        (0.0, height - thickness, width, height),
        (0.0, 0.0, thickness, height),
        (width - thickness, 0.0, width, height),
    ]


def _settings(cfg: dict[str, Any], layout: str, tight: bool) -> dict[str, Any]:
    """Layout parameters, with the tight overrides folded in when asked."""
    base = dict(cfg["perpendicular"] if layout.startswith("perpendicular") else cfg["parallel"])
    if tight:
        for key, value in cfg["tight"].items():
            if key in base:
                base[key] = value
    return base


def _perpendicular_layout(
    rng: np.random.Generator, cfg: dict[str, Any], car: CarModel, reverse: bool, tight: bool
) -> tuple[float, float, list[Rectangle], Pose, int, float, dict[str, float]]:
    settings = _settings(cfg, "perpendicular", tight)
    wall = float(cfg["world"]["wall_thickness"])
    spot_width = _sample_dimension(settings["spot_width"], rng)
    spot_depth = float(settings["spot_depth"])
    aisle = float(settings["aisle_width"])
    clearance = float(settings["nose_clearance"])
    fill = float(settings["fill_probability"])
    low, high = settings["spot_count"]
    count = int(rng.integers(int(low), int(high) + 1))

    width = 2.0 * wall + count * spot_width
    height = 2.0 * wall + 2.0 * spot_depth + aisle
    obstacles = _walls(width, height, wall)

    top_row_end = wall + spot_depth
    bottom_row_start = top_row_end + aisle
    target = int(rng.integers(1, count - 1))  # never the outermost spot

    def spot_x(index: int) -> float:
        return wall + index * spot_width

    for index in range(count):
        inset = (spot_width - car.width) / 2.0
        x0 = spot_x(index) + inset
        x1 = x0 + car.width
        if index != target and rng.random() < fill:
            obstacles.append((x0, wall + clearance, x1, wall + clearance + car.length))
        if rng.random() < fill:
            obstacles.append((
                x0, bottom_row_start + spot_depth - clearance - car.length,
                x1, bottom_row_start + spot_depth - clearance,
            ))

    metrics = {"spot_width": spot_width, "spot_depth": spot_depth, "aisle_width": aisle}
    goal_x = spot_x(target) + spot_width / 2.0
    if reverse:
        # Nose points out at the aisle, so the car has to back in.
        goal = (goal_x, wall + clearance + car.rear_overhang, math.pi / 2.0)
    else:
        goal = (goal_x, wall + clearance + car.front_overhang, -math.pi / 2.0)
    aisle_y = top_row_end + aisle / 2.0
    return width, height, obstacles, goal, target, aisle_y, metrics


def _parallel_layout(
    rng: np.random.Generator, cfg: dict[str, Any], car: CarModel, tight: bool
) -> tuple[float, float, list[Rectangle], Pose, float, dict[str, float]]:
    settings = _settings(cfg, "parallel", tight)
    wall = float(cfg["world"]["wall_thickness"])
    lane_depth = float(settings["lane_depth"])
    spacing = float(settings["car_spacing"])
    aisle = float(settings["aisle_width"])
    gap = _sample_dimension(settings["gap"], rng)
    low, high = settings["bay_count"]
    count = int(rng.integers(int(low), int(high) + 1))
    target = int(rng.integers(0, count - 1))  # the gap goes after this car

    height = 2.0 * wall + 2.0 * lane_depth + aisle
    lane_centre = wall + lane_depth / 2.0
    far_lane_centre = height - wall - lane_depth / 2.0

    cursor = wall + spacing
    near: list[Rectangle] = []
    gap_start = None
    for index in range(count):
        if index == target + 1:
            gap_start = cursor
            cursor += gap
        near.append((cursor, lane_centre - car.width / 2.0,
                     cursor + car.length, lane_centre + car.width / 2.0))
        cursor += car.length + spacing
    if gap_start is None:  # the gap would land past the last car
        gap_start = cursor
        cursor += gap
    width = cursor + wall

    obstacles = _walls(width, height, wall)
    obstacles.extend(near)
    # A second lane of parked cars on the far side of the aisle.
    far_cursor = wall + spacing
    while far_cursor + car.length + wall < width:
        if rng.random() < 0.8:
            obstacles.append((far_cursor, far_lane_centre - car.width / 2.0,
                              far_cursor + car.length, far_lane_centre + car.width / 2.0))
        far_cursor += car.length + spacing

    goal_x = gap_start + (gap - car.length) / 2.0 + car.rear_overhang
    goal = (goal_x, lane_centre, 0.0)
    aisle_y = wall + lane_depth + aisle / 2.0
    metrics = {"gap": gap, "lane_depth": lane_depth, "aisle_width": aisle}
    return width, height, obstacles, goal, aisle_y, metrics


# ------------------------------------------------------------------ generator


def generate_parking_scenario(
    name: str,
    seed: int,
    config: dict[str, Any] | None = None,
    hybrid_config: dict[str, Any] | None = None,
) -> ParkingScenario:
    """Generate the parking scenario of type ``name`` for ``seed``."""
    hybrid_cfg = hybrid_config if hybrid_config is not None else load_config("hybrid")
    car = CarModel.from_config(hybrid_cfg)
    base_type, cfg = resolve_parking_type(
        name, car, config if config is not None else load_config("parking"), hybrid_cfg
    )
    rng = np.random.default_rng(seed)

    tight = base_type == "tight"
    offset_low, offset_high = cfg["start"]["offset"]
    attempts = int(cfg["start"]["max_attempts"])
    last_failure = "no attempt made"

    for _ in range(attempts):
        layout = (
            BASE_LAYOUTS[int(rng.integers(0, len(BASE_LAYOUTS)))] if tight else base_type
        )
        if layout.startswith("perpendicular"):
            reverse = layout.endswith("reverse")
            width, height, obstacles, goal, _, aisle_y, metrics = _perpendicular_layout(
                rng, cfg, car, reverse, tight
            )
            metrics["min_spot_width"] = minimum_spot_width(car, hybrid_cfg)
            metrics["clearance_above_minimum"] = metrics["spot_width"] - metrics["min_spot_width"]
        else:
            reverse = True  # a parallel bay can only be entered by reversing in
            width, height, obstacles, goal, aisle_y, metrics = _parallel_layout(
                rng, cfg, car, tight
            )
            metrics["min_gap"] = minimum_parallel_gap(car, hybrid_cfg)
            metrics["clearance_above_minimum"] = metrics["gap"] - metrics["min_gap"]

        offset = float(rng.uniform(float(offset_low), float(offset_high)))
        direction = 1.0 if rng.random() < 0.5 else -1.0
        start_x = goal[0] - direction * offset
        limit = 1.0 + car.length
        if not (limit < start_x < width - limit):
            start_x = float(np.clip(goal[0] + direction * offset, limit, width - limit))
        start = (start_x, aisle_y, 0.0 if direction > 0 else math.pi)

        scenario = ParkingScenario(
            name=name, seed=seed, layout=layout, width_m=width, height_m=height,
            obstacles=obstacles, start=start, goal=goal, requires_reverse=reverse,
            metrics=metrics, config=cfg, hybrid_config=hybrid_cfg,
        )
        checker = scenario.checker(car)
        if not checker.is_free(start):
            last_failure = "start pose is in collision"
            continue
        if not checker.is_free(goal):
            last_failure = "goal pose is in collision"
            continue
        field_to_goal = scenario.goal_distance_field()
        sx, sy = scenario.to_cell(start[0], start[1])
        if not math.isfinite(float(field_to_goal[sy, sx])):
            last_failure = "goal is not reachable from the start"
            continue
        return scenario

    raise ParkingGenerationError(
        f"could not generate parking scenario {name!r} for seed {seed}: {last_failure}"
    )
