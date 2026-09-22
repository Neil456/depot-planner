"""Hybrid A*: car-shaped planning over continuous ``(x, y, heading)`` states.

The search expands constant-steering arcs of a fixed length, forward and
reverse, collision checking each arc in sub-steps. Duplicate detection
discretises the continuous state to a position cell and a heading bin. The
heuristic is the larger of the straight-line distance and an obstacle-aware
grid distance from the goal. A Reeds-Shepp analytic expansion is attempted
periodically so the exact goal pose is reachable.
"""

from __future__ import annotations

import heapq
import math
import time
from dataclasses import dataclass, field
from typing import Any, Sequence

import numpy as np

from depot_planner import core
from depot_planner.config import load_config
from depot_planner.hybrid import reeds_shepp as rs
from depot_planner.hybrid.car import CarModel, Pose, angle_difference, wrap_angle
from depot_planner.hybrid.collision import CarCollisionChecker

Key = tuple[int, int, int]


@dataclass
class HybridResult:
    """Outcome of one hybrid A* search."""

    success: bool
    poses: list[Pose] = field(default_factory=list)
    directions: list[int] = field(default_factory=list)
    cost: float = math.inf
    path_length_m: float = 0.0
    direction_switches: int = 0
    nodes_expanded: int = 0
    runtime_ms: float = 0.0
    collision_checks: int = 0
    explored: list[Pose] = field(default_factory=list)
    used_analytic: bool = False
    reason: str = ""

    @property
    def reverse_length_m(self) -> float:
        """Total distance driven in reverse."""
        total = 0.0
        for index in range(len(self.poses) - 1):
            if self.directions[index] < 0:
                a, b = self.poses[index], self.poses[index + 1]
                total += math.hypot(b[0] - a[0], b[1] - a[1])
        return total

    @property
    def has_reverse(self) -> bool:
        """True if the plan reverses at any point."""
        return any(direction < 0 for direction in self.directions)


@dataclass
class _Node:
    pose: Pose
    g: float
    direction: int
    steer: float
    parent: Key | None
    segment: list[Pose]        # dense poses from the parent to this node


class PrimitiveSet:
    """The motion primitives, precomputed once in the car's own frame.

    The bicycle model is invariant to the starting pose, so each arc can be
    integrated once from the origin and then rotated and translated onto any
    node. That turns an expansion into a handful of numpy operations and a
    single distance-field lookup.
    """

    def __init__(
        self,
        car: CarModel,
        arc_length: float,
        steers: Sequence[float],
        directions: Sequence[int],
        substeps: int,
    ) -> None:
        arcs: list[list[Pose]] = []
        self.meta: list[tuple[int, float]] = []
        for direction in directions:
            for steer in steers:
                arcs.append(car.sample_arc((0.0, 0.0, 0.0), steer, direction * arc_length, substeps))
                self.meta.append((int(direction), float(steer)))
        self.local = np.asarray(arcs, dtype=float)   # (primitives, substeps, 3)

    def __len__(self) -> int:
        return len(self.meta)

    def expand(self, pose: Pose) -> np.ndarray:
        """``(primitives, substeps, 3)`` array of poses reached from ``pose``."""
        x, y, theta = pose
        cos_t, sin_t = math.cos(theta), math.sin(theta)
        local_x = self.local[..., 0]
        local_y = self.local[..., 1]
        return np.stack(
            [
                x + local_x * cos_t - local_y * sin_t,
                y + local_x * sin_t + local_y * cos_t,
                theta + self.local[..., 2],
            ],
            axis=-1,
        )


def heading_bin(theta: float, bins: int) -> int:
    """Index of the heading bin containing ``theta``."""
    width = 2.0 * math.pi / bins
    return int((theta % (2.0 * math.pi)) / width) % bins


def state_key(pose: Pose, xy_resolution: float, bins: int) -> Key:
    """Discretise a continuous pose for duplicate detection."""
    return (
        int(math.floor(pose[0] / xy_resolution)),
        int(math.floor(pose[1] / xy_resolution)),
        heading_bin(pose[2], bins),
    )


def at_goal(pose: Pose, goal: Pose, position_tolerance: float, heading_tolerance: float) -> bool:
    """True if ``pose`` is within both tolerances of ``goal``, wrapping the heading."""
    if math.hypot(pose[0] - goal[0], pose[1] - goal[1]) > position_tolerance:
        return False
    return abs(angle_difference(pose[2], goal[2])) <= heading_tolerance


class _Heuristic:
    """max(Euclidean, obstacle-aware grid distance) to the goal."""

    def __init__(
        self,
        goal: Pose,
        field: np.ndarray | None,
        resolution: float,
        correct_octile: bool,
    ) -> None:
        self.goal = goal
        self.field = field
        self.resolution = resolution
        # An 8-connected grid overestimates a straight line by up to 1/cos(pi/8);
        # scaling the field back keeps it a lower bound in free space.
        self.scale = math.cos(math.pi / 8.0) if correct_octile else 1.0

    def __call__(self, pose: Pose) -> float:
        euclid = math.hypot(pose[0] - self.goal[0], pose[1] - self.goal[1])
        if self.field is None:
            return euclid
        rows, cols = self.field.shape
        j = int(pose[0] // self.resolution)
        i = int(pose[1] // self.resolution)
        if not (0 <= i < rows and 0 <= j < cols):
            return euclid
        grid = float(self.field[i, j]) * self.scale
        if not math.isfinite(grid):
            return math.inf
        return max(euclid, grid)


def plan(
    start: Pose,
    goal: Pose,
    checker: CarCollisionChecker,
    *,
    heuristic_field: np.ndarray | None = None,
    heuristic_resolution: float = 1.0,
    config: dict[str, Any] | None = None,
    car: CarModel | None = None,
    collect_explored: bool = True,
    backend: str | None = None,
) -> HybridResult:
    """Plan a car-shaped path from ``start`` to ``goal``.

    ``backend`` selects the implementation: ``"cpp"`` for the compiled core,
    ``"python"`` for this module (the reference), or ``"auto"``/``None`` for the
    core whenever it is built and the call does not need the explored poses.
    """
    cfg = config if config is not None else load_config("hybrid")
    model = car if car is not None else CarModel.from_config(cfg)

    if core.resolve(backend) == "cpp" and not collect_explored:
        return _plan_cpp(start, goal, checker, heuristic_field, heuristic_resolution, cfg, model)

    discretisation = cfg["discretisation"]
    xy_resolution = float(discretisation["xy_resolution"])
    bins = int(discretisation["heading_bins"])
    primitives = cfg["primitives"]
    arc_length = float(primitives["arc_length"])
    substeps = int(primitives["substeps"])
    steers = model.steer_angles(primitives["steer_fractions"])
    directions = [int(d) for d in primitives["directions"]]
    fan = PrimitiveSet(model, arc_length, steers, directions, substeps)
    costs = cfg["costs"]
    reverse_multiplier = float(costs["reverse_multiplier"])
    direction_change = float(costs["direction_change"])
    steer_penalty = float(costs["steer_penalty"])
    steer_change_penalty = float(costs["steer_change_penalty"])
    goal_cfg = cfg["goal"]
    position_tolerance = float(goal_cfg["position_tolerance"])
    heading_tolerance = math.radians(float(goal_cfg["heading_tolerance_deg"]))
    limits = cfg["limits"]
    max_expansions = int(limits["max_expansions"])
    time_limit_s = float(limits["time_limit_s"])
    analytic = cfg["analytic"]
    analytic_enabled = bool(analytic["enabled"])
    analytic_every = max(1, int(analytic["every"]))
    analytic_max_distance = float(analytic["max_distance"])
    analytic_step = float(analytic["interpolation_step"])

    began = time.perf_counter()

    def elapsed_ms() -> float:
        return (time.perf_counter() - began) * 1000.0

    def fail(reason: str, expansions: int, explored: list[Pose]) -> HybridResult:
        return HybridResult(False, [], [], math.inf, 0.0, 0, expansions, elapsed_ms(),
                            checker.checks, explored, False, reason)

    explored: list[Pose] = []
    if not checker.is_free(start):
        return fail("start pose is in collision", 0, explored)
    if not checker.is_free(goal):
        return fail("goal pose is in collision", 0, explored)

    heuristic = _Heuristic(goal, heuristic_field, heuristic_resolution,
                           bool(cfg["heuristic"]["octile_correction"]))
    start_h = heuristic(start)
    if not math.isfinite(start_h):
        return fail("goal is unreachable from the start", 0, explored)

    start_key = state_key(start, xy_resolution, bins)
    nodes: dict[Key, _Node] = {start_key: _Node(start, 0.0, 1, 0.0, None, [])}
    g_score: dict[Key, float] = {start_key: 0.0}
    closed: set[Key] = set()
    heap: list[tuple[float, int, Key]] = [(start_h, 0, start_key)]
    counter = 0
    expansions = 0

    while heap:
        _, _, key = heapq.heappop(heap)
        if key in closed:
            continue
        closed.add(key)
        node = nodes[key]
        expansions += 1
        if collect_explored:
            explored.append(node.pose)

        if at_goal(node.pose, goal, position_tolerance, heading_tolerance):
            return _finish(nodes, key, start, [], [], node.g, expansions, elapsed_ms(),
                           checker, explored, False, "goal reached")

        if expansions >= max_expansions:
            return fail("expansion limit reached", expansions, explored)
        if elapsed_ms() > time_limit_s * 1000.0:
            return fail("time limit reached", expansions, explored)

        # Reeds-Shepp shot at the goal.
        if (
            analytic_enabled
            and expansions % analytic_every == 0
            and math.hypot(node.pose[0] - goal[0], node.pose[1] - goal[1]) <= analytic_max_distance
        ):
            tail = _analytic_shot(
                node, goal, model, checker, analytic_step, reverse_multiplier, direction_change
            )
            if tail is not None:
                tail_poses, tail_directions, tail_cost = tail
                return _finish(nodes, key, start, tail_poses, tail_directions,
                               node.g + tail_cost, expansions, elapsed_ms(), checker,
                               explored, True, "goal reached via Reeds-Shepp expansion")

        arcs = fan.expand(node.pose)
        arc_free = checker.poses_are_free(arcs).all(axis=1)
        for index, (direction, steer) in enumerate(fan.meta):
            if not arc_free[index]:
                continue
            segment = [(float(p[0]), float(p[1]), wrap_angle(float(p[2]))) for p in arcs[index]]
            successor = segment[-1]
            successor_key = state_key(successor, xy_resolution, bins)
            if successor_key in closed:
                continue
            step_cost = arc_length * (reverse_multiplier if direction < 0 else 1.0)
            if direction != node.direction:
                step_cost += direction_change
            step_cost += steer_penalty * abs(steer)
            step_cost += steer_change_penalty * abs(steer - node.steer)
            tentative = node.g + step_cost
            if tentative >= g_score.get(successor_key, math.inf) - 1e-12:
                continue
            h = heuristic(successor)
            if not math.isfinite(h):
                continue
            g_score[successor_key] = tentative
            nodes[successor_key] = _Node(successor, tentative, direction, steer, key, segment)
            counter += 1
            heapq.heappush(heap, (tentative + h, counter, successor_key))

    return fail("no path found", expansions, explored)


def cpp_options(config: dict[str, Any]) -> dict[str, Any]:
    """Flatten ``configs/hybrid.yaml`` into the keys the C++ core reads."""
    discretisation = config["discretisation"]
    primitives = config["primitives"]
    costs = config["costs"]
    goal_cfg = config["goal"]
    limits = config["limits"]
    analytic = config["analytic"]
    return {
        "xy_resolution": float(discretisation["xy_resolution"]),
        "heading_bins": int(discretisation["heading_bins"]),
        "arc_length": float(primitives["arc_length"]),
        "steer_fractions": [float(f) for f in primitives["steer_fractions"]],
        "substeps": int(primitives["substeps"]),
        "directions": [int(d) for d in primitives["directions"]],
        "reverse_multiplier": float(costs["reverse_multiplier"]),
        "direction_change": float(costs["direction_change"]),
        "steer_penalty": float(costs["steer_penalty"]),
        "steer_change_penalty": float(costs["steer_change_penalty"]),
        "position_tolerance": float(goal_cfg["position_tolerance"]),
        "heading_tolerance": math.radians(float(goal_cfg["heading_tolerance_deg"])),
        "max_expansions": int(limits["max_expansions"]),
        "time_limit_s": float(limits["time_limit_s"]),
        "analytic_enabled": bool(analytic["enabled"]),
        "analytic_every": int(analytic["every"]),
        "analytic_max_distance": float(analytic["max_distance"]),
        "analytic_step": float(analytic["interpolation_step"]),
        "octile_correction": bool(config["heuristic"]["octile_correction"]),
        "safety_margin": float(config["collision"]["safety_margin"]),
    }


def cpp_car(car: CarModel) -> dict[str, Any]:
    """The car geometry as the keys the C++ core reads."""
    return {
        "wheelbase": car.wheelbase,
        "length": car.length,
        "width": car.width,
        "max_steer": car.max_steer,
        "rear_overhang": car.rear_overhang,
        "n_discs": car.n_discs,
    }


def _plan_cpp(
    start: Pose,
    goal: Pose,
    checker: CarCollisionChecker,
    heuristic_field: np.ndarray | None,
    heuristic_resolution: float,
    config: dict[str, Any],
    car: CarModel,
) -> HybridResult:
    """Run the compiled core and wrap its answer as a :class:`HybridResult`.

    The distance field and the heuristic field are built in Python (they are
    scenario geometry, not planning) and borrowed by the core for the call.
    """
    field = np.ascontiguousarray(checker.field.distance)
    heuristic = (
        None if heuristic_field is None
        else np.ascontiguousarray(np.asarray(heuristic_field, dtype=float))
    )
    (success, poses, directions, cost, length, switches, expansions, runtime_ms,
     checks, used_analytic, reason) = core.require().hybrid_plan(
        (float(start[0]), float(start[1]), float(start[2])),
        (float(goal[0]), float(goal[1]), float(goal[2])),
        cpp_car(car),
        field,
        float(checker.field.resolution),
        heuristic,
        float(heuristic_resolution),
        cpp_options(config),
    )
    # Keep the checker's own tally in step, so a reused checker still reports
    # the total number of poses it has been asked about.
    checker.checks += int(checks)
    if not success:
        return HybridResult(False, [], [], math.inf, 0.0, 0, int(expansions), float(runtime_ms),
                            checker.checks, [], False, str(reason))
    return HybridResult(
        success=True,
        poses=[(float(x), float(y), float(theta)) for x, y, theta in poses],
        directions=[int(d) for d in directions],
        cost=float(cost),
        path_length_m=float(length),
        direction_switches=int(switches),
        nodes_expanded=int(expansions),
        runtime_ms=float(runtime_ms),
        collision_checks=checker.checks,
        explored=[],
        used_analytic=bool(used_analytic),
        reason=str(reason),
    )


def _analytic_shot(
    node: _Node,
    goal: Pose,
    car: CarModel,
    checker: CarCollisionChecker,
    step: float,
    reverse_multiplier: float,
    direction_change: float,
) -> tuple[list[Pose], list[int], float] | None:
    """Try to connect ``node`` to ``goal`` with a collision-free Reeds-Shepp curve."""
    path = rs.shortest_reeds_shepp(node.pose, goal, car.max_curvature)
    if path is None:
        return None
    poses, directions = rs.interpolate(node.pose, path, car, step)
    # The first pose is the node itself and was already checked.
    if not checker.path_is_free(poses[1:]):
        return None

    cost = 0.0
    previous = node.direction
    radius = car.min_turning_radius
    for segment in path.segments:
        distance = abs(segment.length) * radius
        gear = 1 if segment.length >= 0.0 else -1
        cost += distance * (reverse_multiplier if gear < 0 else 1.0)
        if gear != previous:
            cost += direction_change
        previous = gear
    return poses[1:], directions, cost


def _finish(
    nodes: dict[Key, _Node],
    key: Key,
    start: Pose,
    tail_poses: Sequence[Pose],
    tail_directions: Sequence[int],
    cost: float,
    expansions: int,
    runtime_ms: float,
    checker: CarCollisionChecker,
    explored: list[Pose],
    used_analytic: bool,
    reason: str,
) -> HybridResult:
    """Walk the parent chain back to the start and assemble the dense path."""
    segments: list[tuple[list[Pose], int]] = []
    current: Key | None = key
    while current is not None:
        node = nodes[current]
        if node.parent is not None:
            segments.append((node.segment, node.direction))
        current = node.parent
    segments.reverse()

    poses: list[Pose] = [start]
    directions: list[int] = []
    for segment, direction in segments:
        poses.extend(segment)
        directions.extend([direction] * len(segment))
    poses.extend(tail_poses)
    directions.extend(tail_directions)

    length = sum(
        math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(poses, poses[1:])
    )
    switches = sum(1 for a, b in zip(directions, directions[1:]) if a != b)
    return HybridResult(
        success=True, poses=poses, directions=directions, cost=cost, path_length_m=length,
        direction_switches=switches, nodes_expanded=expansions, runtime_ms=runtime_ms,
        collision_checks=checker.checks, explored=explored, used_analytic=used_analytic,
        reason=reason,
    )


def plan_for_scenario(
    scenario,
    config: dict[str, Any] | None = None,
    collect_explored: bool = True,
    backend: str | None = None,
) -> HybridResult:
    """Plan for a :class:`~depot_planner.world.parking.ParkingScenario`."""
    cfg = config if config is not None else scenario.hybrid_config or load_config("hybrid")
    car = CarModel.from_config(cfg)
    checker = scenario.checker(car)
    _, _, resolution = scenario.heuristic_grid()
    return plan(
        scenario.start, scenario.goal, checker,
        heuristic_field=scenario.goal_distance_field(),
        heuristic_resolution=resolution, config=cfg, car=car,
        collect_explored=collect_explored, backend=backend,
    )
