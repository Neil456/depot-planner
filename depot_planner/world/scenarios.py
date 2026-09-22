"""Seeded scenario generators: map + ego start/goal + agent timelines + time limit.

Five scenario types are produced, all reproducible from a single integer seed:

``empty``                no moving agents
``crossing``             one agent crosses the ego's natural route
``head_on``              one agent drives down the same aisle toward the ego
``blocked_then_clears``  one agent sits in the aisle, then drives away
``congested``            6-10 agents moving through the depot

Every generator verifies that the problem is solvable in principle before
returning: the goal must still be reachable once the agents have come to rest.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Sequence

import numpy as np

from depot_planner.config import deep_merge, load_config
from depot_planner.grid_astar.search import astar
from depot_planner.world.agents import (
    Agent,
    Footprint,
    agent_hits_obstacle,
    agents_overlap,
    footprint_mask,
    occupancy_at,
    timed_path,
)
from depot_planner.world.depot_maps import connected_component, generate_depot
from depot_planner.world.grid import CellType, Grid

Cell = tuple[int, int]

SCENARIO_TYPES: tuple[str, ...] = (
    "empty",
    "crossing",
    "head_on",
    "blocked_then_clears",
    "congested",
)

#: Harder variants, defined in ``configs/scenarios.yaml`` under ``hard:``.
HARD_SCENARIO_TYPES: tuple[str, ...] = (
    "congested_hard",
    "head_on_narrow",
)

ALL_SCENARIO_TYPES: tuple[str, ...] = SCENARIO_TYPES + HARD_SCENARIO_TYPES

#: The ego may not enter a cell within this many cells of an agent body.
AGENT_INFLATION = 1


class ScenarioGenerationError(RuntimeError):
    """Raised when a scenario could not be generated within the attempt budget."""


@dataclass
class Scenario:
    """A complete planning problem over time."""

    name: str
    seed: int
    grid: Grid
    start: Cell
    goal: Cell
    agents: list[Agent]
    time_limit: int
    dt: float
    natural_route: list[Cell] = field(default_factory=list)
    config: dict[str, Any] = field(default_factory=dict)
    base: str = ""            # the normal-tier type a hard variant is built from

    def occupied_at(self, t: int, inflate: int = 0) -> set[Cell]:
        """Cells covered by agent bodies at step ``t``."""
        return occupancy_at(self.agents, t, inflate)

    def is_free(self, cell: Cell, t: int, inflate: int = AGENT_INFLATION) -> bool:
        """True if the ego may occupy ``cell`` at step ``t``."""
        x, y = cell
        if not self.grid.is_drivable(x, y):
            return False
        return cell not in self.occupied_at(t, inflate)

    def resting_obstacle_grid(self, inflate: int = AGENT_INFLATION) -> Grid:
        """The map with the agents' final resting footprints marked as obstacles."""
        grid = self.grid.copy()
        for cell in self.occupied_at(self.time_limit, inflate):
            x, y = cell
            if grid.in_bounds(x, y):
                grid.cells[y, x] = CellType.OBSTACLE
        return Grid(grid.cells, grid.resolution, grid.config)

    def summary(self) -> dict[str, Any]:
        """Compact description for logs and the report."""
        return {
            "scenario": self.name,
            "base": self.base or self.name,
            "seed": self.seed,
            "start": self.start,
            "goal": self.goal,
            "n_agents": len(self.agents),
            "time_limit": self.time_limit,
            "dt": self.dt,
        }


def resolve_scenario_type(
    name: str,
    config: dict[str, Any] | None = None,
    depot_config: dict[str, Any] | None = None,
) -> tuple[str, dict[str, Any], dict[str, Any]]:
    """Return ``(base_type, scenario_config, depot_config)`` for ``name``.

    A normal type resolves to itself with the configs untouched. A hard type
    resolves to its base type with the tier overrides merged in, so the normal
    tier can never be changed by editing a hard tier.
    """
    scenario_cfg = config if config is not None else load_config("scenarios")
    depot_cfg = depot_config if depot_config is not None else load_config("depot")
    if name in SCENARIO_TYPES:
        return name, scenario_cfg, depot_cfg
    hard = scenario_cfg.get("hard", {})
    if name not in hard:
        raise ValueError(
            f"unknown scenario type {name!r}; expected one of {ALL_SCENARIO_TYPES}"
        )
    spec = hard[name]
    base = str(spec["base"])
    if base not in SCENARIO_TYPES:
        raise ValueError(f"hard type {name!r} names an unknown base type {base!r}")
    return (
        base,
        deep_merge(scenario_cfg, spec.get("scenarios", {})),
        deep_merge(depot_cfg, spec.get("depot", {})),
    )


# --------------------------------------------------------------------- helpers


def _uniform_cost(mask: np.ndarray) -> np.ndarray:
    return np.where(mask, 1.0, np.inf)


def _route_between(mask: np.ndarray, start: Cell, goal: Cell) -> list[Cell] | None:
    result = astar(_uniform_cost(mask), start, goal, drivable=mask, min_cell_cost=1.0,
                   collect_expanded=False)
    return result.path if result.success else None


def _sample_cell(rng: np.random.Generator, xs: np.ndarray, ys: np.ndarray) -> Cell:
    index = int(rng.integers(0, len(xs)))
    return int(xs[index]), int(ys[index])


def _straight_runs(route: Sequence[Cell], mask: np.ndarray) -> list[tuple[int, int]]:
    """Maximal index ranges of ``route`` that keep one direction and stay on ``mask``."""
    runs: list[tuple[int, int]] = []
    if len(route) < 2:
        return runs
    begin = 0
    previous = (route[1][0] - route[0][0], route[1][1] - route[0][1])
    for index in range(1, len(route)):
        step = (route[index][0] - route[index - 1][0], route[index][1] - route[index - 1][1])
        valid = mask[route[index][1], route[index][0]] and mask[route[index - 1][1], route[index - 1][0]]
        if step != previous or not valid:
            if index - 1 > begin:
                runs.append((begin, index - 1))
            begin = index - 1 if valid else index
            previous = step
    if len(route) - 1 > begin:
        runs.append((begin, len(route) - 1))
    return [(a, b) for a, b in runs if mask[route[a][1], route[a][0]] and mask[route[b][1], route[b][0]]]


def _extend(mask: np.ndarray, cell: Cell, step: Cell, limit: int) -> int:
    """How many steps of ``step`` from ``cell`` stay inside ``mask`` (at most ``limit``)."""
    height, width = mask.shape
    count = 0
    x, y = cell
    while count < limit:
        x, y = x + step[0], y + step[1]
        if not (0 <= x < width and 0 <= y < height) or not mask[y, x]:
            break
        count += 1
    return count


def _fraction_window(length: int, fractions: Sequence[float]) -> range:
    lo = max(1, int(fractions[0] * length))
    hi = max(lo + 1, min(length - 2, int(fractions[1] * length)))
    return range(lo, hi)


# ------------------------------------------------------------ agent placement


def _place_crossing_agent(
    route: Sequence[Cell], mask: np.ndarray, rng: np.random.Generator,
    cfg: dict[str, Any], footprint: Footprint, horizon: int,
) -> Agent | None:
    settings = cfg["crossing"]
    arm = int(settings["min_arm_cells"])
    candidates = list(_fraction_window(len(route), settings["route_fraction"]))
    rng.shuffle(candidates)
    for m in candidates:
        cell = route[m]
        if not mask[cell[1], cell[0]]:
            continue
        dx = route[m + 1][0] - route[m][0]
        dy = route[m + 1][1] - route[m][1]
        perp = (-dy, dx)
        if perp == (0, 0):
            continue
        back = _extend(mask, cell, (-perp[0], -perp[1]), arm * 2)
        forward = _extend(mask, cell, perp, arm * 2)
        if back < arm or forward < arm:
            continue
        agent_route = [
            (cell[0] + i * perp[0], cell[1] + i * perp[1]) for i in range(-back, forward + 1)
        ]
        delay = max(0, m - back)
        timeline = timed_path(agent_route, horizon, rng, start_delay=delay, pause_probability=0.0)
        return Agent(0, timeline, footprint, kind="crossing")
    return None


def _place_head_on_agent(
    route: Sequence[Cell], mask: np.ndarray, rng: np.random.Generator,
    cfg: dict[str, Any], footprint: Footprint, horizon: int,
) -> Agent | None:
    settings = cfg["head_on"]
    minimum = int(settings["min_segment_cells"])
    window = _fraction_window(len(route), settings["route_fraction"])
    runs = [
        (max(a, window.start), min(b, window.stop))
        for a, b in _straight_runs(route, mask)
    ]
    runs = [(a, b) for a, b in runs if b - a >= minimum]
    if not runs:
        return None
    a, b = runs[int(rng.integers(0, len(runs)))]
    agent_route = [route[i] for i in range(b, a - 1, -1)]
    # Start delay chosen so the two vehicles meet in the middle of the shared stretch.
    timeline = timed_path(agent_route, horizon, rng, start_delay=a, pause_probability=0.0)
    return Agent(0, timeline, footprint, kind="head_on")


def _place_blocker_agent(
    route: Sequence[Cell], mask: np.ndarray, rng: np.random.Generator,
    cfg: dict[str, Any], footprint: Footprint, horizon: int,
    anchors: tuple[np.ndarray, np.ndarray],
) -> Agent | None:
    settings = cfg["blocked_then_clears"]
    low, high = settings["release_after_arrival"]
    candidates = list(_fraction_window(len(route), settings["route_fraction"]))
    rng.shuffle(candidates)
    xs, ys = anchors
    for m in candidates:
        cell = route[m]
        if not mask[cell[1], cell[0]]:
            continue
        # Measured from the step the ego would naturally arrive, so the blocker is
        # always in the way when the ego gets there and always clears afterwards.
        release = m + int(rng.integers(int(low), int(high) + 1))
        for _ in range(int(cfg["agent"]["max_route_attempts"])):
            escape_goal = _sample_cell(rng, xs, ys)
            if abs(escape_goal[0] - cell[0]) + abs(escape_goal[1] - cell[1]) < int(
                cfg["agent"]["min_route_cells"]
            ):
                continue
            escape = _route_between(mask, cell, escape_goal)
            if escape is None or len(escape) < int(cfg["agent"]["min_route_cells"]):
                continue
            timeline = timed_path(
                escape, horizon, rng, start_delay=release,
                pause_probability=float(cfg["agent"]["pause_probability"]),
            )
            return Agent(0, timeline, footprint, kind="blocker")
    return None


def _place_traffic_agents(
    mask: np.ndarray, rng: np.random.Generator, cfg: dict[str, Any],
    footprint: Footprint, horizon: int, count: int,
    anchors: tuple[np.ndarray, np.ndarray],
) -> list[Agent] | None:
    xs, ys = anchors
    attempts = int(cfg["agent"]["max_route_attempts"])
    minimum = int(cfg["agent"]["min_route_cells"])
    pause = float(cfg["agent"]["pause_probability"])
    placed: list[Agent] = []
    for index in range(count):
        agent: Agent | None = None
        for _ in range(attempts):
            start = _sample_cell(rng, xs, ys)
            goal = _sample_cell(rng, xs, ys)
            if abs(start[0] - goal[0]) + abs(start[1] - goal[1]) < minimum:
                continue
            route = _route_between(mask, start, goal)
            if route is None or len(route) < minimum:
                continue
            delay = int(rng.integers(0, 25))
            candidate = Agent(
                index,
                timed_path(route, horizon, rng, start_delay=delay, pause_probability=pause),
                footprint,
                kind="traffic",
            )
            if any(agents_overlap(candidate, other, horizon) for other in placed):
                continue
            agent = candidate
            break
        if agent is None:
            return None
        placed.append(agent)
    return placed


# ----------------------------------------------------------------- validation


def _agents_are_consistent(agents: Sequence[Agent], grid: Grid, horizon: int) -> bool:
    for agent in agents:
        if agent_hits_obstacle(agent, grid):
            return False
    for i, first in enumerate(agents):
        for second in agents[i + 1 :]:
            if agents_overlap(first, second, horizon):
                return False
    return True


def _is_solvable_in_principle(scenario: Scenario) -> bool:
    """The ego must be clear at t=0 and able to reach the goal once agents rest."""
    if scenario.start in scenario.occupied_at(0, AGENT_INFLATION):
        return False
    if scenario.goal in scenario.occupied_at(scenario.time_limit, AGENT_INFLATION):
        return False
    resting = scenario.resting_obstacle_grid()
    result = astar(
        resting.cost_map(), scenario.start, scenario.goal,
        drivable=resting.drivable, min_cell_cost=resting.min_cell_cost(),
        collect_expanded=False,
    )
    return result.success


# ------------------------------------------------------------------ generator


def generate_scenario(
    name: str,
    seed: int,
    config: dict[str, Any] | None = None,
    map_seed: int | None = None,
    depot_config: dict[str, Any] | None = None,
) -> Scenario:
    """Generate the scenario of type ``name`` for ``seed``.

    ``name`` may be a normal type or one of :data:`HARD_SCENARIO_TYPES`. The
    same ``(name, seed)`` always yields an identical scenario.
    """
    base, cfg, depot_cfg = resolve_scenario_type(name, config, depot_config)
    settings = cfg["scenario"]
    footprint = Footprint(*cfg["agent"]["footprint"])
    rng = np.random.default_rng(seed)

    grid = generate_depot(seed=seed if map_seed is None else map_seed, config=depot_cfg)
    cost_map = grid.cost_map()
    min_cost = grid.min_cell_cost()
    reachable = connected_component(grid)

    ego_mask = reachable & (grid.cells == CellType.AISLE)
    ego_ys, ego_xs = np.nonzero(ego_mask)
    agent_mask = footprint_mask(grid, footprint, aisles_only=True) & reachable
    agent_ys, agent_xs = np.nonzero(agent_mask)
    if len(ego_xs) == 0 or len(agent_xs) == 0:
        raise ScenarioGenerationError(f"depot for seed {seed} has no usable aisle space")
    anchors = (agent_xs, agent_ys)

    last_failure = "no attempt made"
    for _ in range(int(settings["max_generation_attempts"])):
        start = _sample_cell(rng, ego_xs, ego_ys)
        goal = _sample_cell(rng, ego_xs, ego_ys)
        if np.hypot(start[0] - goal[0], start[1] - goal[1]) < float(settings["min_ego_separation"]):
            last_failure = "ego endpoints too close"
            continue
        reference = astar(cost_map, start, goal, drivable=grid.drivable,
                          min_cell_cost=min_cost, collect_expanded=False)
        if not reference.success or len(reference.path) < 12:
            last_failure = "no usable reference route"
            continue
        route = reference.path
        horizon = int(np.clip(
            settings["time_limit_factor"] * len(route),
            settings["min_time_limit"], settings["max_time_limit"],
        ))

        if base == "empty":
            agents: list[Agent] | None = []
        elif base == "crossing":
            agent = _place_crossing_agent(route, agent_mask, rng, cfg, footprint, horizon)
            agents = None if agent is None else [agent]
        elif base == "head_on":
            agent = _place_head_on_agent(route, agent_mask, rng, cfg, footprint, horizon)
            agents = None if agent is None else [agent]
        elif base == "blocked_then_clears":
            agent = _place_blocker_agent(route, agent_mask, rng, cfg, footprint, horizon, anchors)
            agents = None if agent is None else [agent]
        else:  # congested
            low, high = cfg["congested"]["agent_count"]
            count = int(rng.integers(int(low), int(high) + 1))
            agents = _place_traffic_agents(agent_mask, rng, cfg, footprint, horizon, count, anchors)

        if agents is None:
            last_failure = f"could not place agents for {name}"
            continue
        if not _agents_are_consistent(agents, grid, horizon):
            last_failure = "agents overlap walls or each other"
            continue

        scenario = Scenario(
            name=name, seed=seed, grid=grid, start=start, goal=goal, agents=agents,
            time_limit=horizon, dt=float(cfg["dt"]), natural_route=list(route), config=cfg,
            base=base,
        )
        if not _is_solvable_in_principle(scenario):
            last_failure = "goal not reachable once the agents come to rest"
            continue
        return scenario

    raise ScenarioGenerationError(
        f"could not generate scenario {name!r} for seed {seed}: {last_failure}"
    )
