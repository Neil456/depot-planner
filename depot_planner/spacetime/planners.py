"""Planner wrappers used by the closed-loop runner.

Two planners share one interface:

``SpaceTimePlanner``  plans over ``(x, y, t)`` against the agents' known timelines.
``BaselineReplanPlanner``  plain grid A* that freezes the agents where they are
right now and replans every step.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Any

import numpy as np

from depot_planner.config import load_config
from depot_planner.grid_astar.search import astar
from depot_planner.spacetime import search as st
from depot_planner.spacetime.collision import AgentOccupancy
from depot_planner.world.grid import CellType, Grid

Cell = tuple[int, int]


@dataclass
class PlanResult:
    """A plan handed to the runner: where the ego should be, and when."""

    success: bool
    cells: list[Cell]
    times: list[int]
    cost: float = math.inf
    nodes_expanded: int = 0
    runtime_ms: float = 0.0
    wait_steps: int = 0
    reason: str = ""

    def __len__(self) -> int:
        return len(self.cells)


@dataclass
class _ScenarioCache:
    cost_map: np.ndarray
    drivable: np.ndarray
    min_cell_cost: float
    goal_field: np.ndarray | None = None


class Planner:
    """Common interface. ``replan_every`` lets a planner set its own cadence."""

    name: str = "planner"
    replan_every: int | None = None

    def __init__(self, config: dict[str, Any] | None = None) -> None:
        self.config = config if config is not None else load_config("spacetime")
        self._cached_for: Any = None
        self._cache: _ScenarioCache | None = None

    def prepare(self, scenario) -> _ScenarioCache:
        """Per-scenario derived data, kept across the replans of one episode.

        The cache is keyed on object identity and holds a reference to the
        scenario, so a recycled ``id()`` can never serve a stale cost map.
        """
        if self._cache is None or self._cached_for is not scenario:
            grid: Grid = scenario.grid
            self._cache = _ScenarioCache(grid.cost_map(), grid.drivable, grid.min_cell_cost())
            self._cached_for = scenario
        return self._cache

    def reset(self) -> None:
        self._cache = None
        self._cached_for = None

    def plan(self, scenario, cell: Cell, t: int) -> PlanResult:  # pragma: no cover - interface
        raise NotImplementedError


class SpaceTimePlanner(Planner):
    """Space-time A* over the agents' known timelines."""

    name = "spacetime_astar"

    def prepare(self, scenario) -> _ScenarioCache:
        cached = super().prepare(scenario)
        if cached.goal_field is None and str(
            self.config["spacetime"]["heuristic"]
        ).lower() == "grid_dijkstra":
            cached.goal_field = st.goal_cost_field(cached.cost_map, cached.drivable, scenario.goal)
        return cached

    def plan(self, scenario, cell: Cell, t: int) -> PlanResult:
        cached = self.prepare(scenario)
        # Planning past the episode's own time limit cannot help, and searching that
        # far is what makes a hopeless replan expensive.
        horizon = min(int(self.config["spacetime"]["max_time_horizon"]), int(scenario.time_limit))
        result = st.plan(
            cached.cost_map, cached.drivable, cell, scenario.goal, scenario.agents,
            start_time=t, min_cell_cost=cached.min_cell_cost, config=self.config,
            collect_expanded=False, heuristic_field=cached.goal_field, max_time=horizon,
        )
        if not result.success:
            return PlanResult(False, [cell], [t], math.inf, result.nodes_expanded,
                              result.runtime_ms, 0, result.reason)
        return PlanResult(True, result.cells, result.times, result.cost,
                          result.nodes_expanded, result.runtime_ms, result.wait_steps,
                          result.reason)


class BaselineReplanPlanner(Planner):
    """Grid A* that treats agents as static obstacles at their current positions.

    It has no idea the agents are going to move, so it replans every single step;
    that is the whole baseline.
    """

    name = "baseline_replan"
    replan_every = 1

    def plan(self, scenario, cell: Cell, t: int) -> PlanResult:
        cached = self.prepare(scenario)
        cfg = self.config["baseline"]
        began = time.perf_counter()

        occupancy = AgentOccupancy(scenario.agents, int(cfg["inflate"]))
        blocked = occupancy.margin(t)
        cost_map = cached.cost_map.copy()
        drivable = cached.drivable.copy()
        for bx, by in blocked:
            if 0 <= bx < drivable.shape[1] and 0 <= by < drivable.shape[0]:
                drivable[by, bx] = False
                cost_map[by, bx] = math.inf
        # The ego is where it is: never declare its own cell unusable.
        drivable[cell[1], cell[0]] = cached.drivable[cell[1], cell[0]]
        cost_map[cell[1], cell[0]] = cached.cost_map[cell[1], cell[0]]

        result = astar(cost_map, cell, scenario.goal, drivable=drivable,
                       min_cell_cost=cached.min_cell_cost, collect_expanded=False)
        runtime_ms = (time.perf_counter() - began) * 1000.0

        if result.success:
            times = list(range(t, t + len(result.path)))
            return PlanResult(True, list(result.path), times, result.cost,
                              result.nodes_expanded, runtime_ms, 0, result.reason)
        if bool(cfg["hold_on_failure"]):
            # Blocked right now: hold position for one step and try again next step.
            return PlanResult(True, [cell, cell], [t, t + 1], 0.0, result.nodes_expanded,
                              runtime_ms, 1, "blocked, holding position")
        return PlanResult(False, [cell], [t], math.inf, result.nodes_expanded, runtime_ms,
                          0, result.reason)


PLANNERS: dict[str, type[Planner]] = {
    SpaceTimePlanner.name: SpaceTimePlanner,
    BaselineReplanPlanner.name: BaselineReplanPlanner,
}


def make_planner(name: str, config: dict[str, Any] | None = None) -> Planner:
    if name not in PLANNERS:
        raise ValueError(f"unknown planner {name!r}; expected one of {sorted(PLANNERS)}")
    return PLANNERS[name](config)
