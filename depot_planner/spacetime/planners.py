"""Planner wrappers used by the closed-loop runner.

Two planners share one interface:

``SpaceTimePlanner``  plans over ``(x, y, t)`` against the agents' known timelines.
``BaselineReplanPlanner``  plain grid A* that freezes the agents where they are
right now and replans every step.
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from depot_planner import core
from depot_planner.config import load_config
from depot_planner.grid_astar import backend as _backend
from depot_planner.grid_astar.search import SearchResult, astar
from depot_planner.spacetime import search as st
from depot_planner.spacetime.collision import AgentOccupancy
from depot_planner.world.grid import Grid

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
    #: The agents flattened for the C++ core, built once per scenario.
    agent_timelines: list[tuple[np.ndarray, int, int]] = field(default_factory=list)


class Planner:
    """Common interface. ``replan_every`` lets a planner set its own cadence."""

    name: str = "planner"
    replan_every: int | None = None

    def __init__(
        self, config: dict[str, Any] | None = None, backend: str | None = None
    ) -> None:
        self.config = config if config is not None else load_config("spacetime")
        #: ``"cpp"``, ``"python"`` or ``None``/``"auto"``; see depot_planner.core.
        self.backend = backend
        self._cached_for: Any = None
        self._cache: _ScenarioCache | None = None

    def prepare(self, scenario) -> _ScenarioCache:
        """Per-scenario derived data, kept across the replans of one episode.

        The cache is keyed on object identity and holds a reference to the
        scenario, so a recycled ``id()`` can never serve a stale cost map.
        """
        if self._cache is None or self._cached_for is not scenario:
            grid: Grid = scenario.grid
            self._cache = _ScenarioCache(
                grid.cost_map(), grid.drivable, grid.min_cell_cost(),
                agent_timelines=st.agent_timelines_for(scenario.agents),
            )
            self._cached_for = scenario
        return self._cache

    def reset(self) -> None:
        """Drop the per-scenario cache."""
        self._cache = None
        self._cached_for = None

    def plan(self, scenario, cell: Cell, t: int) -> PlanResult:  # pragma: no cover - interface
        """Plan from ``cell`` at step ``t`` towards the scenario's goal."""
        raise NotImplementedError


class SpaceTimePlanner(Planner):
    """Space-time A* over the agents' known timelines."""

    name = "spacetime_astar"

    def prepare(self, scenario) -> _ScenarioCache:
        """Also precompute the goal cost-to-go field the grid heuristic needs."""
        cached = super().prepare(scenario)
        if cached.goal_field is None and str(
            self.config["spacetime"]["heuristic"]
        ).lower() == "grid_dijkstra":
            cached.goal_field = st.goal_cost_field(cached.cost_map, cached.drivable, scenario.goal)
        return cached

    def plan(self, scenario, cell: Cell, t: int) -> PlanResult:
        """Plan a timed path that avoids the agents' known future trajectories."""
        cached = self.prepare(scenario)
        # Planning past the episode's own time limit cannot help, and searching that
        # far is what makes a hopeless replan expensive.
        horizon = min(int(self.config["spacetime"]["max_time_horizon"]), int(scenario.time_limit))
        result = st.plan(
            cached.cost_map, cached.drivable, cell, scenario.goal, scenario.agents,
            start_time=t, min_cell_cost=cached.min_cell_cost, config=self.config,
            collect_expanded=False, heuristic_field=cached.goal_field, max_time=horizon,
            backend=self.backend, agent_timelines=cached.agent_timelines,
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
        """Plan with the agents frozen where they stand at step ``t``."""
        cached = self.prepare(scenario)
        cfg = self.config["baseline"]
        budget_ms = cfg.get("time_limit_ms")
        began = time.perf_counter()

        if core.resolve(self.backend) == "cpp":
            result = _snapshot_plan_cpp(cached, cell, scenario.goal, t, int(cfg["inflate"]),
                                        budget_ms)
        else:
            result = _snapshot_plan_python(cached, scenario.agents, cell, scenario.goal, t,
                                           int(cfg["inflate"]), budget_ms)
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



def _snapshot_plan_python(
    cached: _ScenarioCache,
    agents,
    cell: Cell,
    goal: Cell,
    t: int,
    inflate: int,
    budget_ms: float | None,
):
    """The reference snapshot plan: block the margin in numpy, then step-1 A*."""
    occupancy = AgentOccupancy(agents, inflate)
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
    return astar(cost_map, cell, goal, drivable=drivable, backend="python",
                 min_cell_cost=cached.min_cell_cost, collect_expanded=False,
                 time_limit_ms=None if budget_ms is None else float(budget_ms))


def _snapshot_plan_cpp(
    cached: _ScenarioCache,
    cell: Cell,
    goal: Cell,
    t: int,
    inflate: int,
    budget_ms: float | None,
):
    """The same plan in the C++ core: it blocks the margin and searches in one call."""
    effective = _backend.effective_cost_map(cached.cost_map, cached.drivable)
    max_nodes = int(load_config("grid")["search"]["max_nodes"])
    success, path, cost, nodes, runtime_ms, reason = core.require().baseline_plan(
        effective,
        (int(cell[0]), int(cell[1])),
        (int(goal[0]), int(goal[1])),
        int(t),
        list(cached.agent_timelines),
        int(inflate),
        float(cached.min_cell_cost),
        max_nodes,
        -1.0 if budget_ms is None else float(budget_ms),  # negative means unlimited
    )
    return SearchResult(bool(success), [(int(x), int(y)) for x, y in path] if success else None,
                        float(cost), int(nodes), float(runtime_ms), set(), str(reason))


PLANNERS: dict[str, type[Planner]] = {
    SpaceTimePlanner.name: SpaceTimePlanner,
    BaselineReplanPlanner.name: BaselineReplanPlanner,
}


def make_planner(
    name: str, config: dict[str, Any] | None = None, backend: str | None = None
) -> Planner:
    """Construct a planner by name; raises on an unknown name."""
    if name not in PLANNERS:
        raise ValueError(f"unknown planner {name!r}; expected one of {sorted(PLANNERS)}")
    return PLANNERS[name](config, backend)
