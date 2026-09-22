"""Space-time A*: A* over states ``(x, y, t)`` where the ego may also wait.

Actions are the eight grid moves of step 1 plus a ``wait`` that keeps the ego in
its cell for one step. Every action advances time by one step and is charged the
step-1 movement cost plus a configurable per-step time cost, so waiting is
allowed but never free.

Collision rules
---------------
* the ego may not occupy a cell inside an agent footprint inflated by ``inflate``
  cells at time ``t``;
* the ego and an agent may not swap cells between ``t`` and ``t + 1``.

If the ego already stands inside the inflated margin (which the closed-loop
runner can produce, since the margin is a comfort buffer and not a real
collision) the margin is relaxed for one move: the ego may move within it, but
never into an agent body. Set ``relax_margin_when_inside: false`` to disable.
"""

from __future__ import annotations

import heapq
import math
import time
from dataclasses import dataclass, field
from typing import Any, Sequence

import numpy as np

from depot_planner.config import load_config
from depot_planner.grid_astar.search import MOVES, dijkstra_field, octile
from depot_planner.spacetime.collision import AgentOccupancy
from depot_planner.world.agents import Agent

Cell = tuple[int, int]
State = tuple[int, int, int]

#: (dx, dy, length) for the eight moves plus the wait action.
ACTIONS: tuple[tuple[int, int, float], ...] = MOVES + ((0, 0, 0.0),)


@dataclass
class SpaceTimeResult:
    """Outcome of one space-time search."""

    success: bool
    cells: list[Cell] | None
    times: list[int] | None
    cost: float
    movement_cost: float
    wait_steps: int
    nodes_expanded: int
    runtime_ms: float
    expanded: set[Cell] = field(default_factory=set)
    reason: str = ""

    @property
    def horizon(self) -> int:
        """Number of simulation steps the plan spans."""
        return 0 if not self.times else self.times[-1] - self.times[0]

    @property
    def path_length_m(self) -> float:
        """Geometric length of the planned path."""
        if not self.cells:
            return 0.0
        return sum(
            math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(self.cells, self.cells[1:])
        )


def chebyshev(a: Cell, b: Cell) -> int:
    """Chebyshev distance, the fewest steps any 8-connected path can take."""
    return max(abs(a[0] - b[0]), abs(a[1] - b[1]))


def goal_cost_field(cost_map: np.ndarray, drivable: np.ndarray, goal: Cell) -> np.ndarray:
    """Exact step-1 cost-to-go to ``goal`` on the static map, ignoring agents.

    The step-1 move cost is symmetric, so a Dijkstra expansion *from* the goal
    gives the cost of reaching the goal *from* every cell. Agents can only add
    cost or waiting time, so this never overestimates: it is admissible, and it
    is consistent because it is an exact distance under the same cost model.
    """
    return dijkstra_field(cost_map, [goal], drivable=drivable)


def plan(
    cost_map: np.ndarray,
    drivable: np.ndarray,
    start: Cell,
    goal: Cell,
    agents: Sequence[Agent],
    *,
    start_time: int = 0,
    min_cell_cost: float | None = None,
    config: dict[str, Any] | None = None,
    collect_expanded: bool = True,
    max_time: int | None = None,
    heuristic_field: np.ndarray | None = None,
) -> SpaceTimeResult:
    """Plan a timed path from ``start`` at ``start_time`` to ``goal``.

    ``heuristic_field`` is an optional cost-to-go field from the goal (see
    :func:`goal_cost_field`); pass it in to avoid recomputing it on every replan.
    """
    cfg = (config if config is not None else load_config("spacetime"))["spacetime"]
    time_cost = float(cfg["time_cost"])
    inflate = int(cfg["inflate"])
    relax = bool(cfg["relax_margin_when_inside"])
    use_time_h = bool(cfg["heuristic_includes_time"])
    kind = str(cfg["heuristic"]).lower()
    horizon_cap = int(max_time if max_time is not None else cfg["max_time_horizon"])
    max_nodes = int(cfg["max_nodes"])
    budget_ms = cfg.get("time_limit_ms")
    budget_ms = None if budget_ms is None else float(budget_ms)

    cost = np.asarray(cost_map, dtype=float)
    mask = np.asarray(drivable, dtype=bool) & np.isfinite(cost)
    height, width = cost.shape
    start = (int(start[0]), int(start[1]))
    goal = (int(goal[0]), int(goal[1]))
    began = time.perf_counter()

    def elapsed_ms() -> float:
        return (time.perf_counter() - began) * 1000.0

    def fail(reason: str, expanded: set[Cell], nodes: int) -> SpaceTimeResult:
        return SpaceTimeResult(False, None, None, math.inf, math.inf, 0, nodes, elapsed_ms(),
                               expanded, reason)

    for cell, label in ((start, "start"), (goal, "goal")):
        if not (0 <= cell[0] < width and 0 <= cell[1] < height) or not mask[cell[1], cell[0]]:
            return fail(f"{label} not drivable", set(), 0)
    if start_time > horizon_cap:
        return fail("start time beyond the horizon", set(), 0)

    if min_cell_cost is None:
        min_cell_cost = float(cost[mask].min()) if mask.any() else 0.0
    occupancy = AgentOccupancy(agents, inflate)

    if kind == "grid_dijkstra":
        field = heuristic_field if heuristic_field is not None else goal_cost_field(cost, mask, goal)
    elif kind == "octile":
        field = None
    else:
        raise ValueError(f"unknown space-time heuristic {kind!r}")

    def heuristic(cell: Cell) -> float:
        if field is None:
            h = octile(cell, goal) * min_cell_cost
        else:
            h = float(field[cell[1], cell[0]])
            if not math.isfinite(h):
                return math.inf
        if use_time_h:
            h += chebyshev(cell, goal) * time_cost
        return h

    start_state: State = (start[0], start[1], int(start_time))
    g_score: dict[State, float] = {start_state: 0.0}
    movement: dict[State, float] = {start_state: 0.0}
    parent: dict[State, State] = {}
    closed: set[State] = set()
    expanded: set[Cell] = set()
    # Ties on f are broken towards the larger g (deeper states first), which cuts the
    # number of equal-cost (x, y, t) plateaus the search has to walk through.
    heap: list[tuple[float, float, int, State]] = [(heuristic(start), -0.0, 0, start_state)]
    counter = 0
    nodes_expanded = 0

    while heap:
        _, neg_g, _, state = heapq.heappop(heap)
        g = -neg_g
        if state in closed:
            continue
        closed.add(state)
        nodes_expanded += 1
        cx, cy, ct = state
        if collect_expanded:
            expanded.add((cx, cy))

        if (cx, cy) == goal:
            cells, times = _reconstruct(parent, start_state, state)
            waits = sum(1 for a, b in zip(cells, cells[1:]) if a == b)
            return SpaceTimeResult(
                True, cells, times, g, movement[state], waits, nodes_expanded,
                elapsed_ms(), expanded, "goal reached",
            )

        if nodes_expanded >= max_nodes:
            return fail("node limit reached", expanded, nodes_expanded)
        if budget_ms is not None and elapsed_ms() > budget_ms:
            return fail("time limit reached", expanded, nodes_expanded)
        if ct >= horizon_cap:
            continue

        next_t = ct + 1
        margin_now = occupancy.margin(ct)
        margin_next = occupancy.margin(next_t)
        body_next = occupancy.body(next_t)
        inside_margin = relax and ((cx, cy) in margin_now)
        forbidden = body_next if inside_margin else margin_next
        current_cost = cost[cy, cx]

        for dx, dy, length in ACTIONS:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < width and 0 <= ny < height) or not mask[ny, nx]:
                continue
            if dx != 0 and dy != 0 and (not mask[cy, nx] or not mask[ny, cx]):
                continue  # no corner cutting
            target = (nx, ny)
            if target in forbidden:
                continue
            if (dx or dy) and occupancy.swaps((cx, cy), target, ct):
                continue
            successor: State = (nx, ny, next_t)
            if successor in closed:
                continue
            move_cost = length * 0.5 * (current_cost + cost[ny, nx])
            tentative = g + move_cost + time_cost
            h = heuristic(target)
            if not math.isfinite(h):
                continue  # the goal is unreachable from there on the static map
            if tentative < g_score.get(successor, math.inf) - 1e-12:
                g_score[successor] = tentative
                movement[successor] = movement[state] + move_cost
                parent[successor] = state
                counter += 1
                heapq.heappush(heap, (tentative + h, -tentative, counter, successor))

    return fail("no timed path to the goal", expanded, nodes_expanded)


def _reconstruct(parent: dict[State, State], start: State, goal: State) -> tuple[list[Cell], list[int]]:
    states = [goal]
    node = goal
    while node != start:
        node = parent[node]
        states.append(node)
    states.reverse()
    return [(s[0], s[1]) for s in states], [s[2] for s in states]
