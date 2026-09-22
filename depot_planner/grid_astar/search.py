"""One generic best-first search over an 8-connected grid.

The same routine implements Dijkstra (``heuristic="none"``), A*
(``heuristic="octile"``) and weighted A* (``heuristic="octile"`` with
``weight > 1``).

Cost model
----------
A move between two adjacent cells costs ``length * mean(cost[a], cost[b])``
where ``length`` is 1 for a straight move and ``sqrt(2)`` for a diagonal one.
Diagonal moves may not cut the corner of an obstacle: both orthogonal
neighbours shared by the two cells must be drivable.

The octile heuristic is scaled by the cheapest drivable cell cost, which makes
it admissible and consistent for this cost model.
"""

from __future__ import annotations

import heapq
import math
import time
from dataclasses import dataclass, field
from typing import Any, Iterable, Sequence

import numpy as np

from depot_planner.config import load_config
from depot_planner.grid_astar import backend as _backend

Cell = tuple[int, int]

SQRT2 = math.sqrt(2.0)

# (dx, dy, length) for the 8-connected neighbourhood.
MOVES: tuple[tuple[int, int, float], ...] = (
    (1, 0, 1.0),
    (-1, 0, 1.0),
    (0, 1, 1.0),
    (0, -1, 1.0),
    (1, 1, SQRT2),
    (1, -1, SQRT2),
    (-1, 1, SQRT2),
    (-1, -1, SQRT2),
)


@dataclass
class SearchResult:
    """Outcome of a single search."""

    success: bool
    path: list[Cell] | None
    cost: float
    nodes_expanded: int
    runtime_ms: float
    expanded: set[Cell] = field(default_factory=set)
    reason: str = ""

    @property
    def path_length_m(self) -> float:
        """Geometric length of the path in cell units."""
        if not self.path:
            return 0.0
        total = 0.0
        for (x0, y0), (x1, y1) in zip(self.path, self.path[1:]):
            total += math.hypot(x1 - x0, y1 - y0)
        return total


def octile(a: Cell, b: Cell) -> float:
    """Octile distance in cell units between two cells."""
    dx = abs(a[0] - b[0])
    dy = abs(a[1] - b[1])
    return (dx + dy) + (SQRT2 - 2.0) * min(dx, dy)


def make_heuristic(goal: Cell, kind: str, scale: float, weight: float):
    """Build the heuristic function used by :func:`search`."""
    kind = kind.lower()
    if kind in ("none", "zero", "dijkstra"):
        return lambda cell: 0.0
    if kind in ("octile", "astar", "a*"):
        factor = scale * weight

        def heuristic(cell: Cell) -> float:
            return octile(cell, goal) * factor

        return heuristic
    raise ValueError(f"unknown heuristic {kind!r}")


def _validated(cost_map: np.ndarray, drivable: np.ndarray | None) -> tuple[np.ndarray, np.ndarray]:
    cost = np.asarray(cost_map, dtype=float)
    if drivable is None:
        mask = np.isfinite(cost)
    else:
        mask = np.asarray(drivable, dtype=bool) & np.isfinite(cost)
    return cost, mask


def search(
    cost_map: np.ndarray,
    start: Cell,
    goal: Cell,
    *,
    drivable: np.ndarray | None = None,
    heuristic: str = "octile",
    weight: float = 1.0,
    min_cell_cost: float | None = None,
    max_nodes: int | None = None,
    time_limit_ms: float | None = None,
    collect_expanded: bool = True,
    config: dict[str, Any] | None = None,
    backend: str = "auto",
) -> SearchResult:
    """Best-first search from ``start`` to ``goal`` over an 8-connected grid.

    ``time_limit_ms`` is an optional wall-clock budget; on expiry the search
    returns a clean "no path" result rather than a partial one.

    ``backend`` selects the implementation: ``"python"`` for this module,
    ``"cpp"`` for the compiled core (an error if it was not built), or
    ``"auto"`` (the default) for the core when it is available and the call
    does not need the expanded-cell set, falling back to Python otherwise. The
    two produce identical paths and costs; see ``tests/test_cpp_backend.py``.
    """
    cfg = config if config is not None else load_config("grid")
    if max_nodes is None:
        max_nodes = int(cfg["search"]["max_nodes"])

    if backend not in ("auto", "python", "cpp"):
        raise ValueError(f"unknown backend {backend!r}; expected auto, python or cpp")
    if backend != "python":
        usable = _backend.can_use_cpp(heuristic, collect_expanded, cost_map, min_cell_cost)
        if backend == "cpp":
            if not _backend.extension_available():
                raise RuntimeError("the C++ core is not built; reinstall with `pip install -e .`")
            if collect_expanded:
                raise ValueError("the C++ core does not report expanded cells")
            if not usable:
                raise ValueError("this call cannot be served by the C++ core")
        if usable:
            return _search_cpp(
                cost_map, start, goal, drivable, heuristic, weight, max_nodes, time_limit_ms
            )

    cost, mask = _validated(cost_map, drivable)
    height, width = cost.shape
    start = (int(start[0]), int(start[1]))
    goal = (int(goal[0]), int(goal[1]))
    began = time.perf_counter()

    def elapsed_ms() -> float:
        return (time.perf_counter() - began) * 1000.0

    def in_bounds(x: int, y: int) -> bool:
        return 0 <= x < width and 0 <= y < height

    for cell, label in ((start, "start"), (goal, "goal")):
        if not in_bounds(*cell):
            return SearchResult(False, None, math.inf, 0, elapsed_ms(), set(), f"{label} out of bounds")
        if not mask[cell[1], cell[0]]:
            return SearchResult(False, None, math.inf, 0, elapsed_ms(), set(), f"{label} not drivable")

    if min_cell_cost is None:
        min_cell_cost = float(cost[mask].min()) if mask.any() else 0.0
    h_of = make_heuristic(goal, heuristic, min_cell_cost, weight)

    g_score: dict[Cell, float] = {start: 0.0}
    parent: dict[Cell, Cell] = {}
    closed: set[Cell] = set()
    expanded: set[Cell] = set()
    heap: list[tuple[float, float, int, Cell]] = []
    counter = 0
    heapq.heappush(heap, (h_of(start), 0.0, counter, start))
    nodes_expanded = 0

    while heap:
        f, g, _, current = heapq.heappop(heap)
        if current in closed:
            continue
        closed.add(current)
        nodes_expanded += 1
        if collect_expanded:
            expanded.add(current)

        if current == goal:
            path = _reconstruct(parent, start, goal)
            return SearchResult(True, path, g, nodes_expanded, elapsed_ms(), expanded, "goal reached")

        if nodes_expanded >= max_nodes:
            return SearchResult(
                False, None, math.inf, nodes_expanded, elapsed_ms(), expanded, "node limit reached"
            )
        if time_limit_ms is not None and elapsed_ms() > time_limit_ms:
            return SearchResult(
                False, None, math.inf, nodes_expanded, elapsed_ms(), expanded, "time limit reached"
            )

        cx, cy = current
        current_cost = cost[cy, cx]
        for dx, dy, length in MOVES:
            nx, ny = cx + dx, cy + dy
            if not in_bounds(nx, ny) or not mask[ny, nx]:
                continue
            if dx != 0 and dy != 0:
                # No corner cutting past an obstacle.
                if not mask[cy, nx] or not mask[ny, cx]:
                    continue
            neighbour = (nx, ny)
            if neighbour in closed:
                continue
            tentative = g + length * 0.5 * (current_cost + cost[ny, nx])
            if tentative < g_score.get(neighbour, math.inf) - 1e-12:
                g_score[neighbour] = tentative
                parent[neighbour] = current
                counter += 1
                heapq.heappush(heap, (tentative + h_of(neighbour), tentative, counter, neighbour))

    return SearchResult(False, None, math.inf, nodes_expanded, elapsed_ms(), expanded, "goal unreachable")


def _search_cpp(
    cost_map: np.ndarray,
    start: Cell,
    goal: Cell,
    drivable: np.ndarray | None,
    heuristic: str,
    weight: float,
    max_nodes: int,
    time_limit_ms: float | None,
) -> SearchResult:
    """Run the compiled core and wrap its answer as a :class:`SearchResult`."""
    effective = _backend.effective_cost_map(cost_map, drivable)
    cpp_weight = _backend.heuristic_weight(heuristic, weight)
    success, path, cost, nodes, runtime_ms, reason = _backend.solve(
        effective, start, goal, cpp_weight, max_nodes, time_limit_ms
    )
    return SearchResult(success, path, cost, nodes, runtime_ms, set(), reason)


def _reconstruct(parent: dict[Cell, Cell], start: Cell, goal: Cell) -> list[Cell]:
    path = [goal]
    node = goal
    while node != start:
        node = parent[node]
        path.append(node)
    path.reverse()
    return path


def dijkstra(cost_map: np.ndarray, start: Cell, goal: Cell, **kwargs: Any) -> SearchResult:
    """Uniform-cost search (A* with a zero heuristic)."""
    kwargs.pop("heuristic", None)
    kwargs.pop("weight", None)
    return search(cost_map, start, goal, heuristic="none", weight=1.0, **kwargs)


def astar(cost_map: np.ndarray, start: Cell, goal: Cell, **kwargs: Any) -> SearchResult:
    """A* with the admissible octile heuristic."""
    kwargs.pop("heuristic", None)
    kwargs.pop("weight", None)
    return search(cost_map, start, goal, heuristic="octile", weight=1.0, **kwargs)


def weighted_astar(
    cost_map: np.ndarray, start: Cell, goal: Cell, weight: float | None = None, **kwargs: Any
) -> SearchResult:
    """A* with an inflated heuristic; the result is at most ``weight`` x optimal."""
    kwargs.pop("heuristic", None)
    if weight is None:
        cfg = kwargs.get("config") or load_config("grid")
        weight = float(cfg["search"]["weight"])
    return search(cost_map, start, goal, heuristic="octile", weight=float(weight), **kwargs)


def dijkstra_field(
    cost_map: np.ndarray,
    sources: Iterable[Cell],
    *,
    drivable: np.ndarray | None = None,
) -> np.ndarray:
    """Cost-to-come from ``sources`` to every drivable cell (``inf`` elsewhere).

    Used as the obstacle-aware heuristic for hybrid A* in step 5.
    """
    cost, mask = _validated(cost_map, drivable)
    height, width = cost.shape
    dist = np.full(cost.shape, np.inf, dtype=float)
    heap: list[tuple[float, int, int]] = []
    for sx, sy in sources:
        sx, sy = int(sx), int(sy)
        if 0 <= sx < width and 0 <= sy < height and mask[sy, sx] and dist[sy, sx] > 0.0:
            dist[sy, sx] = 0.0
            heapq.heappush(heap, (0.0, sx, sy))

    while heap:
        d, cx, cy = heapq.heappop(heap)
        if d > dist[cy, cx]:
            continue
        current_cost = cost[cy, cx]
        for dx, dy, length in MOVES:
            nx, ny = cx + dx, cy + dy
            if not (0 <= nx < width and 0 <= ny < height) or not mask[ny, nx]:
                continue
            if dx != 0 and dy != 0 and (not mask[cy, nx] or not mask[ny, cx]):
                continue
            nd = d + length * 0.5 * (current_cost + cost[ny, nx])
            if nd < dist[ny, nx] - 1e-12:
                dist[ny, nx] = nd
                heapq.heappush(heap, (nd, nx, ny))
    return dist


def path_cells(path: Sequence[Cell] | None) -> list[Cell]:
    return list(path) if path else []
