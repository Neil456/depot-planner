"""Dispatch between the pure-Python step-1 search and the C++ core.

The C++ extension (``depot_planner._cpp``, built from the ``cpp/`` library) is a
drop-in replacement for the step-1 search. The Python implementation is kept as
the reference and is still used when the extension is missing, or when a call
asks for something the core does not report.
"""

from __future__ import annotations

import math

import numpy as np

from depot_planner import core

#: Heuristic names the C++ core implements, mapped to its ``weight`` argument.
_ZERO_HEURISTICS = frozenset({"none", "zero", "dijkstra"})
_OCTILE_HEURISTICS = frozenset({"octile", "astar", "a*"})


def extension_available() -> bool:
    """True if the compiled core is importable."""
    return core.available()


def backend_name() -> str:
    """``"cpp"`` when the compiled core is importable, else ``"python"``."""
    return "cpp" if core.available() else "python"


def heuristic_weight(heuristic: str, weight: float) -> float | None:
    """The C++ ``weight`` for a Python heuristic name, or ``None`` if unsupported."""
    name = heuristic.lower()
    if name in _ZERO_HEURISTICS:
        return 0.0
    if name in _OCTILE_HEURISTICS:
        return float(weight)
    return None


def effective_cost_map(cost_map: np.ndarray, drivable: np.ndarray | None) -> np.ndarray:
    """A cost map whose only non-finite entries are the non-drivable cells.

    The C++ core reads drivability straight off the cost map, so a separate
    mask has to be folded in first.
    """
    cost = np.asarray(cost_map, dtype=float)
    if drivable is None:
        return np.ascontiguousarray(cost)
    mask = np.asarray(drivable, dtype=bool)
    if mask.all():
        return np.ascontiguousarray(cost)
    return np.ascontiguousarray(np.where(mask, cost, np.inf))


def min_finite(cost_map: np.ndarray) -> float:
    """Cheapest finite (drivable) cell cost, the heuristic scale the core derives."""
    finite = np.isfinite(cost_map)
    return float(cost_map[finite].min()) if finite.any() else 0.0


def solve(
    cost_map: np.ndarray,
    start: tuple[int, int],
    goal: tuple[int, int],
    weight: float,
    max_nodes: int,
    time_limit_ms: float | None,
) -> tuple[bool, list[tuple[int, int]] | None, float, int, float, str]:
    """Run the C++ core. Raises ``RuntimeError`` if it was not built."""
    success, path, cost, nodes, runtime_ms, reason = core.require().grid_astar(
        cost_map,
        (int(start[0]), int(start[1])),
        (int(goal[0]), int(goal[1])),
        float(weight),
        int(max_nodes),
        -1.0 if time_limit_ms is None else float(time_limit_ms),  # negative means unlimited
    )
    cells = [(int(x), int(y)) for x, y in path] if success else None
    return bool(success), cells, float(cost), int(nodes), float(runtime_ms), str(reason)


def solve_field(cost_map: np.ndarray, sources: list[tuple[int, int]]) -> np.ndarray:
    """Run the C++ Dijkstra field. Raises ``RuntimeError`` if it was not built."""
    return core.require().dijkstra_field(
        cost_map, [(int(x), int(y)) for x, y in sources]
    )


def can_use_cpp(
    heuristic: str,
    collect_expanded: bool,
    cost_map: np.ndarray,
    min_cell_cost: float | None,
) -> bool:
    """Whether a given call can be served identically by the C++ core."""
    if not extension_available() or collect_expanded:
        return False
    if heuristic_weight(heuristic, 1.0) is None:
        return False
    if min_cell_cost is None:
        return True
    # The core derives the heuristic scale from the grid; a caller-supplied
    # value that disagrees would change the search, so stay in Python.
    return math.isclose(float(min_cell_cost), min_finite(cost_map), rel_tol=0.0, abs_tol=1e-12)
