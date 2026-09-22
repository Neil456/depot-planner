"""The C++ core must match the Python grid search exactly, or be absent.

These checks date from the original step 7, when only the grid search was
ported; they still pin the backend dispatch helpers and the step-1 equivalence
over 200 start/goal pairs. The later ports are covered by test_cpp_core.py,
test_cpp_spacetime.py, test_cpp_hybrid.py and test_cpp_runner.py.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from depot_planner.eval.cpp_bench import compare_backends, summarise_cpp
from depot_planner.grid_astar import backend as backend_module
from depot_planner.grid_astar.search import MOVES, astar, dijkstra, search, weighted_astar
from depot_planner.world.depot_maps import generate_depot, sample_start_goal_pairs
from depot_planner.world.grid import CellType, Grid

PAIRS = 200

needs_cpp = pytest.mark.skipif(
    not backend_module.extension_available(),
    reason="the C++ core is not built",
)


@pytest.fixture(scope="module")
def problems():
    grid = generate_depot(seed=0)
    rng = np.random.default_rng(31337)
    pairs = sample_start_goal_pairs(grid, PAIRS, rng, min_separation=15.0)
    return grid, grid.cost_map(), grid.min_cell_cost(), pairs


def _both(grid, cost_map, min_cost, start, goal, **kwargs):
    common = dict(drivable=grid.drivable, min_cell_cost=min_cost, collect_expanded=False, **kwargs)
    return (
        search(cost_map, start, goal, backend="python", **common),
        search(cost_map, start, goal, backend="cpp", **common),
    )


# ------------------------------------------------------------- availability


def test_the_backend_reports_itself_honestly():
    name = backend_module.backend_name()
    assert name in ("python", "cpp")
    assert (name == "cpp") == backend_module.extension_available()


def test_the_python_backend_always_works(problems):
    grid, cost_map, min_cost, pairs = problems
    start, goal = pairs[0]
    result = search(cost_map, start, goal, drivable=grid.drivable, min_cell_cost=min_cost,
                    backend="python")
    assert result.success


def test_an_unknown_backend_is_rejected(problems):
    grid, cost_map, min_cost, pairs = problems
    start, goal = pairs[0]
    with pytest.raises(ValueError):
        search(cost_map, start, goal, drivable=grid.drivable, backend="assembly")


def test_auto_falls_back_to_python_when_expanded_cells_are_wanted(problems):
    """`collect_expanded` is Python-only, so `auto` must not lose it."""
    grid, cost_map, min_cost, pairs = problems
    start, goal = pairs[0]
    result = search(cost_map, start, goal, drivable=grid.drivable, min_cell_cost=min_cost,
                    backend="auto", collect_expanded=True)
    assert result.success
    assert len(result.expanded) == result.nodes_expanded


def test_the_helper_refuses_calls_the_core_cannot_serve(problems):
    grid, cost_map, min_cost, _ = problems
    assert not backend_module.can_use_cpp("octile", True, cost_map, min_cost)
    assert not backend_module.can_use_cpp("astral_projection", False, cost_map, min_cost)
    # A caller-supplied heuristic scale that disagrees with the grid changes the search.
    assert not backend_module.can_use_cpp("octile", False, cost_map, min_cost * 0.5)
    assert backend_module.heuristic_weight("dijkstra", 3.0) == 0.0
    assert backend_module.heuristic_weight("octile", 1.5) == 1.5
    assert backend_module.heuristic_weight("nonsense", 1.0) is None


def test_a_drivable_mask_is_folded_into_the_cost_map():
    cost = np.ones((3, 3))
    mask = np.ones((3, 3), dtype=bool)
    mask[1, 1] = False
    folded = backend_module.effective_cost_map(cost, mask)
    assert math.isinf(folded[1, 1])
    assert folded[0, 0] == 1.0
    assert np.array_equal(backend_module.effective_cost_map(cost, None), cost)


# --------------------------------------------------------------- equivalence


@needs_cpp
def test_python_and_cpp_agree_on_cost_and_path_over_200_pairs(problems):
    grid, cost_map, min_cost, pairs = problems
    weight = 1.5
    for start, goal in pairs:
        for kwargs in ({"heuristic": "none"}, {"heuristic": "octile"},
                       {"heuristic": "octile", "weight": weight}):
            python_result, cpp_result = _both(grid, cost_map, min_cost, start, goal, **kwargs)
            assert python_result.success == cpp_result.success
            assert cpp_result.cost == pytest.approx(python_result.cost, abs=1e-9, rel=0.0)
            assert cpp_result.path == python_result.path
            assert cpp_result.nodes_expanded == python_result.nodes_expanded


@needs_cpp
def test_the_cpp_paths_are_valid(problems):
    grid, cost_map, min_cost, pairs = problems
    steps = {(dx, dy) for dx, dy, _ in MOVES}
    for start, goal in pairs:
        _, result = _both(grid, cost_map, min_cost, start, goal)
        assert result.success
        assert result.path[0] == start and result.path[-1] == goal
        for x, y in result.path:
            assert grid.cells[y, x] != CellType.OBSTACLE
        for (x0, y0), (x1, y1) in zip(result.path, result.path[1:]):
            assert (x1 - x0, y1 - y0) in steps


@needs_cpp
def test_the_cpp_path_cost_adds_up(problems):
    grid, cost_map, min_cost, pairs = problems
    for start, goal in pairs[:25]:
        _, result = _both(grid, cost_map, min_cost, start, goal)
        total = 0.0
        for (x0, y0), (x1, y1) in zip(result.path, result.path[1:]):
            length = math.hypot(x1 - x0, y1 - y0)
            total += length * 0.5 * (cost_map[y0, x0] + cost_map[y1, x1])
        assert result.cost == pytest.approx(total, rel=1e-9)


@needs_cpp
def test_the_wrappers_dispatch_to_the_core_identically(problems):
    grid, cost_map, min_cost, pairs = problems
    common = dict(drivable=grid.drivable, min_cell_cost=min_cost, collect_expanded=False)
    for start, goal in pairs[:40]:
        for fn in (dijkstra, astar):
            assert fn(cost_map, start, goal, backend="cpp", **common).path == \
                   fn(cost_map, start, goal, backend="python", **common).path
        assert weighted_astar(cost_map, start, goal, weight=1.5, backend="cpp", **common).cost == \
            pytest.approx(
                weighted_astar(cost_map, start, goal, weight=1.5, backend="python", **common).cost
            )


# ----------------------------------------------------- failure modes match


@needs_cpp
def test_both_backends_report_an_unreachable_goal_the_same_way():
    cells = np.zeros((7, 7), dtype=np.uint8)
    cells[3, :] = CellType.OBSTACLE
    grid = Grid(cells)
    for backend in ("python", "cpp"):
        result = search(grid.cost_map(), (1, 1), (5, 5), drivable=grid.drivable,
                        collect_expanded=False, backend=backend)
        assert result.success is False
        assert result.reason == "goal unreachable"
        assert math.isinf(result.cost)


@needs_cpp
def test_both_backends_reject_an_undrivable_endpoint(problems):
    grid, cost_map, min_cost, _ = problems
    ys, xs = np.nonzero(grid.cells == CellType.OBSTACLE)
    goal = (int(xs[0]), int(ys[0]))
    for backend in ("python", "cpp"):
        result = search(cost_map, (2, 7), goal, drivable=grid.drivable, collect_expanded=False,
                        backend=backend)
        assert result.success is False
        assert "not drivable" in result.reason


@needs_cpp
def test_both_backends_honour_the_node_cap(problems):
    grid, cost_map, min_cost, pairs = problems
    start, goal = pairs[0]
    for backend in ("python", "cpp"):
        result = search(cost_map, start, goal, drivable=grid.drivable, min_cell_cost=min_cost,
                        collect_expanded=False, max_nodes=25, backend=backend)
        assert result.success is False
        assert result.reason == "node limit reached"
        assert result.nodes_expanded == 25


@needs_cpp
def test_both_backends_honour_a_wall_clock_budget(problems):
    grid, cost_map, min_cost, pairs = problems
    start, goal = pairs[0]
    for backend in ("python", "cpp"):
        result = search(cost_map, start, goal, drivable=grid.drivable, min_cell_cost=min_cost,
                        collect_expanded=False, time_limit_ms=0.0, backend=backend)
        assert result.success is False
        assert result.reason == "time limit reached"


@needs_cpp
def test_asking_for_the_core_with_expanded_cells_is_an_error(problems):
    grid, cost_map, min_cost, pairs = problems
    start, goal = pairs[0]
    with pytest.raises(ValueError):
        search(cost_map, start, goal, drivable=grid.drivable, collect_expanded=True, backend="cpp")


# ------------------------------------------------------------- the benchmark


@needs_cpp
def test_the_benchmark_shows_agreement_and_reports_a_speedup():
    """The grid slice of the Python-vs-C++ benchmark, kept small for the suite."""
    frame = compare_backends(pairs=6, repeats=1, planners=["grid"])
    assert len(frame) == 6 * 3
    assert frame["agree"].all()
    summary = summarise_cpp(frame)
    assert len(summary) == 3
    assert summary["identical"].all()
    assert (summary["median_speedup"] > 1.0).all()


@needs_cpp
def test_the_benchmark_covers_every_planner():
    frame = compare_backends(pairs=2, repeats=1, scenarios=1)
    assert set(frame["planner"]) == {
        "grid A* (Dijkstra)", "grid A*", "grid A* (weighted, w=1.5)",
        "space-time A*", "hybrid A*",
    }
    assert frame["agree"].all()
    # Setup is excluded from the timing, so nothing should read as free.
    assert (frame["cpp_ms"] > 0.0).all()
    assert (frame["python_ms"] > 0.0).all()
