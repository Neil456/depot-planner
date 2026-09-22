"""Step 1 checks: cost map, generic search, and the three algorithm variants."""

from __future__ import annotations

import math

import numpy as np
import pytest

from depot_planner.config import load_config
from depot_planner.grid_astar.search import (
    MOVES,
    astar,
    dijkstra,
    dijkstra_field,
    octile,
    search,
    weighted_astar,
)
from depot_planner.world.depot_maps import generate_depot, sample_start_goal_pairs
from depot_planner.world.grid import CellType, Grid

N_PAIRS = 50


@pytest.fixture(scope="module")
def problem():
    grid = generate_depot(seed=0)
    rng = np.random.default_rng(2024)
    pairs = sample_start_goal_pairs(grid, N_PAIRS, rng, min_separation=15.0)
    return grid, grid.cost_map(), grid.min_cell_cost(), pairs


def _run(fn, problem, start, goal, **kwargs):
    grid, cost_map, min_cost, _ = problem
    return fn(cost_map, start, goal, drivable=grid.drivable, min_cell_cost=min_cost, **kwargs)


# ------------------------------------------------------------------ cost map


def test_map_is_the_expected_size_and_has_all_three_cell_types(depot):
    assert depot.shape == (40, 60)
    present = set(np.unique(depot.cells).tolist())
    assert present == {int(CellType.AISLE), int(CellType.PARKING), int(CellType.OBSTACLE)}


def test_depot_generation_is_deterministic():
    assert np.array_equal(generate_depot(seed=3).cells, generate_depot(seed=3).cells)
    assert not np.array_equal(generate_depot(seed=0).cells, generate_depot(seed=1).cells)


def test_obstacle_cells_are_not_drivable_and_cost_infinity(depot):
    obstacles = depot.cells == CellType.OBSTACLE
    assert not depot.drivable[obstacles].any()
    assert np.isinf(depot.cost_map()[obstacles]).all()


def test_parking_costs_more_than_aisle_and_penalty_decays_with_distance(depot):
    cost = depot.cost_map()
    base = depot.base_cost_map()
    aisle = depot.cells == CellType.AISLE
    parking = depot.cells == CellType.PARKING
    assert base[aisle].max() < base[parking].min()

    penalty = depot.proximity_penalty()
    dist = depot.obstacle_distance()
    radius = load_config("grid")["obstacle_penalty"]["radius"]
    assert (penalty[dist >= radius] == 0.0).all()
    near, far = dist == 1.0, dist == 2.0
    if near.any() and far.any():
        assert penalty[near].max() > penalty[far].max()
    assert (cost[depot.drivable] >= base[depot.drivable]).all()


# -------------------------------------------------------------- the searches


def test_astar_and_dijkstra_agree_on_cost_over_random_pairs(problem):
    _, _, _, pairs = problem
    for start, goal in pairs:
        d = _run(dijkstra, problem, start, goal)
        a = _run(astar, problem, start, goal)
        assert d.success and a.success, f"no path for {start} -> {goal}"
        assert a.cost == pytest.approx(d.cost, rel=1e-9, abs=1e-9)


def test_astar_never_expands_more_nodes_than_dijkstra(problem):
    _, _, _, pairs = problem
    for start, goal in pairs:
        d = _run(dijkstra, problem, start, goal)
        a = _run(astar, problem, start, goal)
        assert a.nodes_expanded <= d.nodes_expanded


def test_weighted_astar_is_within_the_weight_bound(problem):
    _, _, _, pairs = problem
    weight = float(load_config("grid")["search"]["weight"])
    for start, goal in pairs:
        optimal = _run(astar, problem, start, goal)
        inflated = _run(weighted_astar, problem, start, goal, weight=weight)
        assert inflated.success
        assert inflated.cost <= weight * optimal.cost + 1e-9
        assert inflated.cost >= optimal.cost - 1e-9


def test_paths_are_connected_and_never_touch_obstacles(problem):
    grid, _, _, pairs = problem
    steps = {(dx, dy) for dx, dy, _ in MOVES}
    for start, goal in pairs:
        result = _run(astar, problem, start, goal)
        assert result.path[0] == start and result.path[-1] == goal
        for x, y in result.path:
            assert grid.cells[y, x] != CellType.OBSTACLE
        for (x0, y0), (x1, y1) in zip(result.path, result.path[1:]):
            assert (x1 - x0, y1 - y0) in steps


def test_reported_cost_matches_the_cost_of_the_returned_path(problem):
    grid, cost_map, _, pairs = problem
    for start, goal in pairs[:10]:
        result = _run(astar, problem, start, goal)
        total = 0.0
        for (x0, y0), (x1, y1) in zip(result.path, result.path[1:]):
            length = math.hypot(x1 - x0, y1 - y0)
            total += length * 0.5 * (cost_map[y0, x0] + cost_map[y1, x1])
        assert result.cost == pytest.approx(total, rel=1e-9)


def test_diagonal_moves_do_not_cut_obstacle_corners():
    cells = np.zeros((3, 3), dtype=np.uint8)
    cells[0, 1] = CellType.OBSTACLE
    cells[1, 0] = CellType.OBSTACLE
    grid = Grid(cells)
    result = astar(grid.cost_map(), (1, 1), (0, 0), drivable=grid.drivable)
    assert not result.success


def test_unreachable_goal_returns_a_clean_no_path_result():
    cells = np.zeros((7, 7), dtype=np.uint8)
    cells[3, :] = CellType.OBSTACLE  # wall splitting the map in two
    grid = Grid(cells)
    result = astar(grid.cost_map(), (1, 1), (5, 5), drivable=grid.drivable)
    assert result.success is False
    assert result.path is None
    assert math.isinf(result.cost)
    assert result.reason == "goal unreachable"
    assert result.nodes_expanded > 0


def test_goal_on_an_obstacle_is_rejected_without_searching(depot):
    ys, xs = np.nonzero(depot.cells == CellType.OBSTACLE)
    goal = (int(xs[0]), int(ys[0]))
    result = astar(depot.cost_map(), (2, 7), goal, drivable=depot.drivable)
    assert result.success is False
    assert result.nodes_expanded == 0
    assert "not drivable" in result.reason


def test_node_limit_is_reported_cleanly(depot):
    result = astar(depot.cost_map(), (2, 7), (57, 33), drivable=depot.drivable, max_nodes=25)
    assert result.success is False
    assert result.reason == "node limit reached"
    assert result.nodes_expanded == 25


# ------------------------------------------------------------- the heuristic


def test_octile_heuristic_never_overestimates_the_true_cost(problem):
    grid, _, min_cost, pairs = problem
    for start, goal in pairs:
        truth = _run(astar, problem, start, goal)
        assert octile(start, goal) * min_cost <= truth.cost + 1e-9


def test_dijkstra_field_matches_point_to_point_search(problem):
    grid, cost_map, _, pairs = problem
    source = pairs[0][0]
    field = dijkstra_field(cost_map, [source], drivable=grid.drivable)
    for _, goal in pairs[:15]:
        point = _run(astar, problem, source, goal)
        assert field[goal[1], goal[0]] == pytest.approx(point.cost, rel=1e-9)
