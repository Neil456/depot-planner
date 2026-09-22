"""C++ port, step 2: the shared core types, grid search and distance fields.

Every check here compares the compiled core against the pure-Python reference
implementation on the same inputs. The reference is never adjusted to make the
core pass; where the two genuinely differ (the obstacle-free mask below) the
difference is pinned by a test and explained in docs/DECISIONS.md.
"""

from __future__ import annotations

import math

import numpy as np
import pytest
from scipy.ndimage import distance_transform_edt

from depot_planner import core
from depot_planner.grid_astar.search import dijkstra_field, search
from depot_planner.hybrid.collision import DistanceField
from depot_planner.world.depot_maps import generate_depot, sample_start_goal_pairs
from depot_planner.world.grid import CellType, Grid
from depot_planner.world.parking import PARKING_TYPES, generate_parking_scenario

CASES = 200

needs_cpp = pytest.mark.skipif(not core.available(), reason="the C++ core is not built")


@pytest.fixture(scope="module")
def masks() -> list[np.ndarray]:
    """200 random masks, each with at least one obstacle cell."""
    rng = np.random.default_rng(90210)
    out: list[np.ndarray] = []
    for _ in range(CASES):
        rows = int(rng.integers(2, 40))
        cols = int(rng.integers(2, 40))
        density = float(rng.uniform(0.02, 0.6))
        mask = rng.random((rows, cols)) > density
        if mask.all():                       # guarantee at least one obstacle
            mask[int(rng.integers(rows)), int(rng.integers(cols))] = False
        out.append(np.ascontiguousarray(mask))
    return out


# ------------------------------------------------------------ backend choice


def test_backend_names_are_validated():
    assert core.resolve("python") == "python"
    assert core.resolve("auto") in ("cpp", "python")
    with pytest.raises(ValueError):
        core.resolve("fortran")


def test_auto_prefers_the_core_when_it_is_built():
    assert core.resolve(None) == ("cpp" if core.available() else "python")
    assert core.resolve("auto") == core.resolve(None)


# ------------------------------------------------- the Euclidean transform


@needs_cpp
def test_the_cpp_transform_matches_scipy_bit_for_bit(masks):
    for mask in masks:
        theirs = distance_transform_edt(mask)
        ours = core.require().distance_transform(mask)
        assert np.array_equal(ours, theirs), f"mismatch on a {mask.shape} mask"


@needs_cpp
def test_a_mask_with_no_obstacle_is_reported_as_infinitely_far(recwarn):
    """The one deliberate divergence from scipy; see docs/DECISIONS.md.

    With no background cell there is no nearest obstacle. The core says so;
    scipy returns finite numbers that are an artefact of its own algorithm.
    """
    mask = np.ones((3, 4), dtype=bool)
    ours = core.require().distance_transform(mask)
    assert np.isinf(ours).all()
    assert np.isfinite(distance_transform_edt(mask)).all()


@needs_cpp
def test_the_grid_obstacle_distance_agrees_with_scipy(depot):
    ours = depot.obstacle_distance(backend="cpp")
    theirs = depot.obstacle_distance(backend="python")
    assert np.array_equal(ours, theirs)
    assert np.array_equal(theirs, distance_transform_edt(depot.drivable) * depot.resolution)


@needs_cpp
def test_the_cost_map_is_unchanged_by_the_backend(depot):
    cpp_grid = Grid(depot.cells.copy(), depot.resolution, depot.config)
    python_penalty = cpp_grid.proximity_penalty(backend="python")
    cpp_penalty = cpp_grid.proximity_penalty(backend="cpp")
    assert np.array_equal(python_penalty, cpp_penalty)


@needs_cpp
@pytest.mark.parametrize("parking_type", PARKING_TYPES)
def test_the_parking_distance_field_is_identical(parking_type):
    scenario = generate_parking_scenario(parking_type, 11)
    resolution = float(scenario.hybrid_config["collision"]["field_resolution"])
    common = (scenario.obstacles, scenario.width_m, scenario.height_m, resolution)
    ours = DistanceField.from_rectangles(*common, backend="cpp")
    theirs = DistanceField.from_rectangles(*common, backend="python")
    assert np.array_equal(ours.distance, theirs.distance)


# ------------------------------------------------------- the Dijkstra field


@needs_cpp
def test_the_cpp_dijkstra_field_matches_the_reference(depot):
    cost_map = depot.cost_map()
    rng = np.random.default_rng(777)
    pairs = sample_start_goal_pairs(depot, 25, rng, min_separation=10.0)
    for source, _ in pairs:
        ours = dijkstra_field(cost_map, [source], drivable=depot.drivable, backend="cpp")
        theirs = dijkstra_field(cost_map, [source], drivable=depot.drivable, backend="python")
        assert np.array_equal(ours, theirs)


@needs_cpp
def test_the_dijkstra_field_agrees_with_running_the_search(depot):
    cost_map = depot.cost_map()
    goal = (12, 8)
    field = dijkstra_field(cost_map, [goal], drivable=depot.drivable, backend="cpp")
    rng = np.random.default_rng(31)
    pairs = sample_start_goal_pairs(depot, 20, rng, min_separation=10.0)
    for start, _ in pairs:
        result = search(cost_map, start, goal, drivable=depot.drivable, heuristic="none",
                        collect_expanded=False, backend="cpp")
        assert result.success
        assert field[start[1], start[0]] == pytest.approx(result.cost, abs=1e-9, rel=0.0)


@needs_cpp
def test_the_field_handles_several_sources_and_unreachable_cells():
    cells = np.zeros((9, 9), dtype=np.uint8)
    cells[4, :] = CellType.OBSTACLE
    grid = Grid(cells)
    sources = [(1, 1), (7, 1)]
    ours = dijkstra_field(grid.cost_map(), sources, drivable=grid.drivable, backend="cpp")
    theirs = dijkstra_field(grid.cost_map(), sources, drivable=grid.drivable, backend="python")
    assert np.array_equal(ours, theirs)
    assert ours[1, 1] == 0.0 and ours[1, 7] == 0.0
    assert math.isinf(ours[6, 3])


# --------------------------------------------------- the templated search


@needs_cpp
def test_the_three_search_variants_agree_over_200_cases():
    """Dijkstra, A* and weighted A* must each match the reference exactly."""
    grid = generate_depot(seed=3)
    cost_map = grid.cost_map()
    min_cost = grid.min_cell_cost()
    rng = np.random.default_rng(6060)
    pairs = sample_start_goal_pairs(grid, CASES, rng, min_separation=12.0)
    common = dict(drivable=grid.drivable, min_cell_cost=min_cost, collect_expanded=False)
    variants = ({"heuristic": "none"}, {"heuristic": "octile"},
                {"heuristic": "octile", "weight": 1.5})
    for start, goal in pairs:
        for variant in variants:
            ours = search(cost_map, start, goal, backend="cpp", **common, **variant)
            theirs = search(cost_map, start, goal, backend="python", **common, **variant)
            assert ours.success == theirs.success
            assert ours.cost == pytest.approx(theirs.cost, abs=1e-12, rel=0.0)
            assert ours.path == theirs.path
            assert ours.nodes_expanded == theirs.nodes_expanded
