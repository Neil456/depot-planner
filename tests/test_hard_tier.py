"""Checks for the hard tier of both batteries.

The hard tier must be strictly additive: resolving a hard type may never change
the normal tier's configuration or results.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from depot_planner.config import deep_merge, load_config
from depot_planner.eval.battery import (
    HARD_BATTERY_CSV,
    TIERS,
    run_battery,
    tier_csv,
    tier_planner_config,
    tier_settings,
)
from depot_planner.eval.grid_bench import run_grid_benchmark, summarise_grid
from depot_planner.eval.parking_battery import (
    HARD_PARKING_CSV,
    parking_tier_csv,
    run_parking_battery,
    tier_hybrid_config,
)
from depot_planner.eval.report import format_value, markdown_table
from depot_planner.grid_astar.search import astar
from depot_planner.hybrid.car import CarModel
from depot_planner.hybrid.collision import CarCollisionChecker, DistanceField
from depot_planner.spacetime import search as st
from depot_planner.world.grid import CellType
from depot_planner.world.parking import (
    ALL_PARKING_TYPES,
    HARD_PARKING_TYPES,
    PARKING_TYPES,
    clearance_requirement,
    generate_parking_scenario,
    minimum_parallel_gap,
    minimum_spot_width,
    resolve_parking_type,
)
from depot_planner.world.scenarios import (
    ALL_SCENARIO_TYPES,
    HARD_SCENARIO_TYPES,
    SCENARIO_TYPES,
    generate_scenario,
    resolve_scenario_type,
)

HARD_SEEDS = range(3)


# ----------------------------------------------------------------- merging


def test_deep_merge_leaves_its_inputs_alone():
    base = {"a": {"b": 1, "c": 2}, "d": 3}
    overrides = {"a": {"b": 9}}
    merged = deep_merge(base, overrides)
    assert merged == {"a": {"b": 9, "c": 2}, "d": 3}
    assert base == {"a": {"b": 1, "c": 2}, "d": 3}
    assert overrides == {"a": {"b": 9}}


def test_resolving_a_normal_type_changes_nothing():
    for name in SCENARIO_TYPES:
        base, scenario_cfg, depot_cfg = resolve_scenario_type(name)
        assert base == name
        assert scenario_cfg == load_config("scenarios")
        assert depot_cfg == load_config("depot")


def test_resolving_a_hard_type_does_not_disturb_the_normal_config():
    pristine_scenarios = load_config("scenarios")
    pristine_depot = load_config("depot")
    base, scenario_cfg, depot_cfg = resolve_scenario_type("head_on_narrow")
    assert base == "head_on"
    assert depot_cfg["aisles"]["width"] == 3
    # The files on disk, and a freshly loaded copy, are untouched.
    assert load_config("scenarios") == pristine_scenarios
    assert load_config("depot") == pristine_depot
    assert pristine_depot["aisles"]["width"] == 4


def test_an_unknown_hard_type_is_rejected():
    with pytest.raises(ValueError):
        resolve_scenario_type("congested_impossible")
    with pytest.raises(ValueError):
        generate_scenario("congested_impossible", 0)


def test_the_type_lists_are_consistent():
    assert ALL_SCENARIO_TYPES == SCENARIO_TYPES + HARD_SCENARIO_TYPES
    assert not set(SCENARIO_TYPES) & set(HARD_SCENARIO_TYPES)
    assert ALL_PARKING_TYPES == PARKING_TYPES + HARD_PARKING_TYPES
    assert not set(PARKING_TYPES) & set(HARD_PARKING_TYPES)


# ------------------------------------------------- hard space-time scenarios


def test_congested_hard_really_carries_twelve_to_sixteen_agents():
    low, high = load_config("scenarios")["hard"]["congested_hard"]["scenarios"]["congested"][
        "agent_count"
    ]
    assert (low, high) == (12, 16)
    normal_high = load_config("scenarios")["congested"]["agent_count"][1]
    for seed in HARD_SEEDS:
        scenario = generate_scenario("congested_hard", seed)
        assert scenario.base == "congested"
        assert low <= len(scenario.agents) <= high
        assert len(scenario.agents) > normal_high


def test_head_on_narrow_really_has_narrow_aisles():
    for seed in HARD_SEEDS:
        narrow = generate_scenario("head_on_narrow", seed)
        assert narrow.base == "head_on"
        assert _aisle_band_heights(narrow.grid) == {3}
    for seed in HARD_SEEDS:
        assert _aisle_band_heights(generate_scenario("head_on", seed).grid) == {4}


def _aisle_band_heights(grid) -> set[int]:
    """Heights of the horizontal aisle bands, read back off the map."""
    # A horizontal aisle spans the full interior width; cross aisles do not.
    full_rows = [
        y for y in range(grid.height)
        if (grid.cells[y, 1:-1] == CellType.AISLE).all()
    ]
    heights: set[int] = set()
    run = 1
    for previous, current in zip(full_rows, full_rows[1:]):
        if current == previous + 1:
            run += 1
        else:
            heights.add(run)
            run = 1
    if full_rows:
        heights.add(run)
    return heights


def test_an_agent_and_its_margin_fill_a_narrow_aisle():
    """In a 3-cell aisle the ego cannot squeeze past an agent; that is the point."""
    scenario = generate_scenario("head_on_narrow", 0)
    agent = scenario.agents[0]
    blocked_somewhere = False
    for t in range(scenario.time_limit + 1):
        margin = agent.occupied_at(t, 1)
        rows = {y for _, y in margin}
        if len(rows) >= 4:
            blocked_somewhere = True
            break
    assert blocked_somewhere


def test_hard_scenarios_are_deterministic_and_solvable_in_principle():
    for name in HARD_SCENARIO_TYPES:
        for seed in HARD_SEEDS:
            first = generate_scenario(name, seed)
            second = generate_scenario(name, seed)
            assert first.start == second.start and first.goal == second.goal
            assert len(first.agents) == len(second.agents)
            resting = first.resting_obstacle_grid()
            assert astar(resting.cost_map(), first.start, first.goal,
                         drivable=resting.drivable, collect_expanded=False).success


# ---------------------------------------------------------- planning budgets


def test_the_normal_tier_has_no_planning_budget_and_the_hard_tier_does():
    assert tier_planner_config("normal") is None
    hard = tier_planner_config("hard")
    budget = float(load_config("eval")["hard_battery"]["planning_time_limit_ms"])
    assert hard["spacetime"]["time_limit_ms"] == budget
    assert hard["baseline"]["time_limit_ms"] == budget
    assert load_config("spacetime")["spacetime"]["time_limit_ms"] is None


def test_space_time_search_honours_a_wall_clock_budget():
    scenario = generate_scenario("congested_hard", 0)
    config = tier_planner_config("hard")
    config["spacetime"]["time_limit_ms"] = 0.0   # expire immediately
    grid = scenario.grid
    result = st.plan(grid.cost_map(), grid.drivable, scenario.start, scenario.goal,
                     scenario.agents, min_cell_cost=grid.min_cell_cost(), config=config)
    assert result.success is False
    assert result.reason == "time limit reached"


def test_grid_search_honours_a_wall_clock_budget():
    scenario = generate_scenario("empty", 0)
    grid = scenario.grid
    result = astar(grid.cost_map(), scenario.start, scenario.goal, drivable=grid.drivable,
                   time_limit_ms=0.0, collect_expanded=False)
    assert result.success is False
    assert result.reason == "time limit reached"


def test_the_tier_csv_names_differ():
    assert tier_csv("normal") != tier_csv("hard") == HARD_BATTERY_CSV
    assert parking_tier_csv("normal") != parking_tier_csv("hard") == HARD_PARKING_CSV
    for tier in TIERS:
        assert tier_settings(tier)["episodes_per_type"] > 0


# ------------------------------------------------ minimum-clearance geometry


@pytest.fixture(scope="module")
def car() -> CarModel:
    return CarModel.from_config()


def test_the_minimum_parallel_gap_is_where_the_goal_pose_becomes_free(car):
    """The derived minimum must match what the collision model actually accepts."""
    hybrid = load_config("hybrid")
    minimum = minimum_parallel_gap(car, hybrid)
    resolution = float(hybrid["collision"]["field_resolution"])

    def centred_car_is_free(gap: float) -> bool:
        rectangles = [(0.0, 0.0, 10.0, 6.0), (10.0 + gap, 0.0, 24.0 + gap, 6.0)]
        field = DistanceField.from_rectangles(rectangles, 26.0 + gap, 6.0, resolution)
        checker = CarCollisionChecker(field, car, hybrid)
        pose = (10.0 + (gap - car.length) / 2.0 + car.rear_overhang, 3.0, 0.0)
        return checker.is_free(pose)

    assert centred_car_is_free(minimum + 0.10)
    assert not centred_car_is_free(minimum - 0.10)


def test_the_minimum_gap_exceeds_the_car_length_by_the_disc_overhang(car):
    hybrid = load_config("hybrid")
    required = clearance_requirement(car, hybrid)
    overhang = required - car.length / (2.0 * car.n_discs)
    assert minimum_parallel_gap(car, hybrid) == pytest.approx(car.length + 2.0 * overhang)
    assert minimum_spot_width(car, hybrid) == pytest.approx(required + car.width / 2.0)
    assert required > car.disc_radius  # the field's conservatism is included


def test_hard_parking_gaps_sit_between_0_3_and_0_6_above_the_minimum():
    for name, low, high in (("parallel_minimal", 0.3, 0.6), ("perpendicular_minimal", 0.3, 0.6)):
        for seed in range(6):
            scenario = generate_parking_scenario(name, seed)
            slack = scenario.metrics["clearance_above_minimum"]
            assert low - 1e-9 <= slack <= high + 1e-9, f"{name} seed {seed}: slack {slack}"


def test_hard_parking_goal_poses_are_still_collision_free():
    for name in HARD_PARKING_TYPES:
        for seed in range(6):
            scenario = generate_parking_scenario(name, seed)
            checker = scenario.checker()
            assert checker.is_free(scenario.goal), f"{name} seed {seed}"
            assert checker.is_free(scenario.start), f"{name} seed {seed}"


def test_resolving_a_hard_parking_type_leaves_the_normal_config_alone(car):
    pristine = load_config("parking")
    base, merged = resolve_parking_type("parallel_minimal", car, load_config("parking"),
                                        load_config("hybrid"))
    assert base == "parallel"
    assert merged["parallel"]["gap"] != pristine["parallel"]["gap"]
    assert load_config("parking") == pristine


def test_the_hard_parking_tier_caps_expansions():
    cap = int(load_config("eval")["hard_battery"]["parking_max_expansions"])
    assert tier_hybrid_config("hard")["limits"]["max_expansions"] == cap
    assert tier_hybrid_config("normal")["limits"]["max_expansions"] > cap
    assert load_config("hybrid")["limits"]["max_expansions"] > cap


# ----------------------------------------------------------- the hard runs


def test_a_small_hard_space_time_battery_completes_and_is_labelled():
    frame = run_battery(episodes_per_type=1, verbose=False, tier="hard")
    assert len(frame) == len(HARD_SCENARIO_TYPES) * 1 * 2
    assert set(frame["tier"]) == {"hard"}
    assert set(frame["scenario"]) == set(HARD_SCENARIO_TYPES)
    assert set(frame["base_scenario"]) <= set(SCENARIO_TYPES)
    # A budgeted planner may fail, but it may never claim a colliding trajectory.
    timed = frame[frame["planner"] == "spacetime_astar"]
    assert not (timed["success"] & timed["collision"]).any()


def test_a_small_hard_parking_battery_completes_and_is_labelled():
    frame = run_parking_battery(episodes_per_type=1, verbose=False, tier="hard")
    assert len(frame) == len(HARD_PARKING_TYPES) * 1
    assert set(frame["tier"]) == {"hard"}
    assert set(frame["parking_type"]) == set(HARD_PARKING_TYPES)
    assert frame["verified_collision_free"][frame["success"]].all()
    assert (frame["nodes_expanded"] <= int(
        load_config("eval")["hard_battery"]["parking_max_expansions"]
    )).all()


# ------------------------------------------------------ report ingredients


def test_the_grid_benchmark_agrees_with_the_step_one_guarantees():
    frame = run_grid_benchmark(pairs=12)
    summary = summarise_grid(frame)
    by_name = {row["algorithm"]: row for _, row in summary.iterrows()}
    assert by_name["A*"]["mean_cost_ratio"] == pytest.approx(1.0)
    assert by_name["Dijkstra"]["mean_cost_ratio"] == pytest.approx(1.0)
    weighted = next(name for name in by_name if name.startswith("Weighted"))
    weight = float(load_config("grid")["search"]["weight"])
    assert by_name[weighted]["max_cost_ratio"] <= weight + 1e-9
    assert by_name["A*"]["mean_nodes_expanded"] <= by_name["Dijkstra"]["mean_nodes_expanded"]


def test_markdown_tables_render_missing_values_as_a_dash():
    import pandas as pd

    frame = pd.DataFrame([{"a": 1.5, "b": float("nan")}, {"a": 2.0, "b": 3.0}])
    text = markdown_table(frame, [("a", "A", ".1f"), ("b", "B", ".1f")])
    lines = text.splitlines()
    assert lines[0] == "| A | B |"
    assert "—" in lines[2]
    assert lines[3] == "| 2.0 | 3.0 |"
    assert format_value(None) == "—"
