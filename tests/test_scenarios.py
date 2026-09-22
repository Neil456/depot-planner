"""Step 2 checks: agents, timelines and the five scenario generators."""

from __future__ import annotations

import numpy as np
import pytest

from depot_planner.config import load_config
from depot_planner.grid_astar.search import astar
from depot_planner.world.agents import Agent, Footprint, agents_overlap, footprint_mask, timed_path
from depot_planner.world.grid import CellType
from depot_planner.world.scenarios import (
    AGENT_INFLATION,
    SCENARIO_TYPES,
    ScenarioGenerationError,
    generate_scenario,
)

INSTANCES = 20


@pytest.fixture(scope="module")
def instances() -> dict[str, list]:
    return {
        name: [generate_scenario(name, seed) for seed in range(INSTANCES)]
        for name in SCENARIO_TYPES
    }


# ---------------------------------------------------------------- the agents


def test_footprint_covers_the_expected_block():
    footprint = Footprint(2, 3)
    assert footprint.cells((4, 5)) == {(4, 5), (5, 5), (4, 6), (5, 6), (4, 7), (5, 7)}


def test_inflation_grows_the_body_by_one_chebyshev_ring():
    agent = Agent(0, np.array([[5, 5]]), Footprint(2, 2))
    assert len(agent.cells_at(0)) == 4
    assert len(agent.occupied_at(0, inflate=1)) == 16
    assert (4, 4) in agent.occupied_at(0, inflate=1)
    assert (4, 4) not in agent.cells_at(0)


def test_timeline_holds_start_then_walks_then_holds_end():
    rng = np.random.default_rng(0)
    timeline = timed_path([(1, 1), (2, 1), (3, 1)], total_steps=8, rng=rng, start_delay=2)
    assert timeline.shape == (9, 2)
    assert tuple(timeline[0]) == (1, 1) and tuple(timeline[1]) == (1, 1)
    assert tuple(timeline[2]) == (1, 1) and tuple(timeline[-1]) == (3, 1)


def test_agent_stays_put_beyond_the_end_of_its_timeline():
    agent = Agent(0, np.array([[1, 1], [2, 1]]), Footprint(1, 1))
    assert agent.anchor_at(50) == (2, 1)
    assert agent.anchor_at(-5) == (1, 1)


def test_footprint_mask_only_admits_anchors_whose_whole_body_fits(depot):
    footprint = Footprint(2, 2)
    mask = footprint_mask(depot, footprint)
    ys, xs = np.nonzero(mask)
    for x, y in zip(xs[::37], ys[::37]):
        for cx, cy in footprint.cells((int(x), int(y))):
            assert depot.is_drivable(cx, cy)
    aisle_mask = footprint_mask(depot, footprint, aisles_only=True)
    assert aisle_mask.sum() < mask.sum()
    ys, xs = np.nonzero(aisle_mask)
    for x, y in zip(xs[::37], ys[::37]):
        for cx, cy in footprint.cells((int(x), int(y))):
            assert depot.cells[cy, cx] == CellType.AISLE


def test_overlap_detection_catches_shared_cells_and_crossings():
    footprint = Footprint(1, 1)
    a = Agent(0, np.array([[0, 0], [1, 0]]), footprint)
    b = Agent(1, np.array([[1, 0], [0, 0]]), footprint)
    assert agents_overlap(a, b, horizon=1)
    c = Agent(2, np.array([[0, 5], [1, 5]]), footprint)
    assert not agents_overlap(a, c, horizon=1)


# ------------------------------------------------------------- the scenarios


def test_every_scenario_type_generates_twenty_valid_instances(instances):
    for name in SCENARIO_TYPES:
        assert len(instances[name]) == INSTANCES
        for scenario in instances[name]:
            assert scenario.name == name
            assert scenario.time_limit > 0
            assert scenario.dt == pytest.approx(load_config("scenarios")["dt"])
            assert scenario.grid.is_drivable(*scenario.start)
            assert scenario.grid.is_drivable(*scenario.goal)
            assert scenario.start != scenario.goal


def test_scenario_generation_is_deterministic_under_seed():
    for name in SCENARIO_TYPES:
        first = generate_scenario(name, 5)
        second = generate_scenario(name, 5)
        assert np.array_equal(first.grid.cells, second.grid.cells)
        assert first.start == second.start and first.goal == second.goal
        assert first.time_limit == second.time_limit
        assert len(first.agents) == len(second.agents)
        for a, b in zip(first.agents, second.agents):
            assert np.array_equal(a.timeline, b.timeline)


def test_different_seeds_give_different_problems():
    a, b = generate_scenario("congested", 1), generate_scenario("congested", 2)
    assert (a.start, a.goal) != (b.start, b.goal) or not np.array_equal(a.grid.cells, b.grid.cells)


def test_agents_never_overlap_walls(instances):
    for name in SCENARIO_TYPES:
        for scenario in instances[name]:
            for agent in scenario.agents:
                for t in range(scenario.time_limit + 1):
                    for x, y in agent.cells_at(t):
                        assert scenario.grid.is_drivable(x, y), f"{name} seed {scenario.seed}"


def test_agents_never_overlap_each_other(instances):
    for scenario in instances["congested"]:
        for i, first in enumerate(scenario.agents):
            for second in scenario.agents[i + 1 :]:
                assert not agents_overlap(first, second, scenario.time_limit)


def test_agents_move_at_most_one_cell_per_step(instances):
    for name in SCENARIO_TYPES:
        for scenario in instances[name]:
            for agent in scenario.agents:
                steps = np.abs(np.diff(agent.timeline, axis=0))
                assert steps.max(initial=0) <= 1


def test_timelines_cover_the_whole_time_limit(instances):
    for name in SCENARIO_TYPES:
        for scenario in instances[name]:
            for agent in scenario.agents:
                assert agent.horizon == scenario.time_limit


def test_ego_start_is_clear_and_the_goal_is_reachable_once_agents_rest(instances):
    for name in SCENARIO_TYPES:
        for scenario in instances[name]:
            assert scenario.start not in scenario.occupied_at(0, AGENT_INFLATION)
            assert scenario.goal not in scenario.occupied_at(scenario.time_limit, AGENT_INFLATION)
            resting = scenario.resting_obstacle_grid()
            result = astar(resting.cost_map(), scenario.start, scenario.goal,
                           drivable=resting.drivable, collect_expanded=False)
            assert result.success, f"{name} seed {scenario.seed} unsolvable once agents rest"


def test_the_empty_scenario_really_has_no_agents(instances):
    for scenario in instances["empty"]:
        assert scenario.agents == []
        assert scenario.occupied_at(10) == set()


def test_congested_scenarios_carry_six_to_ten_agents(instances):
    low, high = load_config("scenarios")["congested"]["agent_count"]
    for scenario in instances["congested"]:
        assert low <= len(scenario.agents) <= high


@pytest.mark.parametrize("name", ["crossing", "head_on", "blocked_then_clears"])
def test_targeted_scenarios_actually_obstruct_the_free_space_route(instances, name):
    for scenario in instances[name]:
        blocked = any(
            cell in scenario.occupied_at(t, AGENT_INFLATION)
            for t, cell in enumerate(scenario.natural_route)
        )
        assert blocked, f"{name} seed {scenario.seed} never obstructs the ego"


def test_the_blocker_clears_the_aisle_before_the_time_limit(instances):
    for scenario in instances["blocked_then_clears"]:
        agent = scenario.agents[0]
        assert agent.anchor_at(0) != agent.anchor_at(scenario.time_limit)


def test_an_unknown_scenario_type_is_rejected():
    with pytest.raises(ValueError):
        generate_scenario("teleporting_forklift", 0)


def test_generation_failure_is_reported_as_a_clean_error():
    config = load_config("scenarios")
    config["scenario"]["min_ego_separation"] = 10_000.0
    config["scenario"]["max_generation_attempts"] = 3
    with pytest.raises(ScenarioGenerationError):
        generate_scenario("empty", 0, config=config)
