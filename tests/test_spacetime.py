"""Step 3 checks: space-time A*, waiting, and the replanning baseline."""

from __future__ import annotations

import numpy as np
import pytest

from depot_planner.config import load_config
from depot_planner.grid_astar.search import astar
from depot_planner.sim.collision import check_trajectory
from depot_planner.spacetime import search as st
from depot_planner.spacetime.planners import BaselineReplanPlanner, SpaceTimePlanner, make_planner
from depot_planner.world.agents import Agent, Footprint
from depot_planner.world.grid import CellType, Grid
from depot_planner.world.scenarios import SCENARIO_TYPES, generate_scenario

SEEDS = range(6)


def _plan(scenario, config=None, start=None, t=0, **kwargs):
    grid = scenario.grid
    return st.plan(
        grid.cost_map(), grid.drivable, start or scenario.start, scenario.goal, scenario.agents,
        start_time=t, min_cell_cost=grid.min_cell_cost(), config=config, **kwargs,
    )


# ------------------------------------------------------- the hand-built corridor


def _corridor():
    """A 1-cell corridor crossed by a 1-cell side passage at x = 6.

    The ego must cross (6, 2); the agent walks down the passage through it. The
    corridor is too narrow to pass, so the only legal plan is to wait.
    """
    cells = np.full((5, 13), CellType.OBSTACLE, dtype=np.uint8)
    cells[2, :] = CellType.AISLE      # horizontal corridor
    cells[:, 6] = CellType.AISLE      # vertical passage
    grid = Grid(cells)
    # The agent sits at the top of the passage, then walks down across the corridor
    # at exactly the steps a naive planner would drive through it.
    timeline = [(6, 0)] * 5 + [(6, 1), (6, 2), (6, 3), (6, 4)] + [(6, 4)] * 40
    agent = Agent(0, np.array(timeline), Footprint(1, 1), kind="crossing")
    return grid, [agent]


def test_space_time_astar_waits_for_the_crossing_agent_and_then_proceeds():
    grid, agents = _corridor()
    start, goal = (0, 2), (12, 2)
    result = st.plan(grid.cost_map(), grid.drivable, start, goal, agents,
                     min_cell_cost=grid.min_cell_cost())
    assert result.success, result.reason
    assert result.cells[0] == start and result.cells[-1] == goal
    assert result.wait_steps >= 1, "the ego should have waited for the agent to clear"
    # It waits on the near side, then crosses after the agent has gone by.
    crossing_step = result.cells.index((6, 2))
    assert crossing_step > 6
    assert check_trajectory(grid, agents, result.cells, result.times).ok


def test_the_corridor_plan_has_zero_collisions_by_the_independent_checker():
    grid, agents = _corridor()
    result = st.plan(grid.cost_map(), grid.drivable, (0, 2), (12, 2), agents,
                     min_cell_cost=grid.min_cell_cost())
    report = check_trajectory(grid, agents, result.cells, result.times)
    assert report.ok, report.describe()


def test_without_the_agent_the_corridor_needs_no_waiting():
    grid, _ = _corridor()
    result = st.plan(grid.cost_map(), grid.drivable, (0, 2), (12, 2), [],
                     min_cell_cost=grid.min_cell_cost())
    assert result.success
    assert result.wait_steps == 0
    assert len(result.cells) == 13


# ------------------------------------------------------ agreement with step 1


def test_with_no_agents_the_cost_equals_step_one_astar():
    """With the time term switched off the two searches optimise the same thing."""
    config = load_config("spacetime")
    config["spacetime"]["time_cost"] = 0.0
    for seed in SEEDS:
        scenario = generate_scenario("empty", seed)
        grid = scenario.grid
        reference = astar(grid.cost_map(), scenario.start, scenario.goal,
                          drivable=grid.drivable, min_cell_cost=grid.min_cell_cost())
        timed = _plan(scenario, config)
        assert timed.success
        assert timed.cost == pytest.approx(reference.cost, rel=1e-9)
        assert timed.movement_cost == pytest.approx(reference.cost, rel=1e-9)
        assert timed.wait_steps == 0


def test_with_the_default_time_cost_the_ego_still_never_waits_in_an_empty_depot():
    for seed in SEEDS:
        scenario = generate_scenario("empty", seed)
        grid = scenario.grid
        reference = astar(grid.cost_map(), scenario.start, scenario.goal,
                          drivable=grid.drivable, min_cell_cost=grid.min_cell_cost())
        timed = _plan(scenario)
        assert timed.success and timed.wait_steps == 0
        # A per-step time cost is a different objective, so the movement cost may
        # rise a little, but it can never fall below the step-1 optimum.
        assert timed.movement_cost >= reference.cost - 1e-9
        assert timed.movement_cost <= reference.cost * 1.02 + 1e-9


def test_the_two_heuristics_return_the_same_optimal_cost():
    octile_cfg = load_config("spacetime")
    octile_cfg["spacetime"]["heuristic"] = "octile"
    for name in ("empty", "crossing", "blocked_then_clears"):
        for seed in range(3):
            scenario = generate_scenario(name, seed)
            a = _plan(scenario)
            b = _plan(scenario, octile_cfg)
            assert a.success and b.success
            assert a.cost == pytest.approx(b.cost, rel=1e-9)
            assert a.nodes_expanded <= b.nodes_expanded


def test_an_unknown_heuristic_is_rejected():
    config = load_config("spacetime")
    config["spacetime"]["heuristic"] = "vibes"
    with pytest.raises(ValueError):
        _plan(generate_scenario("empty", 0), config)


# --------------------------------------------------------------- cost accounting


def test_reported_cost_splits_into_movement_plus_time():
    config = load_config("spacetime")
    time_cost = float(config["spacetime"]["time_cost"])
    for name in ("crossing", "congested"):
        scenario = generate_scenario(name, 1)
        result = _plan(scenario)
        assert result.success
        steps = len(result.cells) - 1
        assert result.cost == pytest.approx(result.movement_cost + steps * time_cost, rel=1e-9)


def test_waiting_costs_something():
    grid, agents = _corridor()
    result = st.plan(grid.cost_map(), grid.drivable, (0, 2), (12, 2), agents,
                     min_cell_cost=grid.min_cell_cost())
    no_traffic = st.plan(grid.cost_map(), grid.drivable, (0, 2), (12, 2), [],
                         min_cell_cost=grid.min_cell_cost())
    assert result.cost > no_traffic.cost


# -------------------------------------------------------------- collision rules


def test_plans_on_every_scenario_type_pass_the_independent_checker():
    for name in SCENARIO_TYPES:
        for seed in SEEDS:
            scenario = generate_scenario(name, seed)
            result = _plan(scenario)
            assert result.success, f"{name} seed {seed}: {result.reason}"
            report = check_trajectory(scenario.grid, scenario.agents, result.cells, result.times)
            assert report.ok, f"{name} seed {seed}: {report.describe()}"


def test_the_plan_keeps_the_configured_safety_margin():
    inflate = int(load_config("spacetime")["spacetime"]["inflate"])
    for name in ("crossing", "head_on", "congested"):
        for seed in SEEDS:
            scenario = generate_scenario(name, seed)
            result = _plan(scenario)
            for cell, t in zip(result.cells, result.times):
                for agent in scenario.agents:
                    # The margin may be entered only if the ego was already inside it.
                    assert cell not in agent.cells_at(t)
            entries = sum(
                1 for cell, t in zip(result.cells, result.times)
                if cell in scenario.occupied_at(t, inflate)
            )
            assert entries == 0, f"{name} seed {seed}: plan enters the safety margin"


def test_an_agent_parked_on_the_goal_forever_is_reported_as_no_path():
    grid, _ = _corridor()
    blocker = Agent(0, np.array([[12, 2]] * 60), Footprint(1, 1))
    result = st.plan(grid.cost_map(), grid.drivable, (0, 2), (12, 2), [blocker],
                     min_cell_cost=grid.min_cell_cost())
    assert result.success is False
    assert result.cells is None
    assert result.reason in {"no timed path to the goal", "node limit reached"}


def test_the_time_horizon_cap_is_respected():
    grid, agents = _corridor()
    result = st.plan(grid.cost_map(), grid.drivable, (0, 2), (12, 2), agents,
                     min_cell_cost=grid.min_cell_cost(), max_time=4)
    assert result.success is False


def test_the_node_cap_is_respected():
    config = load_config("spacetime")
    config["spacetime"]["max_nodes"] = 20
    scenario = generate_scenario("congested", 2)
    result = _plan(scenario, config)
    assert result.success is False
    assert result.reason == "node limit reached"


def test_a_goal_on_an_obstacle_is_rejected():
    grid, agents = _corridor()
    result = st.plan(grid.cost_map(), grid.drivable, (0, 2), (0, 0), agents,
                     min_cell_cost=grid.min_cell_cost())
    assert result.success is False
    assert "not drivable" in result.reason


# ----------------------------------------------------------------- the planners


def test_both_planners_are_constructible_by_name():
    assert isinstance(make_planner("spacetime_astar"), SpaceTimePlanner)
    assert isinstance(make_planner("baseline_replan"), BaselineReplanPlanner)
    with pytest.raises(ValueError):
        make_planner("wishful_thinking")


def test_the_baseline_replans_every_step_and_space_time_does_not():
    assert BaselineReplanPlanner.replan_every == 1
    assert SpaceTimePlanner.replan_every is None


def test_the_baseline_ignores_the_future_and_can_plan_through_moving_traffic():
    """The baseline's open-loop plan collides where space-time A* does not."""
    grid, agents = _corridor()

    class Stub:
        pass

    scenario = Stub()
    scenario.grid, scenario.agents, scenario.goal = grid, agents, (12, 2)
    scenario.time_limit = 60
    baseline = make_planner("baseline_replan").plan(scenario, (0, 2), 0)
    timed = make_planner("spacetime_astar").plan(scenario, (0, 2), 0)
    assert baseline.success and timed.success
    assert not check_trajectory(grid, agents, baseline.cells, baseline.times).ok
    assert check_trajectory(grid, agents, timed.cells, timed.times).ok


def test_the_baseline_holds_position_when_it_is_boxed_in():
    grid, _ = _corridor()
    wall = Agent(0, np.array([[3, 2]] * 60), Footprint(1, 1))

    class Stub:
        pass

    scenario = Stub()
    scenario.grid, scenario.agents, scenario.goal = grid, [wall], (12, 2)
    scenario.time_limit = 60
    result = make_planner("baseline_replan").plan(scenario, (0, 2), 0)
    assert result.success
    assert result.cells == [(0, 2), (0, 2)]
    assert result.reason == "blocked, holding position"


def test_planning_from_a_later_start_time_still_works():
    for name in ("crossing", "congested"):
        scenario = generate_scenario(name, 0)
        result = _plan(scenario, t=12)
        assert result.success
        assert result.times[0] == 12
        assert check_trajectory(scenario.grid, scenario.agents, result.cells, result.times).ok
