"""C++ port, step 3: space-time A* and the snapshot baseline.

Every check compares the compiled core against the pure-Python reference on the
same scenarios. The reference is never adjusted; a divergence is a bug in the
port. A wider sweep over every battery seed lives in
``scripts/check_equivalence.py``, which is what step 3's acceptance check ran.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from depot_planner import core
from depot_planner.eval.battery import battery_seeds, run_battery, tier_settings
from depot_planner.sim.runner import run_episode, verify_episode
from depot_planner.spacetime import search as st
from depot_planner.spacetime.planners import make_planner
from depot_planner.world.scenarios import SCENARIO_TYPES, generate_scenario

needs_cpp = pytest.mark.skipif(not core.available(), reason="the C++ core is not built")

#: The fields two plans must agree on, exactly.
PLAN_FIELDS = ("success", "cells", "times", "nodes_expanded", "wait_steps", "reason")

#: Columns the two backends are allowed to differ on.
TIMING_COLUMNS = {"mean_planning_ms", "max_planning_ms", "backend"}


def _assert_same_plan(a, b, what: str) -> None:
    for name in PLAN_FIELDS:
        assert getattr(a, name) == getattr(b, name), f"{what}: {name} differs"
    if math.isinf(a.cost):
        assert math.isinf(b.cost), f"{what}: cost differs"
    else:
        assert a.cost == pytest.approx(b.cost, abs=1e-12, rel=0.0), f"{what}: cost differs"


@pytest.fixture(scope="module")
def seeds() -> list[int]:
    cfg = tier_settings("normal")
    return battery_seeds(6, int(cfg["seed_offset"]))


# ------------------------------------------------------------- the search


@needs_cpp
@pytest.mark.parametrize("scenario_type", SCENARIO_TYPES)
def test_both_backends_return_the_same_timed_plan(scenario_type, seeds):
    for seed in seeds:
        scenario = generate_scenario(scenario_type, seed)
        grid = scenario.grid
        cost_map, drivable = grid.cost_map(), grid.drivable
        field = st.goal_cost_field(cost_map, drivable & np.isfinite(cost_map), scenario.goal)
        common = dict(start_time=0, min_cell_cost=grid.min_cell_cost(), collect_expanded=False,
                      heuristic_field=field, max_time=min(700, int(scenario.time_limit)))
        python_result = st.plan(cost_map, drivable, scenario.start, scenario.goal,
                                scenario.agents, backend="python", **common)
        cpp_result = st.plan(cost_map, drivable, scenario.start, scenario.goal,
                             scenario.agents, backend="cpp", **common)
        _assert_same_plan(python_result, cpp_result, f"{scenario_type} seed {seed}")
        assert python_result.movement_cost == pytest.approx(
            cpp_result.movement_cost, abs=1e-12, rel=0.0
        )


@needs_cpp
def test_the_octile_heuristic_also_agrees(seeds):
    """The non-default heuristic has its own branch in both implementations."""
    from depot_planner.config import deep_merge, load_config

    config = deep_merge(load_config("spacetime"), {"spacetime": {"heuristic": "octile"}})
    for seed in seeds[:3]:
        scenario = generate_scenario("crossing", seed)
        grid = scenario.grid
        common = dict(start_time=0, min_cell_cost=grid.min_cell_cost(), collect_expanded=False,
                      config=config, max_time=min(700, int(scenario.time_limit)))
        _assert_same_plan(
            st.plan(grid.cost_map(), grid.drivable, scenario.start, scenario.goal,
                    scenario.agents, backend="python", **common),
            st.plan(grid.cost_map(), grid.drivable, scenario.start, scenario.goal,
                    scenario.agents, backend="cpp", **common),
            f"octile crossing seed {seed}",
        )


@needs_cpp
def test_the_caps_are_reported_identically(seeds):
    from depot_planner.config import deep_merge, load_config

    scenario = generate_scenario("congested", seeds[0])
    grid = scenario.grid
    for override, expected in (
        ({"max_nodes": 40}, "node limit reached"),
        ({"time_limit_ms": 0.0}, "time limit reached"),
        ({"max_time_horizon": 3}, "no timed path to the goal"),
    ):
        config = deep_merge(load_config("spacetime"), {"spacetime": override})
        results = [
            st.plan(grid.cost_map(), grid.drivable, scenario.start, scenario.goal,
                    scenario.agents, start_time=0, min_cell_cost=grid.min_cell_cost(),
                    collect_expanded=False, config=config, backend=backend)
            for backend in ("python", "cpp")
        ]
        for result in results:
            assert result.success is False
            assert result.reason == expected
        if expected != "time limit reached":  # a wall clock is not reproducible
            assert results[0].nodes_expanded == results[1].nodes_expanded


@needs_cpp
def test_asking_for_expanded_cells_falls_back_to_the_reference(seeds):
    """`collect_expanded` is Python-only, so the default backend must not lose it."""
    scenario = generate_scenario("crossing", seeds[0])
    grid = scenario.grid
    result = st.plan(grid.cost_map(), grid.drivable, scenario.start, scenario.goal,
                     scenario.agents, collect_expanded=True, backend="auto",
                     min_cell_cost=grid.min_cell_cost())
    assert result.success
    assert result.expanded


# ----------------------------------------------------------- the planners


@needs_cpp
@pytest.mark.parametrize("planner_name", ("spacetime_astar", "baseline_replan"))
def test_the_planner_wrappers_agree_from_several_states(planner_name, seeds):
    for seed in seeds:
        scenario = generate_scenario("congested", seed)
        for t in (0, 7, 23):
            plans = [
                make_planner(planner_name, None, backend).plan(scenario, scenario.start, t)
                for backend in ("python", "cpp")
            ]
            _assert_same_plan(plans[0], plans[1], f"{planner_name} seed {seed} t={t}")


@needs_cpp
def test_the_baseline_blocks_the_same_cells_as_the_reference(seeds):
    """Planning from a cell inside an agent's margin is the interesting case."""
    scenario = generate_scenario("crossing", seeds[0])
    agent = scenario.agents[0]
    anchor = agent.anchor_at(0)
    beside = (int(anchor[0]) + agent.footprint.width, int(anchor[1]))
    if not scenario.grid.is_drivable(*beside):
        pytest.skip("the cell beside this agent is not drivable")
    plans = [
        make_planner("baseline_replan", None, backend).plan(scenario, beside, 0)
        for backend in ("python", "cpp")
    ]
    _assert_same_plan(plans[0], plans[1], "baseline beside an agent")


# ------------------------------------------------------- the closed loop


@needs_cpp
@pytest.mark.parametrize("scenario_type", SCENARIO_TYPES)
def test_closed_loop_episodes_are_identical(scenario_type, seeds):
    for seed in seeds[:3]:
        scenario = generate_scenario(scenario_type, seed)
        for planner_name in ("spacetime_astar", "baseline_replan"):
            episodes = {}
            for backend in ("python", "cpp"):
                planner = make_planner(planner_name, None, backend)
                episodes[backend] = run_episode(scenario, planner)
            python_episode, cpp_episode = episodes["python"], episodes["cpp"]
            assert verify_episode(scenario, cpp_episode).ok or cpp_episode.collision
            for name in ("success", "collision", "timeout", "steps", "cells", "times",
                         "wait_steps", "nodes_expanded", "replans", "plan_failures", "reason"):
                assert getattr(python_episode, name) == getattr(cpp_episode, name), (
                    f"{scenario_type} seed {seed} {planner_name}: {name} differs"
                )


@needs_cpp
def test_a_battery_slice_matches_on_every_non_timing_column():
    frames = {
        backend: run_battery(scenario_types=["crossing", "congested"], episodes_per_type=4,
                             verbose=False, backend=backend)
        for backend in ("python", "cpp")
    }
    python_frame, cpp_frame = frames["python"], frames["cpp"]
    columns = [c for c in python_frame.columns if c not in TIMING_COLUMNS]
    assert python_frame[columns].equals(cpp_frame[columns])
    assert (cpp_frame["backend"] == "cpp").all()
    assert (python_frame["backend"] == "python").all()
