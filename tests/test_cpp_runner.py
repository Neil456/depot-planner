"""C++ port, step 7: the closed-loop runner.

The Python loop stays the default and the reference: it hands every executed
step to the independent collision checker as the episode runs. The C++ loop is
opt-in via ``engine="cpp"`` and is held to producing the same episode, metric
for metric. The full sweep over every battery seed is
``scripts/check_equivalence.py --section runner``.
"""

from __future__ import annotations

import math

import pytest

from depot_planner import core
from depot_planner.eval.battery import run_battery
from depot_planner.sim.runner import DEFAULT_ENGINE, run_episode, verify_episode
from depot_planner.spacetime.planners import PLANNERS, Planner, make_planner
from depot_planner.world.scenarios import SCENARIO_TYPES, generate_scenario

needs_cpp = pytest.mark.skipif(not core.available(), reason="the C++ core is not built")

#: Every field of an EpisodeResult the two loops must agree on.
EPISODE_FIELDS = (
    "success", "collision", "timeout", "steps", "cells", "times", "wait_steps",
    "nodes_expanded", "replans", "plan_failures", "reason", "collision_step",
    "collision_detail", "plan_failure_reasons", "plans", "path_length_m",
)

SEEDS = (1000, 1001, 1002)


def test_the_python_loop_is_the_default():
    """The audited path stays the default: the checker runs on every step."""
    assert DEFAULT_ENGINE == "python"


@needs_cpp
@pytest.mark.parametrize("scenario_type", SCENARIO_TYPES)
@pytest.mark.parametrize("planner_name", sorted(PLANNERS))
def test_both_loops_produce_the_same_episode(scenario_type, planner_name):
    for seed in SEEDS:
        scenario = generate_scenario(scenario_type, seed)
        episodes = {
            engine: run_episode(scenario, make_planner(planner_name), engine=engine)
            for engine in ("python", "cpp")
        }
        python_episode, cpp_episode = episodes["python"], episodes["cpp"]
        for name in EPISODE_FIELDS:
            assert getattr(python_episode, name) == getattr(cpp_episode, name), (
                f"{scenario_type} seed {seed} {planner_name}: {name} differs"
            )
        if python_episode.success:
            assert cpp_episode.time_to_goal_s == python_episode.time_to_goal_s
        else:
            assert math.isnan(cpp_episode.time_to_goal_s)


@needs_cpp
def test_the_independent_checker_still_audits_a_cpp_episode():
    """The Python checker is the auditor of record whichever loop ran."""
    for scenario_type in ("crossing", "congested"):
        for seed in SEEDS:
            scenario = generate_scenario(scenario_type, seed)
            episode = run_episode(scenario, make_planner("spacetime_astar"), engine="cpp")
            report = verify_episode(scenario, episode)
            assert report.ok, f"{scenario_type} seed {seed}: {report.describe()}"


@needs_cpp
def test_a_collision_is_reported_the_same_way_by_both_loops():
    """`crossing` seed 1001 is a seed the baseline is known to fail on."""
    scenario = generate_scenario("crossing", 1001)
    episodes = {
        engine: run_episode(scenario, make_planner("baseline_replan"), engine=engine)
        for engine in ("python", "cpp")
    }
    assert episodes["python"].collision
    assert episodes["cpp"].collision
    assert episodes["python"].collision_step == episodes["cpp"].collision_step
    assert episodes["python"].collision_detail == episodes["cpp"].collision_detail
    assert episodes["python"].reason == episodes["cpp"].reason


@needs_cpp
def test_the_cpp_loop_refuses_a_planner_it_does_not_implement():
    class Stubborn(Planner):
        name = "stubborn"

        def plan(self, scenario, cell, t):  # pragma: no cover - never called
            raise AssertionError("should not be reached")

    scenario = generate_scenario("empty", SEEDS[0])
    with pytest.raises(ValueError, match="does not implement"):
        run_episode(scenario, Stubborn(), engine="cpp")


@needs_cpp
def test_the_cpp_loop_refuses_a_planner_pinned_to_python():
    scenario = generate_scenario("empty", SEEDS[0])
    planner = make_planner("spacetime_astar", None, "python")
    with pytest.raises(ValueError, match="pinned to the Python backend"):
        run_episode(scenario, planner, engine="cpp")


@needs_cpp
def test_a_battery_slice_is_identical_under_either_loop():
    frames = {
        engine: run_battery(scenario_types=["crossing", "congested"], episodes_per_type=4,
                            verbose=False, engine=engine)
        for engine in ("python", "cpp")
    }
    columns = [c for c in frames["python"].columns
               if c not in {"mean_planning_ms", "max_planning_ms"}]
    assert frames["python"][columns].equals(frames["cpp"][columns])
