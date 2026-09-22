"""Step 4 checks: the independent collision checker, the runner and the battery."""

from __future__ import annotations

import numpy as np
import pytest

from depot_planner.eval.battery import failures, run_battery, summarise, write_battery
from depot_planner.sim.collision import check_trajectory, first_collision_step
from depot_planner.sim.runner import run_episode, verify_episode
from depot_planner.spacetime.planners import make_planner
from depot_planner.viz.animate import animate_episode, gif_size_mb
from depot_planner.world.agents import Agent, Footprint
from depot_planner.world.grid import CellType, Grid
from depot_planner.world.scenarios import SCENARIO_TYPES, generate_scenario

BATTERY_SEEDS = 3


def _open_grid(width: int = 8, height: int = 6) -> Grid:
    cells = np.full((height, width), CellType.AISLE, dtype=np.uint8)
    cells[0, :] = cells[-1, :] = CellType.OBSTACLE
    cells[:, 0] = cells[:, -1] = CellType.OBSTACLE
    return Grid(cells)


# ------------------------------------------------------ the independent checker


def test_a_clean_trajectory_passes():
    grid = _open_grid()
    report = check_trajectory(grid, [], [(1, 1), (2, 1), (3, 2)])
    assert report.ok and report.describe() == "no collision"
    assert first_collision_step(grid, [], [(1, 1), (2, 1)]) is None


def test_driving_into_an_agent_is_an_overlap():
    grid = _open_grid()
    agent = Agent(7, np.array([[3, 1]] * 5), Footprint(1, 1))
    report = check_trajectory(grid, [agent], [(1, 1), (2, 1), (3, 1)])
    assert not report.ok and report.kind == "overlap"
    assert report.step == 2 and report.agent_id == 7


def test_trading_places_with_an_agent_is_a_swap():
    grid = _open_grid()
    agent = Agent(1, np.array([[2, 1], [1, 1]]), Footprint(1, 1))
    report = check_trajectory(grid, [agent], [(1, 1), (2, 1)])
    assert not report.ok and report.kind == "swap"


def test_driving_into_a_wall_is_reported():
    grid = _open_grid()
    report = check_trajectory(grid, [], [(1, 1), (1, 0)])
    assert not report.ok and report.kind == "obstacle"


def test_leaving_the_map_is_reported():
    grid = _open_grid()
    report = check_trajectory(grid, [], [(1, 1), (-1, 1)])
    assert not report.ok and report.kind == "out_of_bounds"


def test_teleporting_more_than_one_cell_is_reported():
    grid = _open_grid()
    report = check_trajectory(grid, [], [(1, 1), (4, 1)])
    assert not report.ok and report.kind == "jump"


def test_cutting_an_obstacle_corner_is_reported():
    cells = np.full((3, 3), CellType.AISLE, dtype=np.uint8)
    cells[0, 1] = CellType.OBSTACLE
    cells[1, 0] = CellType.OBSTACLE
    report = check_trajectory(Grid(cells), [], [(1, 1), (0, 0)])
    assert not report.ok and report.kind == "corner_cut"


def test_the_checker_honours_explicit_timestamps():
    grid = _open_grid()
    agent = Agent(0, np.array([[9, 9], [9, 9], [9, 9], [2, 1]]), Footprint(1, 1))
    assert check_trajectory(grid, [agent], [(1, 1), (2, 1)], times=[0, 1]).ok
    assert not check_trajectory(grid, [agent], [(1, 1), (2, 1)], times=[2, 3]).ok


def test_mismatched_cells_and_times_are_rejected():
    with pytest.raises(ValueError):
        check_trajectory(_open_grid(), [], [(1, 1), (2, 1)], times=[0])


# ------------------------------------------------------------------ the runner


@pytest.fixture(scope="module")
def episodes():
    out = {}
    for name in SCENARIO_TYPES:
        for planner_name in ("spacetime_astar", "baseline_replan"):
            for seed in range(BATTERY_SEEDS):
                scenario = generate_scenario(name, seed)
                episode = run_episode(scenario, make_planner(planner_name))
                out[(name, planner_name, seed)] = (scenario, episode)
    return out


def test_every_episode_terminates_on_goal_collision_or_timeout(episodes):
    for (name, planner_name, seed), (_, episode) in episodes.items():
        assert sum([episode.success, episode.collision, episode.timeout]) == 1
        assert episode.steps == len(episode.cells) - 1
        assert episode.times == list(range(len(episode.cells)))


def test_episode_metrics_are_internally_consistent(episodes):
    for (_, _, _), (scenario, episode) in episodes.items():
        expected_waits = sum(1 for a, b in zip(episode.cells, episode.cells[1:]) if a == b)
        assert episode.wait_steps == expected_waits
        expected_length = sum(
            np.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(episode.cells, episode.cells[1:])
        ) * scenario.grid.resolution
        assert episode.path_length_m == pytest.approx(expected_length)
        assert episode.replans >= 1
        assert episode.max_planning_ms >= episode.mean_planning_ms - 1e-9
        if episode.success:
            assert episode.cells[-1] == scenario.goal
            assert episode.time_to_goal_s == pytest.approx(episode.steps * scenario.dt)


def test_successful_episodes_have_zero_collisions_by_the_independent_checker(episodes):
    for (name, planner_name, seed), (scenario, episode) in episodes.items():
        if not episode.success:
            continue
        report = verify_episode(scenario, episode)
        assert report.ok, f"{planner_name} on {name} seed {seed}: {report.describe()}"


def test_space_time_astar_never_collides_on_the_test_slice(episodes):
    for (name, planner_name, seed), (scenario, episode) in episodes.items():
        if planner_name != "spacetime_astar":
            continue
        assert not episode.collision, f"{name} seed {seed}: {episode.collision_detail}"
        assert verify_episode(scenario, episode).ok


def test_the_two_planners_use_their_own_replan_cadence(episodes):
    scenario, timed = episodes[("empty", "spacetime_astar", 0)]
    _, baseline = episodes[("empty", "baseline_replan", 0)]
    assert baseline.replans == baseline.steps
    assert timed.replans < timed.steps


def test_running_the_same_episode_twice_gives_the_same_result():
    scenario = generate_scenario("congested", 4)
    first = run_episode(scenario, make_planner("spacetime_astar"))
    second = run_episode(scenario, make_planner("spacetime_astar"))
    assert first.cells == second.cells
    assert first.steps == second.steps and first.nodes_expanded == second.nodes_expanded


def test_a_planner_that_cannot_move_times_out_cleanly():
    """A blocker parked on the goal forever: the ego must not claim success."""
    scenario = generate_scenario("empty", 0)
    scenario.time_limit = 24  # a hopeless replan is expensive; keep the episode short
    blocker = Agent(0, np.array([[scenario.goal[0], scenario.goal[1]]] * 400), Footprint(1, 1))
    scenario.agents = [blocker]
    episode = run_episode(scenario, make_planner("spacetime_astar"))
    assert not episode.success
    assert episode.timeout or episode.collision
    assert episode.plan_failures > 0 or episode.collision


# ----------------------------------------------------------------- the battery


@pytest.fixture(scope="module")
def battery():
    return run_battery(episodes_per_type=BATTERY_SEEDS, verbose=False)


def test_the_battery_completes_with_one_row_per_episode(battery):
    assert len(battery) == len(SCENARIO_TYPES) * BATTERY_SEEDS * 2
    assert set(battery["scenario"]) == set(SCENARIO_TYPES)
    assert set(battery["planner"]) == {"spacetime_astar", "baseline_replan"}
    for column in ("success", "collision", "steps", "wait_steps", "nodes_expanded",
                   "mean_planning_ms", "max_planning_ms", "path_length_m"):
        assert column in battery.columns


def test_the_battery_runs_both_planners_on_the_same_seeds(battery):
    counts = battery.groupby(["scenario", "seed"])["planner"].nunique()
    assert (counts == 2).all()


def test_space_time_episodes_marked_successful_have_no_collisions(battery):
    timed = battery[battery["planner"] == "spacetime_astar"]
    assert not (timed["success"] & timed["collision"]).any()
    assert timed["collision"].sum() == 0


def test_summary_and_failure_views_are_well_formed(battery):
    summary = summarise(battery)
    assert len(summary) == len(SCENARIO_TYPES) * 2
    assert (summary["episodes"] == BATTERY_SEEDS).all()
    assert (summary["success_pct"] >= 0).all() and (summary["success_pct"] <= 100).all()
    failed = failures(battery)
    assert len(failed) == (~battery["success"]).sum()


def test_the_battery_csv_round_trips(battery, tmp_path):
    import pandas as pd

    out = write_battery(battery, tmp_path / "battery.csv")
    assert out.is_file()
    assert len(pd.read_csv(out)) == len(battery)


# --------------------------------------------------------------- the animation


def test_an_episode_animates_to_a_small_gif(tmp_path):
    scenario = generate_scenario("crossing", 1)
    episode = run_episode(scenario, make_planner("spacetime_astar"))
    out = animate_episode(scenario, episode, tmp_path / "episode.gif")
    assert out.is_file()
    assert gif_size_mb(out) < 5.0
    import imageio.v2 as imageio

    frames = imageio.mimread(out)
    assert len(frames) > 3
