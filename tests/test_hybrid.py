"""Step 5 checks: car model, Reeds-Shepp curves, collision model and hybrid A*."""

from __future__ import annotations

import math

import numpy as np
import pytest

from depot_planner.config import load_config
from depot_planner.eval.parking_battery import (
    parking_failures,
    run_parking_battery,
    summarise_parking,
    write_parking_battery,
)
from depot_planner.hybrid import reeds_shepp as rs
from depot_planner.hybrid.car import CarModel, angle_difference, wrap_angle
from depot_planner.hybrid.collision import CarCollisionChecker, DistanceField
from depot_planner.hybrid.search import (
    HybridResult,
    at_goal,
    heading_bin,
    plan,
    plan_for_scenario,
    state_key,
)
from depot_planner.sim.car_collision import car_corners, check_car_path, check_plan, densify
from depot_planner.viz.animate import animate_parking, gif_size_mb
from depot_planner.world.parking import (
    PARKING_TYPES,
    ParkingGenerationError,
    generate_parking_scenario,
)

SEEDS = range(3)


@pytest.fixture(scope="module")
def car() -> CarModel:
    return CarModel.from_config()


@pytest.fixture(scope="module")
def solved():
    out = {}
    for name in PARKING_TYPES:
        for seed in SEEDS:
            scenario = generate_parking_scenario(name, seed)
            out[(name, seed)] = (scenario, plan_for_scenario(scenario))
    return out


def _open_lot(width: float = 40.0, height: float = 30.0, wall: float = 0.5):
    rectangles = [
        (0.0, 0.0, width, wall),
        (0.0, height - wall, width, height),
        (0.0, 0.0, wall, height),
        (width - wall, 0.0, width, height),
    ]
    field = DistanceField.from_rectangles(rectangles, width, height, 0.05)
    return rectangles, CarCollisionChecker(field)


# ------------------------------------------------------------------ car model


def test_car_geometry_matches_the_specification(car):
    assert car.wheelbase == pytest.approx(2.7)
    assert car.length == pytest.approx(4.5)
    assert car.width == pytest.approx(1.9)
    assert car.max_steer == pytest.approx(0.6)
    assert car.rear_overhang + car.front_overhang == pytest.approx(car.length)
    assert car.min_turning_radius == pytest.approx(2.7 / math.tan(0.6))


def test_the_discs_cover_the_whole_footprint(car):
    """Every corner of the rectangle must lie inside some disc."""
    for theta in np.linspace(-math.pi, math.pi, 17):
        corners = car.corners(3.0, 4.0, float(theta))
        centres = car.disc_centres(3.0, 4.0, float(theta))
        for corner in corners:
            distances = np.hypot(centres[:, 0] - corner[0], centres[:, 1] - corner[1])
            assert distances.min() <= car.disc_radius + 1e-9


def test_a_straight_step_moves_along_the_heading(car):
    x, y, theta = car.step((1.0, 2.0, math.pi / 4.0), 0.0, 3.0)
    assert x == pytest.approx(1.0 + 3.0 * math.cos(math.pi / 4.0))
    assert y == pytest.approx(2.0 + 3.0 * math.sin(math.pi / 4.0))
    assert theta == pytest.approx(math.pi / 4.0)


def test_a_full_circle_returns_to_the_start(car):
    pose = (0.0, 0.0, 0.0)
    circumference = 2.0 * math.pi * car.min_turning_radius
    end = car.step(pose, car.max_steer, circumference)
    assert end[0] == pytest.approx(0.0, abs=1e-9)
    assert end[1] == pytest.approx(0.0, abs=1e-9)
    assert abs(angle_difference(end[2], 0.0)) == pytest.approx(0.0, abs=1e-9)


def test_reverse_is_the_exact_inverse_of_forward(car):
    pose = (4.0, 5.0, 0.3)
    forward = car.step(pose, 0.4, 2.0)
    back = car.step(forward, 0.4, -2.0)
    assert back[0] == pytest.approx(pose[0])
    assert back[1] == pytest.approx(pose[1])
    assert angle_difference(back[2], pose[2]) == pytest.approx(0.0, abs=1e-12)


# --------------------------------------------------------- heading wraparound


def test_angles_wrap_across_the_pi_boundary():
    assert wrap_angle(3.0 * math.pi) == pytest.approx(math.pi)
    assert angle_difference(math.radians(179.0), math.radians(-179.0)) == pytest.approx(
        math.radians(-2.0), abs=1e-9
    )
    assert abs(angle_difference(-math.pi + 1e-6, math.pi - 1e-6)) == pytest.approx(2e-6, abs=1e-9)


def test_the_goal_test_treats_the_pi_boundary_as_continuous():
    tolerance = math.radians(5.0)
    assert at_goal((1.0, 1.0, math.radians(179.0)), (1.0, 1.0, math.radians(-179.0)), 0.3, tolerance)
    assert not at_goal((1.0, 1.0, math.radians(170.0)), (1.0, 1.0, math.radians(-179.0)), 0.3,
                       tolerance)


def test_heading_bins_wrap_and_cover_the_circle():
    bins = 72
    assert heading_bin(0.0, bins) == heading_bin(2.0 * math.pi, bins)
    assert heading_bin(-math.pi + 1e-9, bins) == heading_bin(math.pi + 1e-9, bins)
    assert len({heading_bin(2.0 * math.pi * i / bins + 1e-6, bins) for i in range(bins)}) == bins


def test_state_keys_discretise_position_and_heading():
    a = state_key((3.10, 4.10, 0.0), 0.5, 72)
    b = state_key((3.30, 4.30, 0.01), 0.5, 72)
    c = state_key((3.80, 4.10, 0.0), 0.5, 72)
    assert a == b and a != c


# ---------------------------------------------------------------- Reeds-Shepp


def test_every_reeds_shepp_candidate_lands_exactly_on_the_goal(car):
    rng = np.random.default_rng(7)
    checked = 0
    for _ in range(120):
        start = (float(rng.uniform(-15, 15)), float(rng.uniform(-15, 15)),
                 float(rng.uniform(-math.pi, math.pi)))
        goal = (float(rng.uniform(-15, 15)), float(rng.uniform(-15, 15)),
                float(rng.uniform(-math.pi, math.pi)))
        for path in rs.reeds_shepp_paths(start, goal, car.max_curvature):
            poses, directions = rs.interpolate(start, path, car, step=0.05)
            end = poses[-1]
            assert math.hypot(end[0] - goal[0], end[1] - goal[1]) < 1e-6
            assert abs(angle_difference(end[2], goal[2])) < 1e-6
            assert len(directions) == len(poses) - 1
            checked += 1
    assert checked > 500


def test_a_reeds_shepp_path_is_never_shorter_than_the_straight_line(car):
    rng = np.random.default_rng(11)
    for _ in range(200):
        start = (0.0, 0.0, float(rng.uniform(-math.pi, math.pi)))
        goal = (float(rng.uniform(-12, 12)), float(rng.uniform(-12, 12)),
                float(rng.uniform(-math.pi, math.pi)))
        path = rs.shortest_reeds_shepp(start, goal, car.max_curvature)
        assert path is not None
        assert rs.path_length_m(path, car) >= math.hypot(goal[0], goal[1]) - 1e-9


def test_a_straight_ahead_goal_gives_a_straight_forward_curve(car):
    path = rs.shortest_reeds_shepp((0.0, 0.0, 0.0), (6.0, 0.0, 0.0), car.max_curvature)
    assert rs.path_length_m(path, car) == pytest.approx(6.0, rel=1e-6)
    _, directions = rs.interpolate((0.0, 0.0, 0.0), path, car, 0.2)
    assert all(direction > 0 for direction in directions)


def test_a_goal_straight_behind_is_reached_in_reverse(car):
    path = rs.shortest_reeds_shepp((0.0, 0.0, 0.0), (-6.0, 0.0, 0.0), car.max_curvature)
    assert rs.path_length_m(path, car) == pytest.approx(6.0, rel=1e-6)
    _, directions = rs.interpolate((0.0, 0.0, 0.0), path, car, 0.2)
    assert all(direction < 0 for direction in directions)


# ------------------------------------------------------------ collision model


def test_the_distance_field_never_overstates_the_clearance():
    """The planner's field must be a lower bound on the true distance."""
    rectangles = [(4.0, 3.0, 7.0, 5.0)]
    field = DistanceField.from_rectangles(rectangles, 12.0, 9.0, 0.05)
    rng = np.random.default_rng(3)
    for _ in range(400):
        x, y = float(rng.uniform(0, 12)), float(rng.uniform(0, 9))
        reported = float(field.distance_at(np.array([x]), np.array([y]))[0])
        x0, y0, x1, y1 = rectangles[0]
        true = math.hypot(max(x0 - x, 0.0, x - x1), max(y0 - y, 0.0, y - y1))
        assert reported <= true + 1e-9


def test_a_pose_the_planner_accepts_is_accepted_by_the_exact_checker(car):
    """The disc model must be conservative against the exact rectangle test."""
    rectangles = [(0.0, 0.0, 30.0, 0.5), (0.0, 19.5, 30.0, 20.0),
                  (10.0, 6.0, 14.0, 12.0), (20.0, 4.0, 22.0, 16.0)]
    field = DistanceField.from_rectangles(rectangles, 30.0, 20.0, 0.05)
    checker = CarCollisionChecker(field, car)
    rng = np.random.default_rng(5)
    accepted = 0
    for _ in range(3000):
        pose = (float(rng.uniform(0, 30)), float(rng.uniform(0, 20)),
                float(rng.uniform(-math.pi, math.pi)))
        if checker.is_free(pose):
            accepted += 1
            report = check_car_path([pose], rectangles, car.length, car.width, car.rear_overhang)
            assert report.ok, report.describe()
    assert accepted > 300


def test_the_exact_checker_catches_a_car_driven_into_a_wall(car):
    report = check_car_path([(5.0, 5.0, 0.0), (5.0, 0.2, 0.0)], [(0.0, 0.0, 20.0, 0.5)],
                            car.length, car.width, car.rear_overhang)
    assert not report.ok and "overlaps" in report.describe()


def test_densify_bounds_the_sample_spacing():
    dense = densify([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)], step=0.05)
    gaps = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(dense, dense[1:])]
    assert max(gaps) <= 0.05 + 1e-9
    assert dense[0] == (0.0, 0.0, 0.0) and dense[-1] == (1.0, 0.0, 0.0)


def test_car_corners_match_the_car_model(car):
    pose = (2.0, 3.0, 0.7)
    exact = car_corners(pose, car.length, car.width, car.rear_overhang)
    assert np.allclose(exact, car.corners(*pose))


# ------------------------------------------------------------------ hybrid A*


def test_a_straight_goal_in_an_open_lot_needs_no_reverse(car):
    _, checker = _open_lot()
    result = plan((6.0, 15.0, 0.0), (26.0, 15.0, 0.0), checker, car=car)
    assert result.success, result.reason
    assert not result.has_reverse
    assert result.direction_switches == 0
    assert result.path_length_m == pytest.approx(20.0, rel=0.05)


def test_an_open_lot_plan_ends_inside_the_goal_tolerance(car):
    goal_cfg = load_config("hybrid")["goal"]
    _, checker = _open_lot()
    goal = (24.0, 12.0, math.radians(30.0))
    result = plan((6.0, 15.0, 0.0), goal, checker, car=car)
    assert result.success
    end = result.poses[-1]
    assert math.hypot(end[0] - goal[0], end[1] - goal[1]) <= float(goal_cfg["position_tolerance"])
    assert abs(angle_difference(end[2], goal[2])) <= math.radians(
        float(goal_cfg["heading_tolerance_deg"])
    )


def test_a_goal_directly_behind_the_car_is_reached_in_reverse(car):
    _, checker = _open_lot()
    result = plan((26.0, 15.0, 0.0), (18.0, 15.0, 0.0), checker, car=car)
    assert result.success
    assert result.has_reverse


def test_every_parking_type_is_solved_and_verified(solved):
    for (name, seed), (scenario, result) in solved.items():
        assert result.success, f"{name} seed {seed}: {result.reason}"
        report = check_plan(scenario, result)
        assert report.ok, f"{name} seed {seed}: {report.describe()}"


def test_reverse_in_scenarios_actually_use_reverse(solved):
    for (name, seed), (scenario, result) in solved.items():
        if not scenario.requires_reverse:
            continue
        assert result.has_reverse, f"{name} seed {seed} parked without ever reversing"
        assert result.reverse_length_m > 0.0


def test_plans_start_and_end_where_they_should(solved):
    goal_cfg = load_config("hybrid")["goal"]
    for (name, seed), (scenario, result) in solved.items():
        assert result.poses[0] == pytest.approx(scenario.start)
        end = result.poses[-1]
        assert math.hypot(end[0] - scenario.goal[0], end[1] - scenario.goal[1]) <= float(
            goal_cfg["position_tolerance"]
        ) + 1e-9
        assert abs(angle_difference(end[2], scenario.goal[2])) <= math.radians(
            float(goal_cfg["heading_tolerance_deg"])
        ) + 1e-9
        assert len(result.directions) == len(result.poses) - 1


def test_consecutive_poses_respect_the_curvature_limit(solved, car):
    """No plan may turn faster than the car's steering allows.

    Consecutive samples lie on one constant-curvature arc, so the curvature
    follows exactly from the chord and the change of heading:
    ``chord = 2 R sin(turn / 2)``.
    """
    limit = car.max_curvature + 1e-9
    for (name, seed), (_, result) in solved.items():
        for a, b in zip(result.poses, result.poses[1:]):
            chord = math.hypot(b[0] - a[0], b[1] - a[1])
            if chord < 1e-9:
                continue
            turn = abs(angle_difference(b[2], a[2]))
            curvature = 2.0 * math.sin(turn / 2.0) / chord
            assert curvature <= limit, f"{name} seed {seed}: curvature {curvature:.4f}"


def test_the_search_is_deterministic():
    scenario = generate_parking_scenario("parallel", 1)
    first = plan_for_scenario(scenario, collect_explored=False)
    second = plan_for_scenario(scenario, collect_explored=False)
    assert first.cost == pytest.approx(second.cost)
    assert len(first.poses) == len(second.poses)
    assert first.nodes_expanded == second.nodes_expanded


def test_a_goal_inside_an_obstacle_is_rejected_cleanly(car):
    rectangles, checker = _open_lot()
    result = plan((6.0, 15.0, 0.0), (0.2, 15.0, 0.0), checker, car=car)
    assert result.success is False
    assert result.reason == "goal pose is in collision"
    assert result.poses == []


def test_a_start_inside_an_obstacle_is_rejected_cleanly(car):
    _, checker = _open_lot()
    result = plan((0.2, 15.0, 0.0), (20.0, 15.0, 0.0), checker, car=car)
    assert result.success is False
    assert result.reason == "start pose is in collision"


def test_a_walled_off_goal_returns_no_path_within_the_caps(car):
    width, height, wall = 30.0, 20.0, 0.5
    rectangles = [
        (0.0, 0.0, width, wall), (0.0, height - wall, width, height),
        (0.0, 0.0, wall, height), (width - wall, 0.0, width, height),
        (14.0, 0.0, 16.0, height),  # a wall right across the lot
    ]
    field = DistanceField.from_rectangles(rectangles, width, height, 0.05)
    result = plan((6.0, 10.0, 0.0), (24.0, 10.0, 0.0), CarCollisionChecker(field, car), car=car)
    assert result.success is False
    assert result.reason in {"no path found", "expansion limit reached", "time limit reached",
                             "goal is unreachable from the start"}


def test_the_expansion_cap_is_respected(car):
    config = load_config("hybrid")
    config["limits"]["max_expansions"] = 12
    config["analytic"]["enabled"] = False
    _, checker = _open_lot()
    result = plan((6.0, 15.0, 0.0), (34.0, 5.0, 2.0), checker, config=config, car=car)
    assert result.success is False
    assert result.reason == "expansion limit reached"
    assert result.nodes_expanded == 12


def test_reverse_costs_more_than_forward(car):
    """Doubling the reverse multiplier must not make a reverse plan cheaper."""
    _, checker = _open_lot()
    base = load_config("hybrid")
    expensive = load_config("hybrid")
    expensive["costs"]["reverse_multiplier"] = 8.0
    cheap_plan = plan((26.0, 15.0, 0.0), (18.0, 15.0, 0.0), checker, config=base, car=car)
    dear_plan = plan((26.0, 15.0, 0.0), (18.0, 15.0, 0.0), checker, config=expensive, car=car)
    assert cheap_plan.success and dear_plan.success
    assert dear_plan.cost >= cheap_plan.cost - 1e-9


# ------------------------------------------------------- scenarios and battery


def test_parking_scenarios_are_deterministic_under_seed():
    for name in PARKING_TYPES:
        first = generate_parking_scenario(name, 9)
        second = generate_parking_scenario(name, 9)
        assert first.obstacles == second.obstacles
        assert first.start == second.start and first.goal == second.goal
        assert first.layout == second.layout


def test_start_and_goal_poses_are_collision_free(solved):
    for (name, seed), (scenario, _) in solved.items():
        checker = scenario.checker()
        assert checker.is_free(scenario.start), f"{name} seed {seed}"
        assert checker.is_free(scenario.goal), f"{name} seed {seed}"


def test_parked_cars_are_static_obstacles_only(solved):
    for (_, _), (scenario, _) in solved.items():
        assert len(scenario.obstacles) > 4          # four walls plus parked cars
        assert not hasattr(scenario, "agents")


def test_tight_scenarios_use_one_of_the_base_layouts():
    layouts = {generate_parking_scenario("tight", seed).layout for seed in range(12)}
    assert layouts and layouts <= {"perpendicular_forward", "perpendicular_reverse", "parallel"}


def test_an_unknown_parking_type_is_rejected():
    with pytest.raises(ValueError):
        generate_parking_scenario("diagonal_drift", 0)


def test_impossible_parking_settings_raise_cleanly():
    config = load_config("parking")
    config["start"]["max_attempts"] = 2
    config["start"]["offset"] = [1e6, 1e6]
    config["perpendicular"]["spot_width"] = 1.0  # no car fits, so nothing is ever free
    with pytest.raises(ParkingGenerationError):
        generate_parking_scenario("perpendicular_forward", 0, config=config)


@pytest.fixture(scope="module")
def parking_battery():
    return run_parking_battery(episodes_per_type=3, verbose=False)


def test_the_parking_battery_records_the_required_columns(parking_battery):
    assert len(parking_battery) == len(PARKING_TYPES) * 3
    for column in ("parking_type", "seed", "success", "planning_ms", "nodes_expanded",
                   "path_length_m", "direction_switches"):
        assert column in parking_battery.columns
    assert parking_battery["verified_collision_free"][parking_battery["success"]].all()


def test_the_parking_summary_and_failure_views_are_well_formed(parking_battery):
    summary = summarise_parking(parking_battery)
    assert len(summary) == len(PARKING_TYPES)
    assert (summary["success_pct"] >= 0).all() and (summary["success_pct"] <= 100).all()
    assert len(parking_failures(parking_battery)) == (~parking_battery["success"]).sum()


def test_the_parking_csv_round_trips(parking_battery, tmp_path):
    import pandas as pd

    out = write_parking_battery(parking_battery, tmp_path / "parking.csv")
    assert len(pd.read_csv(out)) == len(parking_battery)


def test_a_parking_plan_animates_to_a_small_gif(tmp_path):
    scenario = generate_parking_scenario("perpendicular_reverse", 0)
    result = plan_for_scenario(scenario)
    out = animate_parking(scenario, result, tmp_path / "parking.gif")
    assert out.is_file() and gif_size_mb(out) < 5.0
