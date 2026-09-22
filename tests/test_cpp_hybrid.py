"""C++ port, step 4: hybrid A*, the car model and Reeds-Shepp.

Every check compares the compiled core against the pure-Python reference on the
same scenarios, and hands each C++ plan to the Python independent checker, which
shares no code with either planner. The full sweep over every parking battery
seed lives in ``scripts/check_equivalence.py``.
"""

from __future__ import annotations

import math

import numpy as np
import pytest

from depot_planner import core
from depot_planner.eval.parking_battery import (
    parking_tier_settings,
    run_parking_battery,
    tier_hybrid_config,
)
from depot_planner.hybrid import reeds_shepp as rs
from depot_planner.hybrid.car import CarModel, angle_difference
from depot_planner.hybrid.search import plan_for_scenario
from depot_planner.sim.car_collision import check_plan
from depot_planner.world.parking import PARKING_TYPES, generate_parking_scenario

needs_cpp = pytest.mark.skipif(not core.available(), reason="the C++ core is not built")

#: The fields two plans must agree on, exactly.
PLAN_FIELDS = ("success", "nodes_expanded", "direction_switches", "used_analytic",
               "collision_checks", "reason")

SEEDS = (1000, 1001, 1002)


def _assert_same_plan(a, b, what: str) -> None:
    for name in PLAN_FIELDS:
        assert getattr(a, name) == getattr(b, name), f"{what}: {name} differs"
    if math.isinf(a.cost):
        assert math.isinf(b.cost), f"{what}: cost differs"
    else:
        assert a.cost == pytest.approx(b.cost, abs=1e-12, rel=0.0), f"{what}: cost differs"
    assert a.poses == b.poses, f"{what}: the pose sequences differ"
    assert a.directions == b.directions, f"{what}: the gears differ"
    assert a.path_length_m == pytest.approx(b.path_length_m, abs=1e-12, rel=0.0)


# ---------------------------------------------------------------- planning


@needs_cpp
@pytest.mark.parametrize("parking_type", PARKING_TYPES)
def test_both_backends_return_the_same_parking_plan(parking_type):
    for seed in SEEDS:
        scenario = generate_parking_scenario(parking_type, seed)
        plans = {
            backend: plan_for_scenario(scenario, collect_explored=False, backend=backend)
            for backend in ("python", "cpp")
        }
        _assert_same_plan(plans["python"], plans["cpp"], f"{parking_type} seed {seed}")


@needs_cpp
@pytest.mark.parametrize("parking_type", PARKING_TYPES)
def test_every_cpp_plan_passes_the_independent_checker(parking_type):
    for seed in SEEDS:
        scenario = generate_parking_scenario(parking_type, seed)
        result = plan_for_scenario(scenario, collect_explored=False, backend="cpp")
        if not result.success:
            continue
        report = check_plan(scenario, result)
        assert report.ok, f"{parking_type} seed {seed}: {report.describe()}"


@needs_cpp
def test_the_hard_tier_also_agrees():
    cfg = parking_tier_settings("hard")
    hybrid_cfg = tier_hybrid_config("hard")
    offset = int(cfg["seed_offset"])
    for parking_type in cfg["parking_types"]:
        for index in range(3):
            scenario = generate_parking_scenario(parking_type, offset + index,
                                                 hybrid_config=hybrid_cfg)
            plans = {
                backend: plan_for_scenario(scenario, collect_explored=False, backend=backend)
                for backend in ("python", "cpp")
            }
            _assert_same_plan(plans["python"], plans["cpp"],
                              f"{parking_type} seed {offset + index}")


@needs_cpp
def test_asking_for_the_explored_poses_falls_back_to_the_reference():
    scenario = generate_parking_scenario("perpendicular_reverse", SEEDS[0])
    result = plan_for_scenario(scenario, collect_explored=True, backend="auto")
    assert result.success
    assert result.explored


@needs_cpp
def test_the_expansion_cap_is_reported_identically():
    from depot_planner.config import deep_merge, load_config

    config = deep_merge(load_config("hybrid"), {"limits": {"max_expansions": 25}})
    scenario = generate_parking_scenario("parallel", 1000, hybrid_config=config)
    plans = {
        backend: plan_for_scenario(scenario, config=config, collect_explored=False,
                                   backend=backend)
        for backend in ("python", "cpp")
    }
    for plan in plans.values():
        assert plan.success is False
        assert plan.reason == "expansion limit reached"
    assert plans["python"].nodes_expanded == plans["cpp"].nodes_expanded == 25


@needs_cpp
def test_a_parking_battery_slice_matches_on_every_non_timing_column():
    frames = {
        backend: run_parking_battery(parking_types=["perpendicular_reverse", "parallel"],
                                     episodes_per_type=4, verbose=False, backend=backend)
        for backend in ("python", "cpp")
    }
    columns = [c for c in frames["python"].columns if c not in {"planning_ms", "backend"}]
    assert frames["python"][columns].equals(frames["cpp"][columns])


# ------------------------------------------------------------ Reeds-Shepp


@needs_cpp
def test_the_cpp_reeds_shepp_picks_the_same_curve():
    car = CarModel()
    rng = np.random.default_rng(20240618)
    compared = 0
    for _ in range(400):
        start = tuple(rng.uniform([-12, -12, -math.pi], [12, 12, math.pi]))
        goal = tuple(rng.uniform([-12, -12, -math.pi], [12, 12, math.pi]))
        theirs = rs.shortest_reeds_shepp(start, goal, car.max_curvature)
        ours = core.require().shortest_reeds_shepp(start, goal, car.max_curvature)
        if theirs is None:
            assert ours is None
            continue
        segments, length = ours
        assert length == pytest.approx(theirs.length, abs=1e-15, rel=0.0)
        assert len(segments) == len(theirs.segments)
        for (steering, value), segment in zip(segments, theirs.segments):
            assert steering == segment.steering
            assert value == segment.length
        compared += 1
    assert compared > 350


@needs_cpp
def test_the_cpp_interpolation_matches_pose_for_pose():
    car = CarModel()
    rng = np.random.default_rng(4242)
    for _ in range(200):
        start = tuple(rng.uniform([-10, -10, -math.pi], [10, 10, math.pi]))
        goal = tuple(rng.uniform([-10, -10, -math.pi], [10, 10, math.pi]))
        path = rs.shortest_reeds_shepp(start, goal, car.max_curvature)
        if path is None:
            continue
        theirs, their_gears = rs.interpolate(start, path, car, 0.2)
        segments = [(s.steering, s.length) for s in path.segments]
        ours, our_gears = core.require().interpolate_reeds_shepp(
            start, segments, {
                "wheelbase": car.wheelbase, "length": car.length, "width": car.width,
                "max_steer": car.max_steer, "rear_overhang": car.rear_overhang,
                "n_discs": car.n_discs,
            }, 0.2,
        )
        assert list(our_gears) == their_gears
        assert [tuple(row) for row in ours] == [tuple(p) for p in theirs]


@needs_cpp
def test_the_curves_land_on_the_goal():
    """The same numerical validation the Python implementation was held to."""
    car = CarModel()
    rng = np.random.default_rng(777)
    checked = 0
    for _ in range(300):
        start = tuple(rng.uniform([-12, -12, -math.pi], [12, 12, math.pi]))
        goal = tuple(rng.uniform([-12, -12, -math.pi], [12, 12, math.pi]))
        result = core.require().shortest_reeds_shepp(start, goal, car.max_curvature)
        if result is None:
            continue
        segments, _ = result
        poses, _ = core.require().interpolate_reeds_shepp(
            start, list(segments), {
                "wheelbase": car.wheelbase, "length": car.length, "width": car.width,
                "max_steer": car.max_steer, "rear_overhang": car.rear_overhang,
                "n_discs": car.n_discs,
            }, 0.2,
        )
        end = poses[-1]
        assert end[0] == pytest.approx(goal[0], abs=1e-7)
        assert end[1] == pytest.approx(goal[1], abs=1e-7)
        assert abs(angle_difference(float(end[2]), goal[2])) < 1e-7
        checked += 1
    assert checked > 250
