"""Choosing the episodes the README's GIFs show.

Selection is data driven and deterministic: the same seed slice always yields
the same pick, so the committed GIFs can be regenerated exactly.
"""

from __future__ import annotations

from typing import Any, Sequence

from depot_planner.config import load_config
from depot_planner.sim.runner import EpisodeResult, run_episode
from depot_planner.spacetime.planners import make_planner
from depot_planner.world.scenarios import Scenario, generate_scenario

#: The planner whose behaviour the showcase is meant to demonstrate.
PRIMARY_PLANNER = "spacetime_astar"


def best_contrast_episode(
    scenario_type: str,
    seeds: Sequence[int],
    planners: Sequence[str] | None = None,
    config: dict[str, Any] | None = None,
) -> tuple[Scenario, dict[str, EpisodeResult]]:
    """Find the seed that best contrasts the planners on one scenario type.

    Preference order: a seed where the primary planner reaches the goal but
    another planner does not, then the seed where the primary planner waits the
    longest. Returns the scenario and one episode per planner, all run on that
    same scenario so the GIFs are directly comparable.
    """
    cfg = (config if config is not None else load_config("eval"))["battery"]
    names = list(planners if planners is not None else cfg["planners"])
    if PRIMARY_PLANNER not in names:
        raise ValueError(f"{PRIMARY_PLANNER!r} must be among the planners to contrast")

    best: tuple[tuple[int, int], Scenario, dict[str, EpisodeResult]] | None = None
    for seed in seeds:
        scenario = generate_scenario(scenario_type, int(seed))
        episodes = {name: run_episode(scenario, make_planner(name)) for name in names}
        primary = episodes[PRIMARY_PLANNER]
        others_failed = any(
            not episode.success for name, episode in episodes.items() if name != PRIMARY_PLANNER
        )
        score = (1 if (primary.success and others_failed) else 0, primary.wait_steps)
        if best is None or score > best[0]:
            best = (score, scenario, episodes)
    if best is None:
        raise ValueError(f"no seeds given for {scenario_type!r}")
    return best[1], best[2]


def best_parking_episode(
    parking_type: str,
    seeds: Sequence[int],
    config: dict[str, Any] | None = None,
):
    """Find the most demonstrative solved scenario of one parking type.

    Preference goes to the plan with the most forward/reverse switches, then the
    most distance driven in reverse — that is, the manoeuvre that most obviously
    could not be done by a holonomic planner.
    """
    from depot_planner.hybrid.search import plan_for_scenario
    from depot_planner.world.parking import generate_parking_scenario

    best = None
    for seed in seeds:
        scenario = generate_parking_scenario(parking_type, int(seed), hybrid_config=config)
        result = plan_for_scenario(scenario)
        if not result.success:
            continue
        score = (result.direction_switches, result.reverse_length_m)
        if best is None or score > best[0]:
            best = (score, scenario, result)
    if best is None:
        raise ValueError(f"no solved {parking_type!r} scenario among the given seeds")
    return best[1], best[2]
