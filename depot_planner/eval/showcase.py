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
