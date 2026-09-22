#!/usr/bin/env python3
"""Step 4: one GIF per scenario type for each planner, under results/gifs/.

For each scenario type a single seed is chosen and used for *both* planners, so
the two GIFs show the same traffic and can be compared side by side. The chosen
seed is the most illustrative one in the search slice: first preference to a seed
where the baseline collides, then to the one where space-time A* waits most.
"""

from __future__ import annotations

from depot_planner.config import load_config, results_path
from depot_planner.eval.battery import battery_seeds
from depot_planner.sim.runner import run_episode
from depot_planner.spacetime.planners import make_planner
from depot_planner.viz.animate import animate_episode, gif_size_mb
from depot_planner.world.scenarios import SCENARIO_TYPES, generate_scenario

SEARCH_SEEDS = 10


def choose_seed(scenario_type: str, seeds: list[int], planner_names: list[str]):
    """Return ``(seed, scenario, {planner: episode})`` for the most telling seed."""
    best = None
    for seed in seeds:
        scenario = generate_scenario(scenario_type, seed)
        episodes = {
            name: run_episode(scenario, make_planner(name)) for name in planner_names
        }
        baseline_failed = any(
            not e.success for name, e in episodes.items() if name != "spacetime_astar"
        )
        timed = episodes.get("spacetime_astar")
        score = (1 if baseline_failed else 0, timed.wait_steps if timed else 0)
        if best is None or score > best[0]:
            best = (score, seed, scenario, episodes)
    _, seed, scenario, episodes = best
    return seed, scenario, episodes


def main() -> None:
    cfg = load_config("eval")["battery"]
    planner_names = list(cfg["planners"])
    seeds = battery_seeds(min(SEARCH_SEEDS, int(cfg["episodes_per_type"])), int(cfg["seed_offset"]))

    for scenario_type in SCENARIO_TYPES:
        seed, scenario, episodes = choose_seed(scenario_type, seeds, planner_names)
        for planner_name in planner_names:
            episode = episodes[planner_name]
            out = animate_episode(
                scenario, episode, results_path("gifs", f"{scenario_type}_{planner_name}.gif")
            )
            outcome = "goal" if episode.success else ("collision" if episode.collision else "timeout")
            print(f"{scenario_type:22s} {planner_name:16s} seed {seed} {outcome:9s} "
                  f"waits {episode.wait_steps:3d}  {gif_size_mb(out):.2f} MB -> {out.name}")


if __name__ == "__main__":
    main()
