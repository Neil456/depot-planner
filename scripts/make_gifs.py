#!/usr/bin/env python3
"""Step 4: one GIF per scenario type for each planner, under results/gifs/.

For each scenario type a single seed is chosen and used for *both* planners, so
the two GIFs show the same traffic and can be compared side by side. The seed
comes from :func:`depot_planner.eval.showcase.best_contrast_episode`.
"""

from __future__ import annotations

from depot_planner.config import load_config, results_path
from depot_planner.eval.battery import battery_seeds
from depot_planner.eval.showcase import best_contrast_episode
from depot_planner.viz.animate import animate_episode, gif_size_mb
from depot_planner.world.scenarios import SCENARIO_TYPES

SEARCH_SEEDS = 10


def main() -> None:
    cfg = load_config("eval")["battery"]
    planner_names = list(cfg["planners"])
    seeds = battery_seeds(min(SEARCH_SEEDS, int(cfg["episodes_per_type"])), int(cfg["seed_offset"]))

    for scenario_type in SCENARIO_TYPES:
        scenario, episodes = best_contrast_episode(scenario_type, seeds, planner_names)
        for planner_name in planner_names:
            episode = episodes[planner_name]
            out = animate_episode(
                scenario, episode, results_path("gifs", f"{scenario_type}_{planner_name}.gif")
            )
            outcome = "goal" if episode.success else ("collision" if episode.collision else "timeout")
            print(f"{scenario_type:22s} {planner_name:16s} seed {scenario.seed} {outcome:9s} "
                  f"waits {episode.wait_steps:3d}  {gif_size_mb(out):.2f} MB -> {out.name}")


if __name__ == "__main__":
    main()
