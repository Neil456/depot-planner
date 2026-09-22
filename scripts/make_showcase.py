#!/usr/bin/env python3
"""Write the two polished README animations.

``results/README_assets/hero.gif``         the hero: space-time A* yielding to traffic
``results/README_assets/side_by_side.gif`` the baseline colliding beside it, same seed
``results/README_assets/parking.gif``      hybrid A* reversing into a parallel bay

Every seed is chosen from the battery's own seed slice, so these GIFs always
show a real battery scenario and regenerate identically.
"""

from __future__ import annotations

from depot_planner.config import load_config, results_path
from depot_planner.eval.battery import battery_seeds
from depot_planner.eval.showcase import (
    PRIMARY_PLANNER,
    best_contrast_episode,
    best_parking_episode,
)
from depot_planner.viz.animate import gif_size_mb
from depot_planner.viz.showcase import hero_gif, parking_gif, side_by_side_gif

#: Crossing traffic is the clearest demonstration of planning in time.
SHOWCASE_SCENARIO = "crossing"
#: A parallel bay forces the most forward/reverse switching of any parking type.
SHOWCASE_PARKING = "parallel"
SEARCH_SEEDS = 10
SIZE_LIMIT_MB = 4.0


def main() -> None:
    cfg = load_config("eval")["battery"]
    seeds = battery_seeds(min(SEARCH_SEEDS, int(cfg["episodes_per_type"])), int(cfg["seed_offset"]))
    scenario, episodes = best_contrast_episode(SHOWCASE_SCENARIO, seeds)

    primary = episodes[PRIMARY_PLANNER]
    hero = hero_gif(scenario, primary, results_path("README_assets", "hero.gif"))
    ordered = [(PRIMARY_PLANNER, primary)] + [
        (name, episode) for name, episode in episodes.items() if name != PRIMARY_PLANNER
    ]
    pair = side_by_side_gif(scenario, ordered, results_path("README_assets", "side_by_side.gif"))

    parking_scenario, parking_result = best_parking_episode(SHOWCASE_PARKING, seeds)
    park = parking_gif(parking_scenario, parking_result,
                       results_path("README_assets", "parking.gif"))

    for out in (hero, pair, park):
        size = gif_size_mb(out)
        status = "ok" if size < SIZE_LIMIT_MB else f"TOO BIG (limit {SIZE_LIMIT_MB} MB)"
        print(f"{out.name:20s} {size:5.2f} MB  {status}")
    print(f"\n{SHOWCASE_SCENARIO} seed {scenario.seed}: " + ", ".join(
        f"{name} {'goal' if e.success else 'collision' if e.collision else 'timeout'}"
        f" ({e.wait_steps} waits)" for name, e in ordered
    ))
    print(f"{SHOWCASE_PARKING} seed {parking_scenario.seed}: "
          f"{parking_result.direction_switches} direction changes, "
          f"{parking_result.reverse_length_m:.1f} m in reverse, "
          f"{parking_result.path_length_m:.1f} m total")
    for out in (hero, pair, park):
        if gif_size_mb(out) >= SIZE_LIMIT_MB:
            raise SystemExit(f"{out.name} exceeds {SIZE_LIMIT_MB} MB")


if __name__ == "__main__":
    main()
