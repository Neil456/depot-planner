"""Scenario battery: run every planner on the same seeded scenarios."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any, Callable, Iterable, Sequence

import pandas as pd

from depot_planner.config import load_config, results_path
from depot_planner.sim.runner import EpisodeResult, run_episode, verify_episode
from depot_planner.spacetime.planners import make_planner
from depot_planner.world.scenarios import SCENARIO_TYPES, Scenario, generate_scenario

BATTERY_CSV = "battery_spacetime.csv"


def battery_seeds(count: int, offset: int) -> list[int]:
    return [offset + i for i in range(count)]


def run_battery(
    scenario_types: Sequence[str] = SCENARIO_TYPES,
    episodes_per_type: int | None = None,
    planners: Sequence[str] | None = None,
    config: dict[str, Any] | None = None,
    on_episode: Callable[[Scenario, EpisodeResult], None] | None = None,
    verbose: bool = True,
) -> pd.DataFrame:
    """Run the battery and return one row per episode.

    Both planners see the *same* scenario objects, so the comparison is paired.
    Every episode is re-checked with the independent collision checker; a plan a
    planner believed was safe but which the checker rejects raises immediately.
    """
    cfg = (config if config is not None else load_config("eval"))["battery"]
    episodes = int(episodes_per_type if episodes_per_type is not None else cfg["episodes_per_type"])
    names = list(planners if planners is not None else cfg["planners"])
    seeds = battery_seeds(episodes, int(cfg["seed_offset"]))

    rows: list[dict[str, Any]] = []
    began = time.perf_counter()
    for scenario_type in scenario_types:
        for seed in seeds:
            scenario = generate_scenario(scenario_type, seed)
            for planner_name in names:
                planner = make_planner(planner_name)
                episode = run_episode(scenario, planner)
                report = verify_episode(scenario, episode)
                if episode.success and not report.ok:
                    raise AssertionError(
                        f"{planner_name} reported success on {scenario_type} seed {seed} "
                        f"but the independent checker found {report.describe()}"
                    )
                rows.append(episode.as_row())
                if on_episode is not None:
                    on_episode(scenario, episode)
        if verbose:
            done = len(rows)
            print(f"  {scenario_type:22s} {done:4d} episodes  ({time.perf_counter() - began:5.1f}s)")
    return pd.DataFrame(rows)


def write_battery(frame: pd.DataFrame, path: Path | str | None = None) -> Path:
    out = Path(path) if path is not None else results_path(BATTERY_CSV)
    out.parent.mkdir(parents=True, exist_ok=True)
    frame.to_csv(out, index=False)
    return out


def summarise(frame: pd.DataFrame) -> pd.DataFrame:
    """Per scenario type x planner summary used by the report."""
    grouped = frame.groupby(["scenario", "planner"], sort=False)
    summary = grouped.agg(
        episodes=("success", "size"),
        success_pct=("success", lambda s: 100.0 * s.mean()),
        collisions=("collision", "sum"),
        timeouts=("timeout", "sum"),
        mean_time_to_goal_s=("time_to_goal_s", "mean"),
        mean_wait_steps=("wait_steps", "mean"),
        mean_planning_ms=("mean_planning_ms", "mean"),
        max_planning_ms=("max_planning_ms", "max"),
        mean_nodes_expanded=("nodes_expanded", "mean"),
    ).reset_index()
    return summary


def failures(frame: pd.DataFrame) -> pd.DataFrame:
    """Every episode that did not reach the goal."""
    failed = frame[~frame["success"]].copy()
    return failed.sort_values(["planner", "scenario", "seed"]).reset_index(drop=True)
