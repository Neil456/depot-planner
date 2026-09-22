"""Scenario battery: run every planner on the same seeded scenarios."""

from __future__ import annotations

import time
from pathlib import Path
from typing import Any, Callable, Sequence

import pandas as pd

from depot_planner import core
from depot_planner.config import deep_merge, load_config, results_path
from depot_planner.sim.runner import EpisodeResult, run_episode, verify_episode
from depot_planner.spacetime.planners import make_planner
from depot_planner.world.scenarios import SCENARIO_TYPES, Scenario, generate_scenario

BATTERY_CSV = "battery_spacetime.csv"
HARD_BATTERY_CSV = "battery_spacetime_hard.csv"

TIERS: tuple[str, ...] = ("normal", "hard")


def tier_settings(tier: str, config: dict[str, Any] | None = None) -> dict[str, Any]:
    """The battery block for ``tier``."""
    if tier not in TIERS:
        raise ValueError(f"unknown tier {tier!r}; expected one of {TIERS}")
    cfg = config if config is not None else load_config("eval")
    return cfg["battery" if tier == "normal" else "hard_battery"]


def tier_planner_config(tier: str, config: dict[str, Any] | None = None) -> dict[str, Any] | None:
    """Planner config for ``tier``: ``None`` for normal, a time budget for hard."""
    if tier == "normal":
        return None
    limit = tier_settings(tier, config).get("planning_time_limit_ms")
    if limit is None:
        return None
    return deep_merge(
        load_config("spacetime"),
        {"spacetime": {"time_limit_ms": float(limit)},
         "baseline": {"time_limit_ms": float(limit)}},
    )


def tier_csv(tier: str) -> str:
    """Name of the CSV this tier writes."""
    return BATTERY_CSV if tier == "normal" else HARD_BATTERY_CSV


def battery_seeds(count: int, offset: int) -> list[int]:
    """The ``count`` consecutive seeds a tier uses, starting at ``offset``."""
    return [offset + i for i in range(count)]


def run_battery(
    scenario_types: Sequence[str] | None = None,
    episodes_per_type: int | None = None,
    planners: Sequence[str] | None = None,
    config: dict[str, Any] | None = None,
    on_episode: Callable[[Scenario, EpisodeResult], None] | None = None,
    verbose: bool = True,
    tier: str = "normal",
    backend: str | None = None,
) -> pd.DataFrame:
    """Run one tier of the battery and return one row per episode.

    Both planners see the *same* scenario objects, so the comparison is paired.
    Every episode is re-checked with the independent collision checker; a plan a
    planner believed was safe but which the checker rejects raises immediately.

    ``backend`` picks the planner implementation: ``"cpp"``, ``"python"`` for
    the reference, or ``None`` for the default.
    """
    cfg = tier_settings(tier, config)
    episodes = int(episodes_per_type if episodes_per_type is not None else cfg["episodes_per_type"])
    names = list(planners if planners is not None else cfg["planners"])
    seeds = battery_seeds(episodes, int(cfg["seed_offset"]))
    types = list(
        scenario_types if scenario_types is not None
        else cfg.get("scenario_types", SCENARIO_TYPES)
    )
    planner_config = tier_planner_config(tier, config)

    rows: list[dict[str, Any]] = []
    began = time.perf_counter()
    for scenario_type in types:
        for seed in seeds:
            scenario = generate_scenario(scenario_type, seed)
            for planner_name in names:
                planner = make_planner(planner_name, planner_config, backend)
                episode = run_episode(scenario, planner)
                report = verify_episode(scenario, episode)
                if episode.success and not report.ok:
                    raise AssertionError(
                        f"{planner_name} reported success on {scenario_type} seed {seed} "
                        f"but the independent checker found {report.describe()}"
                    )
                row = episode.as_row()
                row["tier"] = tier
                row["backend"] = core.resolve(backend)
                row["base_scenario"] = scenario.base or scenario.name
                rows.append(row)
                if on_episode is not None:
                    on_episode(scenario, episode)
        if verbose:
            done = len(rows)
            print(f"  {scenario_type:22s} {done:4d} episodes  ({time.perf_counter() - began:5.1f}s)")
    return pd.DataFrame(rows)


def write_battery(frame: pd.DataFrame, path: Path | str | None = None) -> Path:
    """Write the episode rows to CSV and return the path."""
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
