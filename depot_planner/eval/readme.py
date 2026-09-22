"""Filling the generated regions of README.md from the battery CSVs.

The prose in README.md is hand written; every number in it lives between
``<!-- BEGIN GENERATED: name -->`` and ``<!-- END GENERATED: name -->`` markers
and is rewritten from the CSVs under ``results/``. Nothing here recomputes a
planner metric, so the README can never drift from the report.
"""

from __future__ import annotations

import re
from pathlib import Path

import pandas as pd

from depot_planner.config import REPO_ROOT, load_config, results_path
from depot_planner.eval import report as helpers
from depot_planner.eval.battery import BATTERY_CSV, HARD_BATTERY_CSV, tier_csv, summarise
from depot_planner.eval.cpp_bench import CPP_CSV, summarise_cpp
from depot_planner.eval.grid_bench import GRID_CSV, summarise_grid
from depot_planner.eval.parking_battery import HARD_PARKING_CSV, PARKING_CSV, summarise_parking
from depot_planner.hybrid.car import CarModel
from depot_planner.world.parking import minimum_parallel_gap

MARKER = "<!-- {} GENERATED: {} -->"

README_PATH = REPO_ROOT / "README.md"

#: Every region README.md is expected to contain, in the order it contains them.
REGION_NAMES: tuple[str, ...] = (
    "grid_table",
    "spacetime_normal",
    "spacetime_hard",
    "spacetime_hard_python",
    "finding_budget",
    "parking_normal",
    "parking_hard",
    "finding_gaps",
    "cpp_table",
)


def _grid_table(frame: pd.DataFrame) -> str:
    return helpers.markdown_table(summarise_grid(frame), [
        ("algorithm", "algorithm", ""),
        ("backend", "backend", ""),
        ("mean_cost_ratio", "cost / optimal", ".4f"),
        ("mean_nodes_expanded", "nodes expanded", ".0f"),
        ("mean_runtime_ms", "mean ms", ".2f"),
    ])


def _spacetime_table(frame: pd.DataFrame) -> str:
    return helpers.markdown_table(summarise(frame), [
        ("scenario", "scenario", ""),
        ("planner", "planner", ""),
        ("success_pct", "success %", ".1f"),
        ("collisions", "collisions", ".0f"),
        ("timeouts", "timeouts", ".0f"),
        ("mean_wait_steps", "mean waits", ".1f"),
        ("mean_planning_ms", "mean plan ms", ".2f"),
    ])


def _parking_table(frame: pd.DataFrame) -> str:
    return helpers.markdown_table(summarise_parking(frame), [
        ("parking_type", "parking type", ""),
        ("success_pct", "success %", ".1f"),
        ("mean_planning_ms", "mean plan ms", ".1f"),
        ("mean_nodes_expanded", "nodes expanded", ".0f"),
        ("mean_direction_switches", "direction switches", ".2f"),
    ])


def _cpp_table(frame: pd.DataFrame) -> str:
    return helpers.markdown_table(summarise_cpp(frame), [
        ("planner", "planner", ""),
        ("cases", "cases", ".0f"),
        ("median_python_ms", "Python ms", ".4f"),
        ("median_cpp_ms", "C++ ms", ".4f"),
        ("median_speedup", "speedup", ".1f"),
        ("identical", "identical plans", ""),
    ])


def _pct(frame: pd.DataFrame, scenario: str, planner: str) -> float:
    rows = frame[(frame["scenario"] == scenario) & (frame["planner"] == planner)]
    return 100.0 * rows["success"].mean()


def _counts(frame: pd.DataFrame, scenario: str, planner: str) -> tuple[int, int]:
    """``(collisions, timeouts)`` for one scenario/planner pair."""
    rows = frame[(frame["scenario"] == scenario) & (frame["planner"] == planner)]
    return int(rows["collision"].sum()), int(rows["timeout"].sum())


def _plural(count: int, noun: str) -> str:
    return f"{count} {noun}" if count == 1 else f"{count} {noun}s"


def _one_sided_budget_finding(hard: pd.DataFrame, budget: float) -> str:
    """The finding when only one backend's hard tier is on disk."""
    narrow_timed = _pct(hard, "head_on_narrow", "spacetime_astar")
    narrow_base = _pct(hard, "head_on_narrow", "baseline_replan")
    _, narrow_timeouts = _counts(hard, "head_on_narrow", "spacetime_astar")
    congested_timed = _pct(hard, "congested_hard", "spacetime_astar")
    congested_base = _pct(hard, "congested_hard", "baseline_replan")
    timed_coll, timed_to = _counts(hard, "congested_hard", "spacetime_astar")
    base_coll, base_to = _counts(hard, "congested_hard", "baseline_replan")

    if narrow_base > narrow_timed:
        lead = "the cheap planner wins in narrow aisles"
    elif narrow_timed > narrow_base:
        lead = "space-time A* wins in narrow aisles"
    else:
        lead = "the two planners are level in narrow aisles"
    if congested_timed > congested_base:
        congested_verdict = "space-time A* comes out ahead"
    elif congested_timed < congested_base:
        congested_verdict = "the baseline comes out ahead"
    else:
        congested_verdict = "the two tie on success rate"
    return (
        f"**Under a {budget:.0f} ms budget per replan, {lead}.** On `head_on_narrow` the "
        f"replanning baseline reaches the goal in {narrow_base:.1f}% of episodes and "
        f"space-time A* in {narrow_timed:.1f}%, the latter losing "
        f"{_plural(narrow_timeouts, 'episode')} to timeouts. On `congested_hard` "
        f"{congested_verdict} ({congested_timed:.1f}% against {congested_base:.1f}%), and they "
        f"fail in different ways — space-time A* loses {_plural(timed_to, 'episode')} to "
        f"timeouts and {_plural(timed_coll, 'episode')} to a collision, the baseline "
        f"{_plural(base_coll, 'episode')} to collisions and {_plural(base_to, 'episode')} to "
        f"timeouts. A planner that cannot answer inside the control loop is not safe, it is "
        f"just differently unsafe."
    )


def _finding_budget(hard: pd.DataFrame, hard_python: pd.DataFrame | None = None) -> str:
    """The hard-tier budget finding, worded from the numbers rather than around them.

    When both backends' hard tiers are on disk the finding is about the gap
    between them: same planner, same budget, same seeds, different runtime.
    """
    budget = float(load_config("eval")["hard_battery"]["planning_time_limit_ms"])
    if hard_python is None:
        return _one_sided_budget_finding(hard, budget)

    types = ("congested_hard", "head_on_narrow")
    fast = {name: _pct(hard, name, "spacetime_astar") for name in types}
    slow = {name: _pct(hard_python, name, "spacetime_astar") for name in types}
    slow_timeouts = {name: _counts(hard_python, name, "spacetime_astar")[1] for name in types}
    base = {name: _pct(hard, name, "baseline_replan") for name in types}
    base_collisions = {name: _counts(hard, name, "baseline_replan")[0] for name in types}

    gained = [name for name in types if fast[name] > slow[name] + 1e-9]
    lost = [name for name in types if fast[name] < slow[name] - 1e-9]
    if gained and not lost:
        headline = "the implementation decides the outcome"
    elif lost and not gained:
        headline = "the faster implementation does not help"
    elif gained and lost:
        headline = "the two implementations trade places"
    else:
        headline = "the implementation makes no difference"

    scores = " and ".join(f"{fast[name]:.1f}% of `{name}` episodes" for name in types)
    reference = " and ".join(
        f"{slow[name]:.1f}% of `{name}` with {_plural(slow_timeouts[name], 'timeout')}"
        for name in types
    )
    worst_base = min(types, key=lambda name: base[name])
    return (
        f"**Under a {budget:.0f} ms budget per replan, {headline}.** Space-time A\\* on the C++ "
        f"core reaches the goal in {scores}. The same planner on the Python reference, with the "
        f"same budget and the same seeds, manages {reference}: the search does not fit in the "
        f"budget, returns no plan, and the ego stalls in the aisle until the episode times out. "
        f"The replanning baseline is cheap enough either way and is unmoved at "
        f"{base['congested_hard']:.1f}% and {base['head_on_narrow']:.1f}%, but on "
        f"`{worst_base}` it fails by driving into vehicles "
        f"({_plural(base_collisions[worst_base], 'collision')}) rather than by running out of "
        f"time. Optimality is worthless if it does not fit in the control loop — and here "
        f"making it fit was an implementation problem, not an algorithmic one."
    )


def _finding_gaps(hard: pd.DataFrame) -> str:
    hybrid_cfg = load_config("hybrid")
    car = CarModel.from_config(hybrid_cfg)
    minimum = minimum_parallel_gap(car, hybrid_cfg)
    low, high = load_config("parking")["hard"]["parallel_minimal"]["gap_above_minimum"]
    cap = int(load_config("eval")["hard_battery"]["parking_max_expansions"])
    rows = hard[hard["parking_type"] == "parallel_minimal"]
    success = 100.0 * rows["success"].mean()
    perpendicular = 100.0 * hard[hard["parking_type"] == "perpendicular_minimal"]["success"].mean()
    return (
        f"**Parallel parking falls apart in minimal gaps.** The narrowest gap this collision "
        f"model accepts for a {car.length:.1f} m car is {minimum:.2f} m, computed from the "
        f"geometry rather than picked by hand. Given gaps of only {minimum + low:.2f}–"
        f"{minimum + high:.2f} m and a {cap}-expansion cap, `parallel_minimal` succeeds "
        f"{success:.1f}% of the time against {perpendicular:.1f}% for `perpendicular_minimal`, "
        f"which needs far less search."
    )


def build_regions() -> dict[str, str]:
    """Every generated region, keyed by marker name."""
    read = helpers.read_csv
    grid = read(results_path(GRID_CSV), "the step-1 benchmark")
    spacetime_normal = read(results_path(BATTERY_CSV), "the space-time battery")
    spacetime_hard = read(results_path(HARD_BATTERY_CSV), "the hard space-time battery")
    parking_normal = read(results_path(PARKING_CSV), "the parking battery")
    parking_hard = read(results_path(HARD_PARKING_CSV), "the hard parking battery")
    cpp_path = results_path(CPP_CSV)
    python_hard_path = results_path(tier_csv("hard", "python"))
    spacetime_hard_python = (
        pd.read_csv(python_hard_path) if python_hard_path.is_file() else None
    )

    regions = {
        "grid_table": _grid_table(grid),
        "spacetime_normal": _spacetime_table(spacetime_normal),
        "spacetime_hard": _spacetime_table(spacetime_hard),
        "spacetime_hard_python": (
            _spacetime_table(spacetime_hard_python) if spacetime_hard_python is not None
            else "_Not run. `python3 scripts/run_battery.py --tier hard --backend python` "
                 "fills this table._"
        ),
        "parking_normal": _parking_table(parking_normal),
        "parking_hard": _parking_table(parking_hard),
        "finding_budget": _finding_budget(spacetime_hard, spacetime_hard_python),
        "finding_gaps": _finding_gaps(parking_hard),
    }
    if cpp_path.is_file():
        regions["cpp_table"] = _cpp_table(pd.read_csv(cpp_path))
    else:
        regions["cpp_table"] = (
            "_The optional C++ core was not built in this run; run `make report` after "
            "`pip install -e .` to fill this table._"
        )
    assert set(regions) == set(REGION_NAMES), "build_regions() drifted from REGION_NAMES"
    return regions


def fill(text: str, regions: dict[str, str]) -> str:
    """Replace the body of each generated region, leaving the prose untouched."""
    for name, body in regions.items():
        begin = MARKER.format("BEGIN", name)
        end = MARKER.format("END", name)
        pattern = re.compile(f"{re.escape(begin)}.*?{re.escape(end)}", re.DOTALL)
        if not pattern.search(text):
            raise KeyError(f"README.md has no generated region named {name!r}")
        replacement = f"{begin}\n{body}\n{end}"
        text = pattern.sub(lambda _match, value=replacement: value, text)
    return text




def markers_in(text: str) -> list[str]:
    """Names of the generated regions a document declares, in order."""
    return re.findall(r"<!-- BEGIN GENERATED: ([a-z_]+) -->", text)


def update_readme(path: Path | None = None) -> bool:
    """Rewrite README.md's generated regions. Returns True if anything changed."""
    target = Path(path) if path is not None else README_PATH
    original = target.read_text(encoding="utf-8")
    updated = fill(original, build_regions())
    if updated != original:
        target.write_text(updated, encoding="utf-8")
        return True
    return False
