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
from depot_planner.eval.battery import BATTERY_CSV, HARD_BATTERY_CSV, summarise
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
    "finding_budget",
    "parking_normal",
    "parking_hard",
    "finding_gaps",
    "cpp_table",
)


def _grid_table(frame: pd.DataFrame) -> str:
    return helpers.markdown_table(summarise_grid(frame), [
        ("algorithm", "algorithm", ""),
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
        ("algorithm", "algorithm", ""),
        ("mean_python_ms", "Python ms", ".4f"),
        ("mean_cpp_ms", "C++ ms", ".4f"),
        ("mean_speedup", "speedup", ".1f"),
        ("identical_paths", "identical paths", ""),
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


def _finding_budget(hard: pd.DataFrame) -> str:
    """The hard-tier budget finding, worded from the numbers rather than around them."""
    budget = float(load_config("eval")["hard_battery"]["planning_time_limit_ms"])
    narrow_timed = _pct(hard, "head_on_narrow", "spacetime_astar")
    narrow_base = _pct(hard, "head_on_narrow", "baseline_replan")
    _, narrow_timeouts = _counts(hard, "head_on_narrow", "spacetime_astar")
    congested_timed = _pct(hard, "congested_hard", "spacetime_astar")
    congested_base = _pct(hard, "congested_hard", "baseline_replan")
    timed_coll, timed_to = _counts(hard, "congested_hard", "spacetime_astar")
    base_coll, base_to = _counts(hard, "congested_hard", "baseline_replan")

    lead = (
        "the cheap planner wins in narrow aisles" if narrow_base > narrow_timed
        else "the two planners separate"
    )
    if congested_timed > congested_base:
        congested_verdict = "space-time A* comes out ahead"
    elif congested_timed < congested_base:
        congested_verdict = "the baseline comes out ahead"
    else:
        congested_verdict = "the two tie on success rate"
    return (
        f"**Under a {budget:.0f} ms budget per replan, {lead}.** On `head_on_narrow` the "
        f"replanning baseline reaches the goal in {narrow_base:.1f}% of episodes while "
        f"space-time A* manages {narrow_timed:.1f}%, losing {narrow_timeouts} of them to "
        f"timeouts: the space-time search does not fit in the budget, returns no plan, and the "
        f"ego stalls in the aisle. On `congested_hard` {congested_verdict} "
        f"({congested_timed:.1f}% against {congested_base:.1f}%), but they fail in opposite "
        f"ways — space-time A* loses {_plural(timed_to, 'episode')} to timeouts and "
        f"{_plural(timed_coll, 'episode')} to a collision, the baseline "
        f"{_plural(base_coll, 'episode')} to collisions and {_plural(base_to, 'episode')} to "
        f"timeouts. A planner that cannot answer inside the control loop is not safe, it is "
        f"just differently unsafe."
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

    regions = {
        "grid_table": _grid_table(grid),
        "spacetime_normal": _spacetime_table(spacetime_normal),
        "spacetime_hard": _spacetime_table(spacetime_hard),
        "parking_normal": _parking_table(parking_normal),
        "parking_hard": _parking_table(parking_hard),
        "finding_budget": _finding_budget(spacetime_hard),
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
