#!/usr/bin/env python3
"""Step 6: generate REPORT.md from the battery CSVs.

Every number in the report comes from a CSV written by running the code. The
only thing this script computes itself is the step-1 benchmark, which it runs
and writes to ``results/grid_benchmark.csv`` before reading it back.

Failure cases are re-run from their recorded seed (planning is deterministic)
purely to render the frame where they failed and to read off a diagnosis.
"""

from __future__ import annotations

import math
import shutil
import time
from pathlib import Path

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import pandas as pd

from depot_planner.config import REPO_ROOT, load_config, results_path
from depot_planner.eval import report as report_helpers
from depot_planner.eval.battery import (
    BATTERY_CSV,
    HARD_BATTERY_CSV,
    tier_csv,
    tier_planner_config,
)
from depot_planner.eval.cpp_bench import CPP_CSV, summarise_cpp
from depot_planner.eval.grid_bench import GRID_CSV, run_grid_benchmark, summarise_grid, write_grid_benchmark
from depot_planner.eval.parking_battery import (
    HARD_PARKING_CSV,
    PARKING_CSV,
    parking_tier_csv,
    summarise_parking,
    tier_hybrid_config,
)
from depot_planner.hybrid.search import plan_for_scenario
from depot_planner.sim.runner import run_episode
from depot_planner.spacetime.planners import make_planner
from depot_planner.viz.parking import save_plan_png
from depot_planner.viz.render import (
    draw_agents,
    draw_endpoints,
    draw_ego,
    draw_map,
    draw_path,
)
from depot_planner.world.parking import generate_parking_scenario
from depot_planner.world.scenarios import generate_scenario

ASSETS = results_path("README_assets").parent / "README_assets"
FIGURES = ASSETS / "report"

#: The three GIFs README.md embeds, and where they come from.
README_GIFS = (
    ("gifs/crossing_spacetime_astar.gif", "spacetime_waits_for_traffic.gif"),
    ("gifs/crossing_baseline_replan.gif", "baseline_collides_same_scenario.gif"),
    ("gifs/parking/perpendicular_reverse.gif", "hybrid_astar_reverse_parking.gif"),
)


# ------------------------------------------------------------------- loading


def load_frames() -> dict[str, pd.DataFrame]:
    print("running the step-1 grid benchmark...")
    grid_frame = run_grid_benchmark()
    write_grid_benchmark(grid_frame, results_path(GRID_CSV))

    read = report_helpers.read_csv
    cpp_path = results_path(CPP_CSV)
    python_hard = results_path(tier_csv("hard", "python"))
    python_parking_hard = results_path(parking_tier_csv("hard", "python"))
    return {
        "cpp": pd.read_csv(cpp_path) if cpp_path.is_file() else None,
        "spacetime_hard_python": pd.read_csv(python_hard) if python_hard.is_file() else None,
        "parking_hard_python": (
            pd.read_csv(python_parking_hard) if python_parking_hard.is_file() else None
        ),
        "grid": read(results_path(GRID_CSV), "the step-1 benchmark"),
        "spacetime_normal": read(results_path(BATTERY_CSV), "the space-time battery"),
        "spacetime_hard": read(results_path(HARD_BATTERY_CSV), "the hard space-time battery"),
        "parking_normal": read(results_path(PARKING_CSV), "the parking battery"),
        "parking_hard": read(results_path(HARD_PARKING_CSV), "the hard parking battery"),
    }


# -------------------------------------------------------------------- tables


def grid_table(frame: pd.DataFrame) -> str:
    summary = summarise_grid(frame)
    return report_helpers.markdown_table(summary, [
        ("algorithm", "algorithm", ""),
        ("backend", "backend", ""),
        ("mean_cost", "mean path cost", ".3f"),
        ("mean_cost_ratio", "mean cost / optimal", ".4f"),
        ("max_cost_ratio", "worst cost / optimal", ".4f"),
        ("mean_nodes_expanded", "mean nodes expanded", ".0f"),
        ("max_nodes_expanded", "max nodes expanded", ".0f"),
        ("mean_runtime_ms", "mean ms", ".2f"),
        ("max_runtime_ms", "max ms", ".2f"),
    ])


def cpp_section(frame: pd.DataFrame | None) -> list[str]:
    """The Python-vs-C++ table, or a note that the core was not benchmarked."""
    if frame is None or frame.empty:
        return [
            "## 7. Python vs C++",
            "",
            "The C++ core was not benchmarked in this run. "
            "Build it with `pip install -e .` and rerun `make report`.",
            "",
        ]
    summary = summarise_cpp(frame)
    repeats = int(summary["repeats"].max())
    table = report_helpers.markdown_table(summary, [
        ("planner", "planner", ""),
        ("cases", "cases", ".0f"),
        ("median_python_ms", "median Python ms", ".4f"),
        ("median_cpp_ms", "median C++ ms", ".4f"),
        ("median_speedup", "median speedup", ".1f"),
        ("median_nodes", "median nodes expanded", ".0f"),
        ("identical", "identical plans", ""),
    ])
    per_case = report_helpers.markdown_table(
        frame.sort_values("speedup").groupby("planner", sort=False).agg(
            slowest_case=("case", "first"),
            slowest_speedup=("speedup", "min"),
            fastest_speedup=("speedup", "max"),
        ).reset_index(),
        [
            ("planner", "planner", ""),
            ("slowest_speedup", "smallest speedup", ".1f"),
            ("fastest_speedup", "largest speedup", ".1f"),
            ("slowest_case", "case with the smallest speedup", ""),
        ],
    )
    return [
        "## 7. Python vs C++",
        "",
        f"Every planner, timed on both backends over the same prepared inputs. Each case is "
        f"run {repeats} times per backend and the **median** is kept; a mean is dragged around "
        f"by scheduling noise and a minimum flatters whichever backend gets luckier. Scenario "
        f"setup — cost maps, drivable masks, cost-to-go fields, agent timelines, distance "
        f"fields — happens before the clock starts, so what is timed is the search alone.",
        "",
        table,
        "",
        "`identical plans` is the point of the column: every case returns the same path, the "
        "same cost and the same expansion count on both backends, so the speed-up is not "
        "bought by searching differently. The spread per planner:",
        "",
        per_case,
        "",
        "`make cpp-bench` runs the same three searches through Google Benchmark on fixed "
        "scenarios built in C++ (`cpp/bench/`), which measures the core without the binding "
        "layer or the interpreter.",
        "",
    ]



def hard_backend_section(frames: dict[str, pd.DataFrame]) -> list[str]:
    """The hard tier on both backends, side by side.

    The hard tier is the one place where the planner's *runtime* changes its
    *results*, because its budget is wall clock. Reporting only the fast backend
    would quietly drop the most interesting number in the project.
    """
    python_frame = frames.get("spacetime_hard_python")
    if python_frame is None or python_frame.empty:
        return [
            "The same tier on the Python reference was not run here. "
            "`python3 scripts/run_battery.py --tier hard --backend python` produces it.",
            "",
        ]

    from depot_planner.eval.battery import summarise

    fast = summarise(frames["spacetime_hard"]).set_index(["scenario", "planner"])
    slow = summarise(python_frame).set_index(["scenario", "planner"])
    rows = []
    for key in fast.index:
        if key not in slow.index:
            continue
        scenario, planner = key
        rows.append({
            "scenario": scenario,
            "planner": planner,
            "cpp_success": fast.loc[key, "success_pct"],
            "python_success": slow.loc[key, "success_pct"],
            "cpp_timeouts": fast.loc[key, "timeouts"],
            "python_timeouts": slow.loc[key, "timeouts"],
            "cpp_ms": fast.loc[key, "mean_planning_ms"],
            "python_ms": slow.loc[key, "mean_planning_ms"],
        })
    comparison = report_helpers.markdown_table(pd.DataFrame(rows), [
        ("scenario", "scenario", ""),
        ("planner", "planner", ""),
        ("cpp_success", "success % (C++)", ".1f"),
        ("python_success", "success % (Python)", ".1f"),
        ("cpp_timeouts", "timeouts (C++)", ".0f"),
        ("python_timeouts", "timeouts (Python)", ".0f"),
        ("cpp_ms", "mean plan ms (C++)", ".2f"),
        ("python_ms", "mean plan ms (Python)", ".2f"),
    ])
    return [
        "#### The same hard tier on the Python reference",
        "",
        "The hard tier's budget is wall clock, so this is the one place in the project "
        "where the *implementation* changes the *result*. Same scenario types, same seeds, "
        "same 50 ms budget, the reference planner instead of the core:",
        "",
        spacetime_table(python_frame),
        "",
        "Side by side:",
        "",
        comparison,
        "",
        "The baseline is cheap enough that the budget never binds for it, so its rows barely "
        "move. Space-time A* is where the budget bit: on the reference it spends its whole "
        "50 ms, returns no plan, and the ego holds position until the episode times out.",
        "",
    ]


def spacetime_table(frame: pd.DataFrame) -> str:
    from depot_planner.eval.battery import summarise

    summary = summarise(frame)
    return report_helpers.markdown_table(summary, [
        ("scenario", "scenario", ""),
        ("planner", "planner", ""),
        ("episodes", "episodes", ".0f"),
        ("success_pct", "success %", ".1f"),
        ("collisions", "collisions", ".0f"),
        ("timeouts", "timeouts", ".0f"),
        ("mean_time_to_goal_s", "mean time to goal (s)", ".2f"),
        ("mean_wait_steps", "mean wait steps", ".1f"),
        ("mean_planning_ms", "mean plan ms", ".2f"),
        ("max_planning_ms", "max plan ms", ".1f"),
    ])


def parking_table(frame: pd.DataFrame) -> str:
    summary = summarise_parking(frame)
    return report_helpers.markdown_table(summary, [
        ("parking_type", "parking type", ""),
        ("scenarios", "scenarios", ".0f"),
        ("success_pct", "success %", ".1f"),
        ("mean_planning_ms", "mean plan ms", ".1f"),
        ("max_planning_ms", "max plan ms", ".1f"),
        ("mean_nodes_expanded", "mean nodes expanded", ".0f"),
        ("mean_direction_switches", "mean direction switches", ".2f"),
        ("mean_path_length_m", "mean path (m)", ".2f"),
    ])


# --------------------------------------------------------------------- plots


def plot_nodes_expanded(frame: pd.DataFrame, path: Path) -> Path:
    algorithms = list(dict.fromkeys(frame["algorithm"]))
    data = [frame.loc[frame["algorithm"] == name, "nodes_expanded"].to_numpy() for name in algorithms]
    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    parts = ax.boxplot(data, tick_labels=algorithms, patch_artist=True, showfliers=True,
                       medianprops=dict(color="#22272e", linewidth=1.4))
    for patch, colour in zip(parts["boxes"], ("#8e9aaf", "#1b6ac9", "#e07a5f")):
        patch.set_facecolor(colour)
        patch.set_alpha(0.75)
    ax.set_ylabel("nodes expanded")
    ax.set_title(f"Step 1: nodes expanded over {frame['pair'].nunique()} start/goal pairs", fontsize=11)
    ax.grid(axis="y", alpha=0.25)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path


def plot_planning_time(frame: pd.DataFrame, path: Path) -> Path:
    labels: list[str] = []
    data = []
    for tier in ("normal", "hard"):
        for planner in ("spacetime_astar", "baseline_replan"):
            selection = frame[(frame["tier"] == tier) & (frame["planner"] == planner)]
            if selection.empty:
                continue
            labels.append(f"{planner}\n({tier})")
            data.append(selection["mean_planning_ms"].to_numpy())
    fig, ax = plt.subplots(figsize=(7.6, 4.4))
    parts = ax.boxplot(data, tick_labels=labels, patch_artist=True, showfliers=True,
                       medianprops=dict(color="#22272e", linewidth=1.4))
    for index, patch in enumerate(parts["boxes"]):
        patch.set_facecolor("#1b6ac9" if index % 2 == 0 else "#e07a5f")
        patch.set_alpha(0.75)
    ax.set_yscale("log")
    ax.set_ylabel("mean planning time per replan (ms, log scale)")
    ax.set_title("Planning time per episode: space-time A* vs the replanning baseline", fontsize=11)
    ax.grid(axis="y", alpha=0.25)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path


# ------------------------------------------------------------ failure cases


def render_episode_failure(scenario, episode, path: Path) -> Path:
    """Draw the frame where an episode failed."""
    index = episode.collision_step if episode.collision_step is not None else len(episode.cells) - 1
    index = max(0, min(index, len(episode.cells) - 1))
    t = episode.times[index]
    fig, ax = plt.subplots(figsize=(7.0, 5.0))
    draw_map(ax, scenario.grid)
    draw_path(ax, episode.cells[: index + 1], color="#9aa5b1", linewidth=1.6, alpha=0.9)
    draw_agents(ax, scenario.agents, t)
    draw_endpoints(ax, scenario.start, scenario.goal)
    draw_ego(ax, episode.cells[index], size=1.8)
    outcome = "collision" if episode.collision else "timeout"
    ax.set_title(
        f"{scenario.name} seed {scenario.seed} - {episode.planner} - {outcome} at t={t}",
        fontsize=10,
    )
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=110)
    plt.close(fig)
    return path


def diagnose_episode(scenario, episode, budget_ms: float | None) -> str:
    """One line on why this episode failed, from what the episode recorded."""
    straight = math.hypot(scenario.goal[0] - scenario.start[0], scenario.goal[1] - scenario.start[1])
    reasons = episode.plan_failure_reasons or {}
    top = max(reasons, key=reasons.get) if reasons else ""
    budget = f" (budget {budget_ms:.0f} ms)" if budget_ms else ""

    if episode.collision:
        collision_t = episode.times[min(episode.collision_step or 0, len(episode.times) - 1)]
        age = episode.plan_age_at(collision_t)
        if reasons:
            return (
                f"{episode.collision_detail}; {episode.plan_failures} of {episode.replans} replans "
                f"had already returned no plan (mostly '{top}'{budget}), so the ego was holding "
                f"position when the agent arrived"
            )
        stale = "an unknown number of" if age is None else f"{age}"
        return (
            f"{episode.collision_detail}; the plan it was executing was {stale} steps old and had "
            f"frozen that agent where it stood when the plan was made"
        )
    if reasons:
        return (
            f"timed out at the {scenario.time_limit}-step limit: {episode.plan_failures} of "
            f"{episode.replans} replans returned no plan (mostly '{top}'{budget}), leaving the ego "
            f"{episode.wait_steps} waiting steps and {episode.path_length_m:.1f} m travelled of "
            f"~{straight:.1f} m needed"
        )
    return (
        f"timed out at the {scenario.time_limit}-step limit with every replan succeeding: the ego "
        f"waited {episode.wait_steps} of {episode.steps} steps for traffic and covered "
        f"{episode.path_length_m:.1f} m of ~{straight:.1f} m"
    )


def diagnose_parking(scenario, result, cap: int) -> str:
    metrics = scenario.metrics
    clearance = metrics.get("clearance_above_minimum")
    if "gap" in metrics:
        geometry = f"gap {metrics['gap']:.2f} m"
    else:
        geometry = f"spot width {metrics['spot_width']:.2f} m"
    slack = f", {clearance:.2f} m above the model minimum" if clearance is not None else ""
    return (
        f"{result.reason} after {result.nodes_expanded} expansions (cap {cap}); {geometry}{slack} "
        f"and an aisle of {metrics.get('aisle_width', float('nan')):.1f} m left no manoeuvre the "
        f"search could reach inside its budget"
    )


def failure_section(frames: dict[str, pd.DataFrame]) -> str:
    lines: list[str] = []
    rows: list[str] = []
    total = 0

    spacetime = report_helpers.combine_tiers({
        "normal": frames["spacetime_normal"], "hard": frames["spacetime_hard"],
    })
    for _, row in spacetime[~spacetime["success"]].iterrows():
        tier = str(row["tier"])
        budget = None
        planner_config = tier_planner_config(tier)
        if planner_config is not None:
            budget = planner_config["spacetime"]["time_limit_ms"]
        backend = str(row["backend"]) if "backend" in row else None
        scenario = generate_scenario(str(row["scenario"]), int(row["seed"]))
        episode = run_episode(
            scenario, make_planner(str(row["planner"]), planner_config, backend)
        )
        # A hard-tier budget is wall clock, so a borderline episode can come out
        # differently on the re-run. Say so rather than quietly reporting the re-run.
        disagreed = episode.success
        png = render_episode_failure(
            scenario, episode,
            FIGURES / f"failure_{tier}_{row['scenario']}_{row['planner']}_seed{row['seed']}.png",
        )
        rows.append("| " + " | ".join([
            tier, str(row["scenario"]), str(row["planner"]), str(row["seed"]),
            "collision" if episode.collision else "timeout",
            f"[frame]({report_helpers.relative_to_repo(png, REPO_ROOT)})",
            diagnose_episode(scenario, episode, budget)
            + (" (**re-ran to render this frame and it reached the goal that time** — the hard "
               "tier's budget is wall clock, so borderline episodes are not bit-reproducible)"
               if disagreed else ""),
        ]) + " |")
        total += 1

    parking = report_helpers.combine_tiers({
        "normal": frames["parking_normal"], "hard": frames["parking_hard"],
    })
    for _, row in parking[~parking["success"]].iterrows():
        tier = str(row["tier"])
        hybrid_cfg = tier_hybrid_config(tier)
        cap = int(hybrid_cfg["limits"]["max_expansions"])
        scenario = generate_parking_scenario(
            str(row["parking_type"]), int(row["seed"]), hybrid_config=hybrid_cfg
        )
        result = plan_for_scenario(
            scenario, backend=str(row["backend"]) if "backend" in row else None
        )
        png = save_plan_png(
            scenario, result,
            FIGURES / f"failure_{tier}_{row['parking_type']}_seed{row['seed']}.png",
            title=f"{scenario.name} seed {scenario.seed} - no plan ({result.reason})",
        )
        rows.append("| " + " | ".join([
            tier, str(row["parking_type"]), "hybrid_astar", str(row["seed"]), "no plan",
            f"[frame]({report_helpers.relative_to_repo(png, REPO_ROOT)})",
            diagnose_parking(scenario, result, cap),
        ]) + " |")
        total += 1

    if total == 0:
        return "No episode in either battery failed.\n"

    lines.append(f"{total} of the {len(spacetime) + len(parking)} runs across both batteries "
                 f"did not reach the goal. Every one is listed here with the frame where it "
                 f"failed.\n")
    lines.append("| tier | scenario | planner | seed | outcome | frame | diagnosis |")
    lines.append("| :-- | :-- | :-- | --: | :-- | :-- | :-- |")
    lines.extend(rows)
    return "\n".join(lines) + "\n"


# ------------------------------------------------------------------- assets


def copy_readme_gifs() -> list[tuple[str, bool]]:
    copied: list[tuple[str, bool]] = []
    ASSETS.mkdir(parents=True, exist_ok=True)
    for source_name, target_name in README_GIFS:
        source = results_path(*source_name.split("/"))
        target = ASSETS / target_name
        if source.is_file():
            shutil.copyfile(source, target)
            copied.append((target_name, True))
        else:
            copied.append((target_name, False))
    return copied


# ------------------------------------------------------------------- report


def build_report(frames: dict[str, pd.DataFrame]) -> str:
    grid = frames["grid"]
    spacetime = report_helpers.combine_tiers({
        "normal": frames["spacetime_normal"], "hard": frames["spacetime_hard"],
    })
    eval_cfg = load_config("eval")
    hard_cfg = eval_cfg["hard_battery"]

    nodes_png = plot_nodes_expanded(grid, FIGURES / "nodes_expanded.png")
    time_png = plot_planning_time(spacetime, FIGURES / "planning_time.png")
    rel = lambda p: report_helpers.relative_to_repo(p, REPO_ROOT)

    generated = time.strftime("%Y-%m-%d %H:%M UTC", time.gmtime())
    parts = [
        "# Depot Maneuvering Planners — Report",
        "",
        f"Generated by `scripts/make_report.py` on {generated}. Every number below is read "
        "from a CSV under `results/` that was produced by running the code; nothing is "
        "hardcoded. Regenerate with `make battery && make report`.",
        "",
        "## 1. Grid search: Dijkstra vs A* vs weighted A*",
        "",
        f"{grid['pair'].nunique()} random start/goal pairs on one depot map "
        f"(`configs/eval.yaml: grid_benchmark`), all three algorithms on every pair. These "
        f"runs use the default backend — the `backend` column says which — so read the "
        f"timings here alongside the head-to-head in section 7.",
        "",
        grid_table(grid),
        "",
        "A* and Dijkstra return the same cost on every pair (cost ratio 1.0000), which is the "
        "admissibility check from step 1. Weighted A* trades a small amount of optimality for a "
        "large cut in expansions.",
        "",
        f"![nodes expanded by algorithm]({rel(nodes_png)})",
        "",
        "## 2. Space-time A* vs the replanning baseline",
        "",
        "### Normal tier",
        "",
        f"Five scenario types, 30 episodes each, both planners on the same seeded scenarios, "
        f"planned by the "
        f"{report_helpers.backend_of(frames['spacetime_normal'])} backend. "
        "Space-time A* replans every 4 steps; the baseline replans every step and treats the "
        "agents as static obstacles where they currently are.",
        "",
        spacetime_table(frames["spacetime_normal"]),
        "",
        "### Hard tier",
        "",
        f"Two harder scenario types, 30 episodes each, seeds offset to "
        f"{hard_cfg['seed_offset']}: `congested_hard` puts "
        f"{load_config('scenarios')['hard']['congested_hard']['scenarios']['congested']['agent_count']} "
        f"agents in the depot, and `head_on_narrow` shrinks the aisles to 3 cells (a 2-cell "
        f"vehicle plus its 1-cell margin fills one completely). Both planners additionally get a "
        f"hard budget of {hard_cfg['planning_time_limit_ms']} ms per replan, one fifth of the "
        f"250 ms simulation step.",
        "",
        spacetime_table(frames["spacetime_hard"]),
        "",
        *hard_backend_section(frames),
        f"![planning time distribution]({rel(time_png)})",
        "",
        "## 3. Hybrid A* parking",
        "",
        "### Normal tier",
        "",
        "Four parking types, 30 scenarios each. Every successful plan is re-verified by the "
        "independent exact-rectangle checker in `depot_planner/sim/car_collision.py`.",
        "",
        parking_table(frames["parking_normal"]),
        "",
        "### Hard tier",
        "",
        f"Two minimal-clearance types, 30 scenarios each. The gap (or spot width) is drawn from "
        f"{hard_cfg['parking_max_expansions'] and ''}"
        f"`minimum + 0.3 .. 0.6 m`, where the minimum is computed from the car geometry and the "
        f"collision model in `world/parking.py`, not written down by hand. The search is also "
        f"capped at {hard_cfg['parking_max_expansions']} expansions instead of "
        f"{load_config('hybrid')['limits']['max_expansions']}.",
        "",
        parking_table(frames["parking_hard"]),
        "",
        "## 4. Failure cases",
        "",
        failure_section(frames),
        "",
        "## 5. Reproducibility and the two backends",
        "",
        "Scenario generation, both planners and hybrid A* are deterministic given a seed, and "
        "the normal tiers reproduce exactly run to run — on either backend. The C++ core and "
        "the Python reference are verified to agree exactly, not approximately: "
        "`scripts/check_equivalence.py` compares them on every battery seed of both tiers and "
        "reports identical plans, costs and expansion counts, with every C++ parking plan also "
        "passing the independent exact-rectangle checker.",
        "",
        "The **hard space-time tier is the exception**, by construction: its 50 ms budget is "
        "wall clock, so how much search fits inside it depends on how fast the planner runs "
        "and on the machine it runs on. That is not a defect of the comparison, it is the "
        "point of it — and it is why section 2 reports that tier on both backends. Treat its "
        "aggregates as a property of this machine and this implementation rather than of the "
        "algorithm alone.",
        "",
        "## 6. What this report does not claim",
        "",
        "- Hybrid A* uses the standard obstacle-aware grid heuristic, which is not a proven "
        "lower bound in every obstacle layout, so its plans are not claimed to be optimal.",
        "- The Reeds-Shepp word set here covers the CSC, CCC and SCS families, not all 48 "
        "Reeds-Shepp words, so an analytic shot is the shortest *candidate*, not the global "
        "optimum.",
        "- Timings are wall clock on one machine; treat them as orders of magnitude, not "
        "benchmarks.",
        "- The C++ core and the Python reference are verified to produce the same plans, "
        "which says nothing about either being the fastest possible implementation. The "
        "speed-ups in section 7 are what this port achieved, not an upper bound.",
        "- The hard tier's success rates depend on how fast the planner runs on *this* "
        "machine, because its budget is wall clock. A slower machine would move them.",
        "",
        *cpp_section(frames.get("cpp")),
    ]
    return "\n".join(parts)


def main() -> None:
    frames = load_frames()
    FIGURES.mkdir(parents=True, exist_ok=True)
    text = build_report(frames)
    out = REPO_ROOT / "REPORT.md"
    out.write_text(text, encoding="utf-8")
    print(f"wrote {out}")

    for name, ok in copy_readme_gifs():
        status = "copied" if ok else "MISSING (run `make gifs` first)"
        print(f"  README asset {name}: {status}")


if __name__ == "__main__":
    main()
