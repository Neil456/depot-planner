"""Closed-loop execution of a planner on a scenario.

Every ``replan_every`` steps the planner is asked for a plan from the ego's
current state; the ego then executes the next steps of that plan while the
agents follow their fixed timelines. The episode ends when the ego reaches the
goal, when the independent checker reports a collision, or at the time limit.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any

import numpy as np

from depot_planner import core
from depot_planner.config import load_config
from depot_planner.grid_astar import backend as _grid_backend
from depot_planner.sim.collision import CollisionReport, check_trajectory
from depot_planner.spacetime import search as st
from depot_planner.spacetime.planners import BaselineReplanPlanner, Planner, SpaceTimePlanner
from depot_planner.world.scenarios import Scenario

Cell = tuple[int, int]

#: Which implementation of the loop to run. The Python loop is the default and
#: the reference: it calls the independent collision checker on every executed
#: step, which is the project's strongest safety property. ``"cpp"`` runs the
#: whole episode in the core and is verified against it (see docs/DECISIONS.md).
DEFAULT_ENGINE = "python"


@dataclass
class EpisodeResult:
    """Everything the battery and the animations need from one episode."""

    scenario: str
    seed: int
    planner: str
    success: bool
    collision: bool
    timeout: bool
    steps: int
    time_to_goal_s: float
    path_length_m: float
    wait_steps: int
    nodes_expanded: int
    replans: int
    plan_failures: int
    mean_planning_ms: float
    max_planning_ms: float
    reason: str
    cells: list[Cell] = field(default_factory=list)
    times: list[int] = field(default_factory=list)
    plans: list[tuple[int, list[Cell]]] = field(default_factory=list)
    collision_step: int | None = None
    collision_detail: str = ""
    plan_failure_reasons: dict[str, int] = field(default_factory=dict)

    def plan_age_at(self, t: int) -> int | None:
        """How many steps old the plan being executed at step ``t`` was."""
        issued = [when for when, _ in self.plans if when <= t]
        return None if not issued else t - issued[-1]

    def as_row(self) -> dict[str, Any]:
        """Flat record for the battery CSV (no trajectories)."""
        return {
            "scenario": self.scenario,
            "seed": self.seed,
            "planner": self.planner,
            "success": self.success,
            "collision": self.collision,
            "timeout": self.timeout,
            "steps": self.steps,
            "time_to_goal_s": self.time_to_goal_s,
            "path_length_m": self.path_length_m,
            "wait_steps": self.wait_steps,
            "nodes_expanded": self.nodes_expanded,
            "replans": self.replans,
            "plan_failures": self.plan_failures,
            "mean_planning_ms": self.mean_planning_ms,
            "max_planning_ms": self.max_planning_ms,
            "collision_step": self.collision_step,
            "reason": self.reason,
            "plan_failure_reasons": "; ".join(
                f"{reason} x{count}" for reason, count in sorted(self.plan_failure_reasons.items())
            ),
        }


def _summarise_timings(timings: list[float]) -> tuple[float, float]:
    if not timings:
        return 0.0, 0.0
    return sum(timings) / len(timings), max(timings)


def run_episode(
    scenario: Scenario,
    planner: Planner,
    config: dict[str, Any] | None = None,
    engine: str | None = None,
) -> EpisodeResult:
    """Run ``planner`` on ``scenario`` in closed loop and return the metrics.

    ``engine`` selects the loop itself, not the planner: ``"python"`` (the
    default) runs this function and checks every executed step with the
    independent checker, ``"cpp"`` runs the whole episode in the core.
    """
    cfg = config if config is not None else load_config("sim")
    if core.resolve(engine if engine is not None else DEFAULT_ENGINE) == "cpp":
        return _run_episode_cpp(scenario, planner, cfg)
    replan_every = int(planner.replan_every or cfg["replan_every"])
    record_plans = bool(cfg["record_plans"])
    hold_on_failure = bool(cfg["hold_on_plan_failure"])

    grid = scenario.grid
    state: Cell = scenario.start
    t = 0

    cells: list[Cell] = [state]
    times: list[int] = [t]
    plans: list[tuple[int, list[Cell]]] = []
    timings: list[float] = []
    nodes = 0
    replans = 0
    plan_failures = 0
    failure_reasons: dict[str, int] = {}
    collision: CollisionReport | None = None

    plan_cells: list[Cell] = []
    plan_index = 0
    steps_since_replan = replan_every  # force a plan on the first iteration

    while t < scenario.time_limit and state != scenario.goal:
        need_plan = steps_since_replan >= replan_every or plan_index + 1 >= len(plan_cells)
        if need_plan:
            result = planner.plan(scenario, state, t)
            replans += 1
            nodes += result.nodes_expanded
            timings.append(result.runtime_ms)
            if result.success and len(result.cells) > 1:
                plan_cells = list(result.cells)
                plan_index = 0
                if record_plans:
                    plans.append((t, list(result.cells)))
            else:
                plan_failures += 1
                reason = result.reason or "no plan"
                failure_reasons[reason] = failure_reasons.get(reason, 0) + 1
                if not hold_on_failure:
                    return _finish(scenario, planner, cells, times, plans, timings, nodes,
                                   replans, plan_failures, None, "no plan and holding disabled",
                                   failure_reasons)
                plan_cells = [state, state]
                plan_index = 0
                if record_plans:
                    plans.append((t, [state]))
            steps_since_replan = 0

        plan_index += 1
        next_state = plan_cells[plan_index] if plan_index < len(plan_cells) else state
        t += 1
        state = next_state
        cells.append(state)
        times.append(t)
        steps_since_replan += 1

        report = check_trajectory(grid, scenario.agents, cells[-2:], times[-2:])
        if not report.ok:
            # The checker saw a two-step window; translate its index back to the
            # position in the full trajectory so the report renders the right frame.
            report.step = len(cells) - 2 + (report.step or 0)
            collision = report
            break

    return _finish(scenario, planner, cells, times, plans, timings, nodes, replans,
                   plan_failures, collision, "", failure_reasons)


def _run_episode_cpp(
    scenario: Scenario, planner: Planner, cfg: dict[str, Any]
) -> EpisodeResult:
    """Run the whole episode in the core and rebuild the same EpisodeResult.

    Only the two planners the core implements can drive it; anything else is an
    error rather than a silent fall back to the Python loop.
    """
    if planner.backend is not None and core.resolve(planner.backend) == "python":
        raise ValueError(
            "the C++ episode runner cannot drive a planner pinned to the Python backend"
        )
    if isinstance(planner, SpaceTimePlanner):
        kind = "spacetime"
    elif isinstance(planner, BaselineReplanPlanner):
        kind = "baseline"
    else:
        raise ValueError(
            f"the C++ episode runner does not implement the planner {planner.name!r}"
        )

    cached = planner.prepare(scenario)
    effective = _grid_backend.effective_cost_map(cached.cost_map, cached.drivable)
    spacetime_cfg = planner.config["spacetime"]
    baseline_cfg = planner.config["baseline"]

    field = None
    if kind == "spacetime" and str(spacetime_cfg["heuristic"]).lower() == "grid_dijkstra":
        field = cached.goal_field
        if field is None:
            field = st.goal_cost_field(effective, np.isfinite(effective), scenario.goal)
        field = np.ascontiguousarray(np.asarray(field, dtype=float))

    def budget(value: Any) -> float:
        return -1.0 if value is None else float(value)

    settings = {
        "replan_every": int(planner.replan_every or cfg["replan_every"]),
        "record_plans": bool(cfg["record_plans"]),
        "hold_on_plan_failure": bool(cfg["hold_on_plan_failure"]),
        "time_limit": int(scenario.time_limit),
        "planner": kind,
        "time_cost": float(spacetime_cfg["time_cost"]),
        "inflate": int(spacetime_cfg["inflate"] if kind == "spacetime" else baseline_cfg["inflate"]),
        "relax_margin_when_inside": bool(spacetime_cfg["relax_margin_when_inside"]),
        "heuristic_includes_time": bool(spacetime_cfg["heuristic_includes_time"]),
        "min_cell_cost": float(cached.min_cell_cost),
        # The space-time planner clamps its horizon to the episode's own limit.
        "horizon_cap": min(int(spacetime_cfg["max_time_horizon"]), int(scenario.time_limit)),
        "max_nodes": int(spacetime_cfg["max_nodes"]),
        "time_limit_ms": budget(spacetime_cfg.get("time_limit_ms")),
        "baseline_hold_on_failure": bool(baseline_cfg["hold_on_failure"]),
        "baseline_max_nodes": int(load_config("grid")["search"]["max_nodes"]),
        "baseline_time_limit_ms": budget(baseline_cfg.get("time_limit_ms")),
    }

    out = core.require().run_episode(
        effective,
        (int(scenario.start[0]), int(scenario.start[1])),
        (int(scenario.goal[0]), int(scenario.goal[1])),
        [
            (agent.timeline, int(agent.footprint.width), int(agent.footprint.height),
             int(agent.agent_id))
            for agent in scenario.agents
        ],
        field,
        settings,
    )

    collision = None
    if out["collision"]:
        collision = CollisionReport(
            False, str(out["collision_kind"]), int(out["collision_step"]),
            int(out["collision_time"]),
            tuple(int(v) for v in out["collision_cell"]),
            None if out["collision_agent"] is None else int(out["collision_agent"]),
        )

    cells = [(int(x), int(y)) for x, y in out["cells"]]
    reason_override = str(out["reason_override"])
    if reason_override:
        reason = reason_override
    elif collision is not None:
        reason = collision.describe()
    elif out["success"]:
        reason = "goal reached"
    else:
        reason = "time limit reached"

    steps = int(out["steps"])
    return EpisodeResult(
        scenario=scenario.name,
        seed=scenario.seed,
        planner=planner.name,
        success=bool(out["success"]),
        collision=bool(out["collision"]),
        timeout=bool(out["timeout"]),
        steps=steps,
        time_to_goal_s=steps * scenario.dt if out["success"] else float("nan"),
        path_length_m=float(out["path_length_cells"]) * scenario.grid.resolution,
        wait_steps=int(out["wait_steps"]),
        nodes_expanded=int(out["nodes_expanded"]),
        replans=int(out["replans"]),
        plan_failures=int(out["plan_failures"]),
        mean_planning_ms=float(out["mean_planning_ms"]),
        max_planning_ms=float(out["max_planning_ms"]),
        reason=reason,
        cells=cells,
        times=[int(t) for t in out["times"]],
        plans=[(int(when), [(int(x), int(y)) for x, y in plan]) for when, plan in out["plans"]],
        collision_step=None if collision is None else collision.step,
        collision_detail="" if collision is None else collision.describe(),
        plan_failure_reasons={str(name): int(count) for name, count in
                              out["plan_failure_reasons"]},
    )


def _finish(
    scenario: Scenario,
    planner: Planner,
    cells: list[Cell],
    times: list[int],
    plans: list[tuple[int, list[Cell]]],
    timings: list[float],
    nodes: int,
    replans: int,
    plan_failures: int,
    collision: CollisionReport | None,
    reason_override: str,
    failure_reasons: dict[str, int] | None = None,
) -> EpisodeResult:
    resolution = scenario.grid.resolution
    steps = len(cells) - 1
    length = sum(
        math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(cells, cells[1:])
    ) * resolution
    waits = sum(1 for a, b in zip(cells, cells[1:]) if a == b)
    mean_ms, max_ms = _summarise_timings(timings)

    collided = collision is not None
    success = (not collided) and cells[-1] == scenario.goal
    timeout = (not collided) and not success
    if reason_override:
        reason = reason_override
    elif collided:
        reason = collision.describe()
    elif success:
        reason = "goal reached"
    else:
        reason = "time limit reached"

    return EpisodeResult(
        scenario=scenario.name,
        seed=scenario.seed,
        planner=planner.name,
        success=success,
        collision=collided,
        timeout=timeout,
        steps=steps,
        time_to_goal_s=steps * scenario.dt if success else float("nan"),
        path_length_m=length,
        wait_steps=waits,
        nodes_expanded=nodes,
        replans=replans,
        plan_failures=plan_failures,
        mean_planning_ms=mean_ms,
        max_planning_ms=max_ms,
        reason=reason,
        cells=cells,
        times=times,
        plans=plans,
        collision_step=None if collision is None else collision.step,
        collision_detail="" if collision is None else collision.describe(),
        plan_failure_reasons=dict(failure_reasons or {}),
    )


def verify_episode(scenario: Scenario, episode: EpisodeResult) -> CollisionReport:
    """Re-check a finished episode's whole trajectory with the independent checker."""
    return check_trajectory(scenario.grid, scenario.agents, episode.cells, episode.times)
