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

from depot_planner.config import load_config
from depot_planner.sim.collision import CollisionReport, check_trajectory
from depot_planner.spacetime.planners import Planner
from depot_planner.world.scenarios import Scenario

Cell = tuple[int, int]


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
        }


def _summarise_timings(timings: list[float]) -> tuple[float, float]:
    if not timings:
        return 0.0, 0.0
    return sum(timings) / len(timings), max(timings)


def run_episode(
    scenario: Scenario,
    planner: Planner,
    config: dict[str, Any] | None = None,
) -> EpisodeResult:
    """Run ``planner`` on ``scenario`` in closed loop and return the metrics."""
    cfg = config if config is not None else load_config("sim")
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
                if not hold_on_failure:
                    return _finish(scenario, planner, cells, times, plans, timings, nodes,
                                   replans, plan_failures, None, "no plan and holding disabled")
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
            collision = report
            break

    return _finish(scenario, planner, cells, times, plans, timings, nodes, replans,
                   plan_failures, collision, "")


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
    )


def verify_episode(scenario: Scenario, episode: EpisodeResult) -> CollisionReport:
    """Re-check a finished episode's whole trajectory with the independent checker."""
    return check_trajectory(scenario.grid, scenario.agents, episode.cells, episode.times)
