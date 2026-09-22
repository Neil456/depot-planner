"""Independent collision checker.

This module deliberately shares no code with the planners. It takes an executed
ego trajectory and re-derives, from the grid and the agent timelines alone,
whether that trajectory was legal. If it disagrees with a planner, the planner
is wrong.

Checks performed at every step:

* the ego cell is inside the map and drivable;
* the ego cell is not covered by an agent footprint (true footprints, no margin);
* the ego and an agent do not swap cells between consecutive steps;
* the ego moves at most one cell in x and y per step;
* a diagonal ego move does not cut the corner of an obstacle.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

from depot_planner.world.agents import Agent
from depot_planner.world.grid import CellType, Grid

Cell = tuple[int, int]


@dataclass
class CollisionReport:
    """Verdict on one executed trajectory."""

    ok: bool
    kind: str = ""          # overlap | swap | obstacle | out_of_bounds | jump | corner_cut
    step: int | None = None
    time: int | None = None
    cell: Cell | None = None
    agent_id: int | None = None

    def describe(self) -> str:
        """One-line summary for logs and the report."""
        if self.ok:
            return "no collision"
        return f"{self.kind} at step {self.step} (t={self.time}) cell {self.cell} agent {self.agent_id}"


def _is_obstacle(grid: Grid, cell: Cell) -> bool:
    x, y = cell
    return grid.cells[y, x] == CellType.OBSTACLE


def check_trajectory(
    grid: Grid,
    agents: Sequence[Agent],
    cells: Sequence[Cell],
    times: Sequence[int] | None = None,
) -> CollisionReport:
    """Verify an executed ego trajectory. ``times[i]`` defaults to ``i``."""
    if not cells:
        return CollisionReport(True)
    steps = list(range(len(cells))) if times is None else [int(t) for t in times]
    if len(steps) != len(cells):
        raise ValueError("cells and times must have the same length")

    height, width = grid.shape
    for index, (cell, t) in enumerate(zip(cells, steps)):
        x, y = int(cell[0]), int(cell[1])
        if not (0 <= x < width and 0 <= y < height):
            return CollisionReport(False, "out_of_bounds", index, t, (x, y))
        if _is_obstacle(grid, (x, y)):
            return CollisionReport(False, "obstacle", index, t, (x, y))
        for agent in agents:
            if (x, y) in agent.cells_at(t):
                return CollisionReport(False, "overlap", index, t, (x, y), agent.agent_id)

    for index in range(len(cells) - 1):
        source = (int(cells[index][0]), int(cells[index][1]))
        target = (int(cells[index + 1][0]), int(cells[index + 1][1]))
        t0, t1 = steps[index], steps[index + 1]
        dx, dy = target[0] - source[0], target[1] - source[1]
        if abs(dx) > 1 or abs(dy) > 1 or t1 != t0 + 1:
            return CollisionReport(False, "jump", index, t0, target)
        if dx and dy:
            if _is_obstacle(grid, (target[0], source[1])) or _is_obstacle(grid, (source[0], target[1])):
                return CollisionReport(False, "corner_cut", index, t0, target)
        for agent in agents:
            if target in agent.cells_at(t0) and source in agent.cells_at(t1):
                return CollisionReport(False, "swap", index, t0, target, agent.agent_id)

    return CollisionReport(True)


def first_collision_step(
    grid: Grid, agents: Sequence[Agent], cells: Sequence[Cell], times: Sequence[int] | None = None
) -> int | None:
    """Index of the first illegal step, or ``None`` if the trajectory is clean."""
    report = check_trajectory(grid, agents, cells, times)
    return None if report.ok else report.step
