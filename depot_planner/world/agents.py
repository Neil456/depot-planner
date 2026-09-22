"""Other vehicles in the depot.

An agent occupies a rectangular footprint of cells and follows a **known,
precomputed** timed path: it never reacts to the ego vehicle. One simulation
step is ``dt`` seconds and an agent moves at most one cell per step, or pauses.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Iterable, Sequence

import numpy as np

from depot_planner.config import load_config
from depot_planner.world.grid import CellType, Grid

Cell = tuple[int, int]


@dataclass(frozen=True)
class Footprint:
    """A ``width`` x ``height`` block of cells anchored at its lower-index corner."""

    width: int = 2
    height: int = 2

    @property
    def offsets(self) -> tuple[Cell, ...]:
        return tuple((i, j) for i in range(self.width) for j in range(self.height))

    def cells(self, anchor: Cell) -> frozenset[Cell]:
        ax, ay = anchor
        return frozenset((ax + i, ay + j) for i, j in self.offsets)


def footprint_mask(grid: Grid, footprint: Footprint, aisles_only: bool = False) -> np.ndarray:
    """Anchors where the whole footprint fits on drivable (or aisle) cells."""
    base = (grid.cells == CellType.AISLE) if aisles_only else grid.drivable
    mask = np.ones(grid.shape, dtype=bool)
    height, width = grid.shape
    for i, j in footprint.offsets:
        shifted = np.zeros_like(mask)
        src = base[j:height, i:width] if (i or j) else base
        shifted[: height - j, : width - i] = src
        mask &= shifted
    return mask


@dataclass
class Agent:
    """One other vehicle with a fixed timeline of anchor cells."""

    agent_id: int
    timeline: np.ndarray                    # (T, 2) int array of anchor cells
    footprint: Footprint = field(default_factory=Footprint)
    kind: str = "traffic"
    _cache: dict[tuple[int, int], frozenset[Cell]] = field(
        default_factory=dict, repr=False, compare=False
    )

    def __post_init__(self) -> None:
        self.timeline = np.asarray(self.timeline, dtype=int).reshape(-1, 2)
        if len(self.timeline) == 0:
            raise ValueError("an agent needs at least one timeline entry")

    @property
    def horizon(self) -> int:
        """Last timestep the timeline explicitly covers."""
        return len(self.timeline) - 1

    def anchor_at(self, t: int) -> Cell:
        """Anchor cell at step ``t``; the agent stays put beyond its timeline."""
        index = min(max(int(t), 0), self.horizon)
        row = self.timeline[index]
        return int(row[0]), int(row[1])

    def cells_at(self, t: int) -> frozenset[Cell]:
        """Cells the agent body covers at step ``t``."""
        return self.occupied_at(t, inflate=0)

    def occupied_at(self, t: int, inflate: int = 0) -> frozenset[Cell]:
        """Body cells at step ``t``, grown by ``inflate`` cells in Chebyshev distance."""
        index = min(max(int(t), 0), self.horizon)
        key = (index, int(inflate))
        cached = self._cache.get(key)
        if cached is not None:
            return cached
        body = self.footprint.cells(self.anchor_at(index))
        if inflate <= 0:
            grown = body
        else:
            grown = frozenset(
                (x + dx, y + dy)
                for x, y in body
                for dx in range(-inflate, inflate + 1)
                for dy in range(-inflate, inflate + 1)
            )
        self._cache[key] = grown
        return grown

    def positions(self) -> np.ndarray:
        return self.timeline.copy()


def occupancy_at(agents: Iterable[Agent], t: int, inflate: int = 0) -> set[Cell]:
    """Union of the agents' (optionally inflated) footprints at step ``t``."""
    occupied: set[Cell] = set()
    for agent in agents:
        occupied |= agent.occupied_at(t, inflate)
    return occupied


def agents_overlap(a: Agent, b: Agent, horizon: int) -> bool:
    """True if two agents ever share a cell, or cross through each other."""
    for t in range(horizon + 1):
        a_now, b_now = a.cells_at(t), b.cells_at(t)
        if a_now & b_now:
            return True
        if t < horizon:
            a_next, b_next = a.cells_at(t + 1), b.cells_at(t + 1)
            if (a_next & b_now) and (a_now & b_next):
                return True
    return False


def agent_hits_obstacle(agent: Agent, grid: Grid) -> bool:
    """True if the agent ever covers a non-drivable or out-of-bounds cell."""
    for t in range(agent.horizon + 1):
        for x, y in agent.cells_at(t):
            if not grid.is_drivable(x, y):
                return True
    return False


def timed_path(
    route: Sequence[Cell],
    total_steps: int,
    rng: np.random.Generator,
    start_delay: int = 0,
    pause_probability: float = 0.0,
    hold_before_start: Cell | None = None,
) -> np.ndarray:
    """Turn a cell route into a ``(total_steps + 1, 2)`` timeline.

    The agent holds its starting cell for ``start_delay`` steps, then advances one
    cell per step (pausing at random with probability ``pause_probability``), then
    holds its final cell for the rest of the horizon.
    """
    if not route:
        raise ValueError("route must not be empty")
    hold = tuple(hold_before_start if hold_before_start is not None else route[0])
    anchors: list[Cell] = [hold] * max(int(start_delay), 0)
    for index, cell in enumerate(route):
        anchors.append((int(cell[0]), int(cell[1])))
        if index < len(route) - 1 and pause_probability > 0.0:
            while rng.random() < pause_probability:
                anchors.append((int(cell[0]), int(cell[1])))
                if len(anchors) > total_steps:
                    break
        if len(anchors) > total_steps:
            break
    if len(anchors) < total_steps + 1:
        anchors.extend([anchors[-1]] * (total_steps + 1 - len(anchors)))
    return np.asarray(anchors[: total_steps + 1], dtype=int)


def agent_config(config: dict[str, Any] | None = None) -> dict[str, Any]:
    cfg = config if config is not None else load_config("scenarios")
    return cfg["agent"]
