"""The collision model the planners reason with.

This is the *planner side* of collision checking: agent footprints grown by a
safety margin. The independent checker in :mod:`depot_planner.sim.collision`
uses the true footprints and shares no code with this module.
"""

from __future__ import annotations

from typing import Sequence

from depot_planner.world.agents import Agent

Cell = tuple[int, int]


class AgentOccupancy:
    """Cached lookups of what the agents cover, and when."""

    def __init__(self, agents: Sequence[Agent], inflate: int = 1) -> None:
        self.agents = list(agents)
        self.inflate = int(inflate)
        self._body: dict[int, frozenset[Cell]] = {}
        self._margin: dict[int, frozenset[Cell]] = {}

    def body(self, t: int) -> frozenset[Cell]:
        """Cells actually covered by agent bodies at step ``t``."""
        cached = self._body.get(t)
        if cached is None:
            cached = frozenset().union(*(a.cells_at(t) for a in self.agents)) if self.agents else frozenset()
            self._body[t] = cached
        return cached

    def margin(self, t: int) -> frozenset[Cell]:
        """Agent bodies grown by the safety margin at step ``t``."""
        cached = self._margin.get(t)
        if cached is None:
            if not self.agents:
                cached = frozenset()
            else:
                cached = frozenset().union(
                    *(a.occupied_at(t, self.inflate) for a in self.agents)
                )
            self._margin[t] = cached
        return cached

    def swaps(self, source: Cell, target: Cell, t: int) -> bool:
        """True if the ego and an agent would trade places between ``t`` and ``t+1``."""
        for agent in self.agents:
            if target in agent.cells_at(t) and source in agent.cells_at(t + 1):
                return True
        return False
