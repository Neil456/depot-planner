"""Top-down rendering of depot maps, searches and plans."""

from __future__ import annotations

from pathlib import Path
from typing import Iterable, Sequence

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.colors import ListedColormap

from depot_planner.grid_astar.search import SearchResult
from depot_planner.world.grid import Grid

Cell = tuple[int, int]

# aisle, parking, obstacle
MAP_COLORS = ("#e9edf2", "#cfd8e3", "#4a5568")
MAP_CMAP = ListedColormap(MAP_COLORS)

EGO_COLOR = "#1b6ac9"
AGENT_COLORS = ("#e07a5f", "#8e44ad", "#c0392b", "#16a085", "#d68910", "#0e7c9b")


def draw_map(ax, grid: Grid) -> None:
    """Draw the static map as a background image."""
    ax.imshow(
        grid.cells,
        cmap=MAP_CMAP,
        vmin=0,
        vmax=2,
        origin="upper",
        interpolation="nearest",
        extent=(-0.5, grid.width - 0.5, grid.height - 0.5, -0.5),
    )
    ax.set_xlim(-0.5, grid.width - 0.5)
    ax.set_ylim(grid.height - 0.5, -0.5)
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_aspect("equal")


def draw_expanded(ax, grid: Grid, expanded: Iterable[Cell]) -> None:
    """Overlay the expanded cells as a faint heatmap."""
    cells = list(expanded)
    if not cells:
        return
    layer = np.zeros(grid.shape, dtype=float)
    for x, y in cells:
        layer[y, x] = 1.0
    ax.imshow(
        np.ma.masked_where(layer == 0.0, layer),
        cmap="YlOrRd",
        vmin=0.0,
        vmax=1.6,
        alpha=0.45,
        origin="upper",
        interpolation="nearest",
        extent=(-0.5, grid.width - 0.5, grid.height - 0.5, -0.5),
    )


def draw_path(ax, path: Sequence[Cell] | None, color: str = EGO_COLOR, label: str | None = None,
              linewidth: float = 2.4, alpha: float = 1.0) -> None:
    """Draw a cell path as a bold polyline."""
    if not path:
        return
    xs = [c[0] for c in path]
    ys = [c[1] for c in path]
    ax.plot(xs, ys, color=color, linewidth=linewidth, alpha=alpha, solid_capstyle="round", label=label)


def draw_endpoints(ax, start: Cell | None, goal: Cell | None) -> None:
    """Mark start (circle) and goal (star)."""
    if start is not None:
        ax.plot(start[0], start[1], "o", color="#1b7f3b", markersize=9, markeredgecolor="white",
                markeredgewidth=1.2, zorder=5)
    if goal is not None:
        ax.plot(goal[0], goal[1], "*", color="#b3261e", markersize=16, markeredgecolor="white",
                markeredgewidth=1.0, zorder=5)


def render_search(
    grid: Grid,
    result: SearchResult,
    start: Cell,
    goal: Cell,
    title: str = "",
    ax=None,
    path_color: str = EGO_COLOR,
):
    """Draw one search: map, expanded heatmap, path, endpoints."""
    created = ax is None
    if created:
        _, ax = plt.subplots(figsize=(7.0, 5.0))
    draw_map(ax, grid)
    draw_expanded(ax, grid, result.expanded)
    draw_path(ax, result.path, color=path_color)
    draw_endpoints(ax, start, goal)
    if title:
        ax.set_title(title, fontsize=10)
    return ax


def save_comparison(
    grid: Grid,
    results: Sequence[tuple[str, SearchResult]],
    start: Cell,
    goal: Cell,
    path: Path | str,
    suptitle: str = "",
) -> Path:
    """Save a side-by-side PNG comparing several searches on one problem."""
    n = len(results)
    fig, axes = plt.subplots(1, n, figsize=(6.0 * n, 4.6))
    if n == 1:
        axes = [axes]
    for ax, (name, result) in zip(axes, results):
        if result.success:
            subtitle = (
                f"{name}\ncost {result.cost:.2f} | expanded {result.nodes_expanded} | "
                f"{result.runtime_ms:.1f} ms"
            )
        else:
            subtitle = f"{name}\nno path ({result.reason})"
        render_search(grid, result, start, goal, subtitle, ax=ax)
    if suptitle:
        fig.suptitle(suptitle, fontsize=12)
        fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.94))
    else:
        fig.tight_layout()
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=130)
    plt.close(fig)
    return out


def save_map_png(grid: Grid, path: Path | str, title: str = "") -> Path:
    """Save a plain picture of a map."""
    fig, ax = plt.subplots(figsize=(7.0, 5.0))
    draw_map(ax, grid)
    if title:
        ax.set_title(title, fontsize=10)
    fig.tight_layout()
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=130)
    plt.close(fig)
    return out


def draw_agents(ax, agents, t: int, alpha: float = 0.95) -> list:
    """Draw agent footprints at step ``t`` as coloured rectangles."""
    from matplotlib.patches import Rectangle

    patches = []
    for index, agent in enumerate(agents):
        x, y = agent.anchor_at(t)
        color = AGENT_COLORS[index % len(AGENT_COLORS)]
        patch = Rectangle(
            (x - 0.5, y - 0.5),
            agent.footprint.width,
            agent.footprint.height,
            facecolor=color,
            edgecolor="white",
            linewidth=0.8,
            alpha=alpha,
            zorder=4,
        )
        ax.add_patch(patch)
        patches.append(patch)
    return patches


def draw_ego(ax, cell, color: str = EGO_COLOR, size: float = 1.0):
    """Draw the ego vehicle as a highlighted square."""
    from matplotlib.patches import Rectangle

    patch = Rectangle(
        (cell[0] - size / 2.0, cell[1] - size / 2.0),
        size,
        size,
        facecolor=color,
        edgecolor="white",
        linewidth=1.2,
        zorder=6,
    )
    ax.add_patch(patch)
    return patch


def save_scenario_png(scenario, path, t: int = 0, title: str = ""):
    """Snapshot of a scenario at step ``t``: map, agents, ego, goal."""
    fig, ax = plt.subplots(figsize=(7.0, 5.0))
    draw_map(ax, scenario.grid)
    draw_path(ax, scenario.natural_route, color="#7f8c8d", linewidth=1.6, alpha=0.8)
    draw_agents(ax, scenario.agents, t)
    draw_endpoints(ax, scenario.start, scenario.goal)
    ax.set_title(title or f"{scenario.name} (seed {scenario.seed}) at t={t}", fontsize=10)
    fig.tight_layout()
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=130)
    plt.close(fig)
    return out


__all__ = [
    "AGENT_COLORS",
    "draw_agents",
    "draw_ego",
    "save_scenario_png",
    "EGO_COLOR",
    "draw_endpoints",
    "draw_expanded",
    "draw_map",
    "draw_path",
    "render_search",
    "save_comparison",
    "save_map_png",
]
