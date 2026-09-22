"""Polished animations for the README: one hero GIF and one side-by-side.

These share the project's palette with :mod:`depot_planner.viz.render` but lay
the frame out for a reader who has never seen the project: the ego is the only
saturated thing on screen, other vehicles are muted, the plan is drawn ahead of
the ego, and a legend names every mark.

The axes rectangle is fixed rather than tight-laid-out, so every frame comes out
at exactly the same pixel size — GIF writers require that.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Sequence

import imageio.v2 as imageio
import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D
from matplotlib.patches import Circle, Patch, Rectangle

from depot_planner.config import load_config
from depot_planner.sim.runner import EpisodeResult
from depot_planner.viz.render import draw_map
from depot_planner.world.scenarios import Scenario

Cell = tuple[int, int]

#: One palette for every showcase animation.
PALETTE = {
    "ego": "#1b6ac9",        # the only saturated blue on screen
    "agent": "#93a1b3",      # other vehicles, deliberately muted
    "plan": "#1b6ac9",
    "trail": "#c3cad4",
    "goal": "#b3261e",
    "yield": "#e08e0b",      # halo drawn while the ego is holding position
    "collision": "#b3261e",
}

PLANNER_LABELS = {
    "spacetime_astar": "Space-time A*",
    "baseline_replan": "Replanning baseline",
}

PARKING_LABELS = {
    "parallel": "parallel park",
    "perpendicular_forward": "forward into a bay",
    "perpendicular_reverse": "reverse into a bay",
    "tight": "minimal-clearance park",
    "parallel_minimal": "parallel park, minimal gap",
    "perpendicular_minimal": "bay park, minimal width",
}


def planner_label(name: str) -> str:
    """Human-readable planner name for a title."""
    return PLANNER_LABELS.get(name, name)


def parking_title(scenario) -> str:
    """Title for a parking animation, naming the manoeuvre rather than the type."""
    label = PARKING_LABELS.get(scenario.name, scenario.name)
    if scenario.layout != scenario.name:
        label = f"{label} via {PARKING_LABELS.get(scenario.layout, scenario.layout)}"
    return f"Hybrid A* — {label} (seed {scenario.seed})"


def legend_handles(include_yield: bool = True) -> list[Any]:
    """Proxy artists naming every mark the showcase draws."""
    handles: list[Any] = [
        Patch(facecolor=PALETTE["ego"], edgecolor="white", label="ego vehicle"),
        Patch(facecolor=PALETTE["agent"], edgecolor="white", label="other vehicles"),
        Line2D([], [], color=PALETTE["plan"], linewidth=2.4, label="planned path"),
        Line2D([], [], color=PALETTE["trail"], linewidth=2.0, label="path driven"),
        Line2D([], [], color=PALETTE["goal"], marker="*", linestyle="none",
               markersize=11, markeredgecolor="white", label="goal"),
    ]
    if include_yield:
        handles.append(
            Line2D([], [], color=PALETTE["yield"], marker="o", linestyle="none",
                   markerfacecolor="none", markeredgewidth=1.6, markersize=10,
                   label="ego yielding"),
        )
    return handles


def _plan_ahead(episode: EpisodeResult, index: int) -> list[Cell]:
    """The remaining part of the plan the ego is currently executing."""
    t = episode.times[index]
    current: list[Cell] = []
    for issued, cells in episode.plans:
        if issued <= t:
            current = cells
        else:
            break
    if not current:
        return []
    here = episode.cells[index]
    if here in current:
        return current[current.index(here):]
    return current


def _is_yielding(episode: EpisodeResult, index: int) -> bool:
    return index > 0 and episode.cells[index] == episode.cells[index - 1]


def _status(episode: EpisodeResult, index: int) -> str:
    if index == len(episode.cells) - 1:
        if episode.success:
            return "goal reached"
        return "collision" if episode.collision else "out of time"
    return "yielding to traffic" if _is_yielding(episode, index) else "driving"


def draw_episode_frame(
    ax,
    scenario: Scenario,
    episode: EpisodeResult,
    index: int,
    subtitle: str | None = None,
) -> None:
    """Draw one frame of an episode in the showcase style."""
    ax.clear()
    draw_map(ax, scenario.grid)
    t = episode.times[index]

    driven = episode.cells[: index + 1]
    if len(driven) > 1:
        ax.plot([c[0] for c in driven], [c[1] for c in driven], color=PALETTE["trail"],
                linewidth=2.0, alpha=0.95, solid_capstyle="round", zorder=3)

    ahead = _plan_ahead(episode, index)
    if len(ahead) > 1:
        ax.plot([c[0] for c in ahead], [c[1] for c in ahead], color=PALETTE["plan"],
                linewidth=2.4, alpha=0.85, solid_capstyle="round", zorder=4)

    for agent in scenario.agents:
        x, y = agent.anchor_at(t)
        ax.add_patch(Rectangle((x - 0.5, y - 0.5), agent.footprint.width, agent.footprint.height,
                               facecolor=PALETTE["agent"], edgecolor="white", linewidth=0.8,
                               zorder=5))

    ax.plot(scenario.goal[0], scenario.goal[1], "*", color=PALETTE["goal"], markersize=15,
            markeredgecolor="white", markeredgewidth=1.0, zorder=6)

    here = episode.cells[index]
    final_collision = episode.collision and index == len(episode.cells) - 1
    if _is_yielding(episode, index) and not final_collision:
        ax.add_patch(Circle(here, 2.1, facecolor="none", edgecolor=PALETTE["yield"],
                            linewidth=1.8, zorder=7))
    ax.add_patch(Rectangle((here[0] - 0.9, here[1] - 0.9), 1.8, 1.8, facecolor=PALETTE["ego"],
                           edgecolor="white", linewidth=1.2, zorder=8))

    if final_collision:
        ax.add_patch(Circle(here, 2.6, facecolor="none", edgecolor=PALETTE["collision"],
                            linewidth=2.4, zorder=9))

    if subtitle is not None:
        ax.set_title(subtitle, fontsize=9, color="#3c4552", pad=6)
    ax.text(0.015, 0.975, f"t = {t * scenario.dt:5.2f} s   {_status(episode, index)}",
            transform=ax.transAxes, ha="left", va="top", fontsize=8.5, color="#22272e",
            zorder=10,
            bbox=dict(boxstyle="round,pad=0.32", facecolor="white", alpha=0.82,
                      edgecolor="none"))


def _frame_indices(count: int, maximum: int) -> list[int]:
    stride = 1
    while len(range(0, count, stride)) > maximum:
        stride += 1
    indices = list(range(0, count, stride))
    if indices and indices[-1] != count - 1:
        indices.append(count - 1)
    return indices


def _capture(fig) -> np.ndarray:
    fig.canvas.draw()
    return np.asarray(fig.canvas.buffer_rgba())[..., :3].copy()


def _write_gif(path: Path | str, frames: Sequence[np.ndarray], fps: int) -> Path:
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    imageio.mimsave(out, list(frames), format="GIF", duration=1000.0 / fps, loop=0)
    return out


def hero_gif(
    scenario: Scenario,
    episode: EpisodeResult,
    path: Path | str,
    config: dict[str, Any] | None = None,
) -> Path:
    """Write the single-panel hero animation for the top of the README."""
    cfg = (config if config is not None else load_config("eval"))["showcase"]
    fps = int(cfg["fps"])
    width_px = int(cfg["width_px"])
    dpi = int(cfg["dpi"])
    width_in = width_px / dpi
    aspect = scenario.grid.height / scenario.grid.width
    height_in = width_in * aspect * 0.96 + 1.35

    fig = plt.figure(figsize=(width_in, height_in), dpi=dpi)
    ax = fig.add_axes((0.02, 0.135, 0.96, 0.775))
    fig.suptitle(
        f"{planner_label(episode.planner)} — {scenario.name} scenario (seed {scenario.seed})",
        fontsize=12, y=0.975, color="#22272e",
    )
    fig.legend(handles=legend_handles(), loc="lower center", ncol=6, fontsize=8,
               frameon=False, bbox_to_anchor=(0.5, 0.005))

    indices = _frame_indices(len(episode.cells), int(cfg["max_frames"]))
    frames = []
    for index in indices:
        draw_episode_frame(ax, scenario, episode, index)
        frames.append(_capture(fig))
    frames.extend([frames[-1]] * max(1, int(cfg["hold_frames"])))
    plt.close(fig)
    return _write_gif(path, frames, fps)


def side_by_side_gif(
    scenario: Scenario,
    episodes: Sequence[tuple[str, EpisodeResult]],
    path: Path | str,
    config: dict[str, Any] | None = None,
) -> Path:
    """Write a panel per planner, stepped in lockstep on one scenario.

    A panel whose episode has already ended holds its final frame, so the
    comparison stays aligned in time.
    """
    cfg = (config if config is not None else load_config("eval"))["showcase"]
    fps = int(cfg["fps"])
    dpi = int(cfg["dpi"])
    panels = len(episodes)
    width_in = int(cfg["side_by_side_width_px"]) / dpi
    panel_width = width_in / panels
    aspect = scenario.grid.height / scenario.grid.width
    height_in = panel_width * aspect * 0.96 + 1.5

    fig = plt.figure(figsize=(width_in, height_in), dpi=dpi)
    axes = []
    margin, gap = 0.015, 0.02
    span = (1.0 - 2.0 * margin - gap * (panels - 1)) / panels
    for column in range(panels):
        axes.append(fig.add_axes((margin + column * (span + gap), 0.14, span, 0.74)))
    fig.suptitle(f"Same depot, same traffic, same seed — {scenario.name} "
                 f"(seed {scenario.seed})", fontsize=12, y=0.975, color="#22272e")
    fig.legend(handles=legend_handles(), loc="lower center", ncol=6, fontsize=8,
               frameon=False, bbox_to_anchor=(0.5, 0.005))

    longest = max(len(episode.cells) for _, episode in episodes)
    indices = _frame_indices(longest, int(cfg["max_frames"]))
    frames = []
    for index in indices:
        for ax, (name, episode) in zip(axes, episodes):
            local = min(index, len(episode.cells) - 1)
            outcome = ("reaches the goal" if episode.success
                       else "collides" if episode.collision else "runs out of time")
            draw_episode_frame(ax, scenario, episode, local,
                               subtitle=f"{planner_label(name)} — {outcome}")
        frames.append(_capture(fig))
    frames.extend([frames[-1]] * max(1, int(cfg["hold_frames"])))
    plt.close(fig)
    return _write_gif(path, frames, fps)


def parking_legend_handles() -> list[Any]:
    """Proxy artists for the parking showcase."""
    from depot_planner.viz.parking import PARKED_COLOR

    return [
        Patch(facecolor=PALETTE["ego"], edgecolor="white", label="ego driving forward"),
        Patch(facecolor=PALETTE["yield"], edgecolor="white", label="ego reversing"),
        Patch(facecolor=PARKED_COLOR, edgecolor="white", label="parked cars"),
        Line2D([], [], color=PALETTE["ego"], linewidth=2.2, label="forward path"),
        Line2D([], [], color=PALETTE["yield"], linewidth=2.2, label="reverse path"),
        Line2D([], [], color=PALETTE["goal"], linewidth=1.8, label="goal pose"),
    ]


def parking_gif(
    scenario,
    result,
    path: Path | str,
    config: dict[str, Any] | None = None,
) -> Path:
    """Write the polished parking animation for the README.

    Forward and reverse motion are coloured differently, both in the trail and
    in the car body, so the direction switches read at a glance. The explored
    states appear as faint dots in the first frame.
    """
    from depot_planner.hybrid.car import CarModel
    from depot_planner.viz import parking as parking_viz

    cfg = (config if config is not None else load_config("eval"))["showcase"]
    fps = int(cfg["fps"])
    dpi = int(cfg["dpi"])
    width_in = int(cfg["parking_width_px"]) / dpi
    car = CarModel.from_config(scenario.hybrid_config)
    poses = result.poses or [scenario.start]
    directions = result.directions or [1]
    aspect = scenario.height_m / max(scenario.width_m, 1e-6)
    height_in = width_in * aspect * 0.96 + 1.5

    fig = plt.figure(figsize=(width_in, height_in), dpi=dpi)
    ax = fig.add_axes((0.02, 0.145, 0.96, 0.735))
    fig.suptitle(parking_title(scenario), fontsize=12, y=0.975, color="#22272e")
    fig.legend(handles=parking_legend_handles(), loc="lower center", ncol=6, fontsize=8,
               frameon=False, bbox_to_anchor=(0.5, 0.005))

    indices = _frame_indices(len(poses), int(cfg["max_frames"]))
    total = result.path_length_m
    frames = []
    for frame_number, index in enumerate(indices):
        ax.clear()
        parking_viz.setup_axes(ax, scenario)
        if frame_number == 0:
            parking_viz.draw_explored(ax, result.explored, alpha=0.5)
        for step in range(index):
            colour = PALETTE["ego"] if directions[step] >= 0 else PALETTE["yield"]
            ax.plot([poses[step][0], poses[step + 1][0]], [poses[step][1], poses[step + 1][1]],
                    color=colour, linewidth=2.2, solid_capstyle="round", zorder=4)
        parking_viz.draw_car(ax, car, scenario.goal, PALETTE["goal"], linewidth=1.6)
        gear = directions[min(index, len(directions) - 1)]
        colour = PALETTE["ego"] if gear >= 0 else PALETTE["yield"]
        parking_viz.draw_car(ax, car, poses[index], colour, alpha=0.95, linewidth=1.6,
                             fill=True, zorder=8)
        travelled = sum(
            math.hypot(b[0] - a[0], b[1] - a[1])
            for a, b in zip(poses[: index + 1], poses[1: index + 1])
        )
        label = "reversing" if gear < 0 else "driving forward"
        if index == len(poses) - 1:
            label = "parked"
        ax.text(0.01, 0.96, f"{label}   {travelled:4.1f} m of {total:4.1f} m   "
                            f"{result.direction_switches} direction changes",
                transform=ax.transAxes, ha="left", va="top", fontsize=8.5, color="#22272e",
                zorder=10,
                bbox=dict(boxstyle="round,pad=0.32", facecolor="white", alpha=0.85,
                          edgecolor="none"))
        frames.append(_capture(fig))
    frames.extend([frames[-1]] * max(1, int(cfg["hold_frames"])))
    plt.close(fig)
    return _write_gif(path, frames, fps)
