"""GIF animations of closed-loop episodes."""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Sequence

import imageio.v2 as imageio
import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np

from depot_planner.config import load_config
from depot_planner.sim.runner import EpisodeResult
from depot_planner.viz.render import EGO_COLOR, draw_agents, draw_ego, draw_map, draw_path
from depot_planner.world.scenarios import Scenario

PLAN_COLOR = "#1b6ac9"


def _frame_to_array(fig) -> np.ndarray:
    fig.canvas.draw()
    return np.asarray(fig.canvas.buffer_rgba())[..., :3].copy()


def _plan_at(plans: Sequence[tuple[int, list[tuple[int, int]]]], t: int) -> list[tuple[int, int]]:
    """The most recent plan issued at or before step ``t``."""
    current: list[tuple[int, int]] = []
    for issued, cells in plans:
        if issued <= t:
            current = cells
        else:
            break
    return current


def _frame_indices(count: int, stride: int, maximum: int) -> list[int]:
    stride = max(1, int(stride))
    while len(range(0, count, stride)) > maximum:
        stride += 1
    indices = list(range(0, count, stride))
    if indices and indices[-1] != count - 1:
        indices.append(count - 1)
    return indices


def animate_episode(
    scenario: Scenario,
    episode: EpisodeResult,
    path: Path | str,
    config: dict[str, Any] | None = None,
    title: str | None = None,
) -> Path:
    """Write a GIF of one episode: map, agents, ego, live plan and a text overlay."""
    cfg = (config if config is not None else load_config("eval"))["animation"]
    fps = int(cfg["fps"])
    figure_size = tuple(cfg["figure_size"])
    dpi = int(cfg["dpi"])
    indices = _frame_indices(len(episode.cells), int(cfg["step_stride"]), int(cfg["max_frames"]))

    fig, ax = plt.subplots(figsize=figure_size, dpi=dpi)
    frames: list[np.ndarray] = []
    label = title or f"{scenario.name} (seed {scenario.seed}) - {episode.planner}"

    for index in indices:
        t = episode.times[index]
        ax.clear()
        draw_map(ax, scenario.grid)
        plan = _plan_at(episode.plans, t)
        if plan:
            ahead = plan[plan.index(episode.cells[index]):] if episode.cells[index] in plan else plan
            draw_path(ax, ahead, color=PLAN_COLOR, linewidth=2.0, alpha=0.75)
        draw_agents(ax, scenario.agents, t)
        ax.plot(scenario.goal[0], scenario.goal[1], "*", color="#b3261e", markersize=13,
                markeredgecolor="white", markeredgewidth=0.8, zorder=5)
        draw_ego(ax, episode.cells[index], color=EGO_COLOR, size=1.6)
        ax.set_title(f"{label}\nt = {t * scenario.dt:5.2f} s   step {t}", fontsize=8)
        if index == indices[-1]:
            outcome = "GOAL" if episode.success else ("COLLISION" if episode.collision else "TIMEOUT")
            ax.text(
                0.5, 0.5, outcome, transform=ax.transAxes, ha="center", va="center",
                fontsize=22, fontweight="bold", zorder=10,
                color="#1b7f3b" if episode.success else "#b3261e",
                bbox=dict(boxstyle="round,pad=0.4", facecolor="white", alpha=0.8, edgecolor="none"),
            )
        fig.tight_layout()
        frames.append(_frame_to_array(fig))

    # Linger on the final frame so the outcome is readable.
    frames.extend([frames[-1]] * max(1, fps // 2))
    plt.close(fig)

    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    imageio.mimsave(out, frames, format="GIF", duration=1000.0 / fps, loop=0)
    return out


def gif_size_mb(path: Path | str) -> float:
    """Size of a written GIF in mebibytes."""
    return Path(path).stat().st_size / (1024 * 1024)


def animate_parking(
    scenario,
    result,
    path: Path | str,
    config: dict[str, Any] | None = None,
    title: str | None = None,
) -> Path:
    """GIF of the car executing a hybrid A* plan.

    The footprint is drawn along the path, forward segments in blue and reverse
    segments in orange, and the explored states appear as faint dots in the
    first frame.
    """
    from depot_planner.hybrid.car import CarModel
    from depot_planner.viz import parking as parking_viz

    cfg = (config if config is not None else load_config("eval"))["parking_animation"]
    fps = int(cfg["fps"])
    car = CarModel.from_config(scenario.hybrid_config)
    poses = result.poses or [scenario.start]
    directions = result.directions or [1]
    indices = _frame_indices(len(poses), 1, int(cfg["max_frames"]))

    aspect = scenario.height_m / max(scenario.width_m, 1e-6)
    width_in = float(cfg["figure_size"][0])
    fig, ax = plt.subplots(figsize=(width_in, max(2.4, width_in * aspect + 0.6)), dpi=int(cfg["dpi"]))
    frames: list[np.ndarray] = []
    label = title or f"{scenario.name} (seed {scenario.seed}) - hybrid A*"

    for frame_number, index in enumerate(indices):
        ax.clear()
        parking_viz.setup_axes(ax, scenario)
        if frame_number == 0:
            parking_viz.draw_explored(ax, result.explored)
        parking_viz.draw_path(ax, poses[: index + 1], directions[:index], linewidth=1.8)
        parking_viz.draw_car(ax, car, scenario.goal, parking_viz.GOAL_COLOR, linewidth=1.4)
        gear = directions[min(index, len(directions) - 1)]
        colour = parking_viz.FORWARD_COLOR if gear >= 0 else parking_viz.REVERSE_COLOR
        parking_viz.draw_car(ax, car, poses[index], colour, linewidth=1.8)
        travelled = sum(
            math.hypot(b[0] - a[0], b[1] - a[1])
            for a, b in zip(poses[: index + 1], poses[1: index + 1])
        )
        gear_text = "forward" if gear >= 0 else "reverse"
        ax.set_title(f"{label}\n{gear_text}   {travelled:5.1f} m of {result.path_length_m:5.1f} m",
                     fontsize=8)
        fig.tight_layout()
        frames.append(_frame_to_array(fig))

    frames.extend([frames[-1]] * max(1, fps))
    plt.close(fig)

    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    imageio.mimsave(out, frames, format="GIF", duration=1000.0 / fps, loop=0)
    return out
