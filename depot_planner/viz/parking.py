"""Rendering of parking scenarios and hybrid A* plans."""

from __future__ import annotations

from pathlib import Path
from typing import Iterable, Sequence

import matplotlib

matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon, Rectangle as MplRectangle

from depot_planner.hybrid.car import CarModel, Pose

OBSTACLE_COLOR = "#4a5568"
PARKED_COLOR = "#8b98ad"
FORWARD_COLOR = "#1b6ac9"
REVERSE_COLOR = "#e07a5f"
EXPLORED_COLOR = "#b0b8c4"
START_COLOR = "#1b7f3b"
GOAL_COLOR = "#b3261e"


def setup_axes(ax, scenario) -> None:
    """Draw the lot: walls, parked cars, and the axis frame."""
    ax.set_facecolor("#eef1f5")
    for index, (x0, y0, x1, y1) in enumerate(scenario.obstacles):
        color = OBSTACLE_COLOR if index < 4 else PARKED_COLOR
        ax.add_patch(MplRectangle((x0, y0), x1 - x0, y1 - y0, facecolor=color,
                                  edgecolor="white", linewidth=0.6, zorder=2))
    ax.set_xlim(0.0, scenario.width_m)
    ax.set_ylim(scenario.height_m, 0.0)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])


def draw_car(ax, car: CarModel, pose: Pose, color: str, alpha: float = 1.0,
             linewidth: float = 1.2, fill: bool = False, zorder: int = 5):
    """Outline (or filled outline) of the car footprint at ``pose``."""
    corners = car.corners(*pose)
    patch = Polygon(corners, closed=True, facecolor=color if fill else "none",
                    edgecolor=color, linewidth=linewidth, alpha=alpha, zorder=zorder)
    ax.add_patch(patch)
    # A short stub showing which way the nose points.
    nose = corners[1:3].mean(axis=0)
    axle = np.array(pose[:2])
    ax.plot([axle[0], nose[0]], [axle[1], nose[1]], color=color, linewidth=linewidth,
            alpha=alpha, zorder=zorder)
    return patch


def draw_explored(ax, poses: Iterable[Pose], alpha: float = 0.45) -> None:
    """Faint dots for the states the search expanded."""
    points = np.asarray([(p[0], p[1]) for p in poses], dtype=float)
    if len(points) == 0:
        return
    ax.scatter(points[:, 0], points[:, 1], s=1.6, c=EXPLORED_COLOR, alpha=alpha,
               linewidths=0.0, zorder=3)


def draw_path(ax, poses: Sequence[Pose], directions: Sequence[int], linewidth: float = 2.0) -> None:
    """Plan polyline, forward segments blue and reverse segments orange."""
    if len(poses) < 2:
        return
    xs = np.array([p[0] for p in poses])
    ys = np.array([p[1] for p in poses])
    for index in range(len(poses) - 1):
        color = FORWARD_COLOR if directions[index] >= 0 else REVERSE_COLOR
        ax.plot(xs[index:index + 2], ys[index:index + 2], color=color,
                linewidth=linewidth, solid_capstyle="round", zorder=4)


def save_scenario_png(scenario, path: Path | str, car: CarModel | None = None,
                      title: str = "") -> Path:
    """Picture of the lot with the start and goal poses."""
    model = car if car is not None else CarModel.from_config(scenario.hybrid_config)
    fig, ax = plt.subplots(figsize=(7.0, 7.0 * scenario.height_m / max(scenario.width_m, 1e-6)))
    setup_axes(ax, scenario)
    draw_car(ax, model, scenario.start, START_COLOR, linewidth=1.6)
    draw_car(ax, model, scenario.goal, GOAL_COLOR, linewidth=1.6)
    ax.set_title(title or f"{scenario.name} (seed {scenario.seed})", fontsize=10)
    fig.tight_layout()
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=130)
    plt.close(fig)
    return out


def save_plan_png(scenario, result, path: Path | str, car: CarModel | None = None,
                  title: str = "") -> Path:
    """Picture of a hybrid A* plan: explored states, path, start and goal."""
    model = car if car is not None else CarModel.from_config(scenario.hybrid_config)
    fig, ax = plt.subplots(figsize=(7.0, 7.0 * scenario.height_m / max(scenario.width_m, 1e-6)))
    setup_axes(ax, scenario)
    draw_explored(ax, result.explored)
    if result.poses:
        draw_path(ax, result.poses, result.directions)
        for index in range(0, len(result.poses), max(1, len(result.poses) // 12)):
            colour = FORWARD_COLOR if result.directions[min(index, len(result.directions) - 1)] >= 0 \
                else REVERSE_COLOR
            draw_car(ax, model, result.poses[index], colour, alpha=0.35, linewidth=0.9)
    draw_car(ax, model, scenario.start, START_COLOR, linewidth=1.6)
    draw_car(ax, model, scenario.goal, GOAL_COLOR, linewidth=1.6)
    ax.set_title(title or f"{scenario.name} (seed {scenario.seed})", fontsize=10)
    fig.tight_layout()
    out = Path(path)
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=130)
    plt.close(fig)
    return out
