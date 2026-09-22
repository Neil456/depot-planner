"""Reeds-Shepp curves: shortest paths for a car that can drive forwards and back.

Used as the analytic expansion of hybrid A*: once the search gets near the goal,
a Reeds-Shepp curve connects the last state to the goal pose exactly, which is
what makes the 0.3 m / 5 degree goal tolerance reachable at all.

The word set covers CSC (``LSL``, ``LSR``), CCC (``LRL``) and SCS (``SLS``)
families, each expanded by the standard timeflip and reflect symmetries. That is
not the full 48-word Reeds-Shepp set, so the curve returned is a valid shortest
*candidate*, not provably the global optimum; every candidate is collision
checked and its endpoint verified before use.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Callable, Sequence

from depot_planner.hybrid.car import CarModel, Pose, wrap_angle

LEFT, STRAIGHT, RIGHT = 1, 0, -1


@dataclass(frozen=True)
class RSSegment:
    """One Reeds-Shepp segment: a steering direction and a signed length.

    ``length`` is in units of the turning radius; its sign is the gear
    (positive forward, negative reverse).
    """

    steering: int
    length: float


@dataclass(frozen=True)
class RSPath:
    """A sequence of Reeds-Shepp segments."""

    segments: tuple[RSSegment, ...]

    @property
    def length(self) -> float:
        """Total path length in units of the turning radius."""
        return sum(abs(segment.length) for segment in self.segments)

    @property
    def n_direction_changes(self) -> int:
        gears = [1 if s.length >= 0.0 else -1 for s in self.segments if abs(s.length) > 1e-9]
        return sum(1 for a, b in zip(gears, gears[1:]) if a != b)


def _polar(x: float, y: float) -> tuple[float, float]:
    return math.hypot(x, y), math.atan2(y, x)


def _mod2pi(theta: float) -> float:
    """Wrap to ``(-pi, pi]`` (the convention the word formulas below assume)."""
    value = math.fmod(theta, 2.0 * math.pi)
    if value < -math.pi:
        value += 2.0 * math.pi
    elif value > math.pi:
        value -= 2.0 * math.pi
    return value


# ------------------------------------------------------------- the base words


def _lsl(x: float, y: float, phi: float) -> tuple[float, float, float] | None:
    u, t = _polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
    if 0.0 <= t <= math.pi:
        v = _mod2pi(phi - t)
        if 0.0 <= v <= math.pi:
            return t, u, v
    return None


def _lsr(x: float, y: float, phi: float) -> tuple[float, float, float] | None:
    u1, t1 = _polar(x + math.sin(phi), y - 1.0 - math.cos(phi))
    squared = u1 * u1
    if squared >= 4.0:
        u = math.sqrt(squared - 4.0)
        theta = math.atan2(2.0, u)
        t = _mod2pi(t1 + theta)
        v = _mod2pi(t - phi)
        if t >= 0.0 and v >= 0.0:
            return t, u, v
    return None


def _lrl(x: float, y: float, phi: float) -> tuple[float, float, float] | None:
    u1, t1 = _polar(x - math.sin(phi), y - 1.0 + math.cos(phi))
    if u1 <= 4.0:
        u = -2.0 * math.asin(0.25 * u1)
        t = _mod2pi(t1 + 0.5 * u + math.pi)
        v = _mod2pi(phi - t + u)
        if t >= 0.0 and u <= 0.0:
            return t, u, v
    return None


def _sls(x: float, y: float, phi: float) -> tuple[float, float, float] | None:
    phi = _mod2pi(phi)
    if abs(y) < 1e-9 or not (1e-9 < abs(phi) < math.pi * 0.99):
        return None
    if phi < 0.0:
        return None
    xd = -y / math.tan(phi) + x
    t = xd - math.tan(phi / 2.0)
    u = phi
    v = math.hypot(x - xd, y) - math.tan(phi / 2.0)
    if y < 0.0:
        v = -math.hypot(x - xd, y) - math.tan(phi / 2.0)
    return t, u, v


def _mirror(word: Sequence[int]) -> tuple[int, ...]:
    return tuple(-s for s in word)


#: (base function, steering word) pairs; each is expanded by four symmetries.
_WORDS: tuple[tuple[Callable[..., tuple[float, float, float] | None], tuple[int, int, int]], ...] = (
    (_lsl, (LEFT, STRAIGHT, LEFT)),
    (_lsr, (LEFT, STRAIGHT, RIGHT)),
    (_lrl, (LEFT, RIGHT, LEFT)),
    (_sls, (STRAIGHT, LEFT, STRAIGHT)),
)


def _candidates(x: float, y: float, phi: float) -> list[RSPath]:
    """Every candidate curve for a goal at ``(x, y, phi)`` with unit turning radius."""
    paths: list[RSPath] = []

    def add(lengths: tuple[float, float, float], word: Sequence[int]) -> None:
        if any(not math.isfinite(value) for value in lengths):
            return
        segments = tuple(
            RSSegment(steering, length)
            for steering, length in zip(word, lengths)
            if abs(length) > 1e-9
        )
        if segments:
            paths.append(RSPath(segments))

    for base, word in _WORDS:
        mirrored = _mirror(word)
        result = base(x, y, phi)
        if result is not None:
            add(result, word)
        result = base(-x, y, -phi)                      # timeflip
        if result is not None:
            add(tuple(-value for value in result), word)
        result = base(x, -y, -phi)                      # reflect
        if result is not None:
            add(result, mirrored)
        result = base(-x, -y, phi)                      # timeflip + reflect
        if result is not None:
            add(tuple(-value for value in result), mirrored)
    return paths


def reeds_shepp_paths(start: Pose, goal: Pose, max_curvature: float) -> list[RSPath]:
    """All candidate Reeds-Shepp curves from ``start`` to ``goal``."""
    dx = goal[0] - start[0]
    dy = goal[1] - start[1]
    cos_t, sin_t = math.cos(start[2]), math.sin(start[2])
    x = (cos_t * dx + sin_t * dy) * max_curvature
    y = (-sin_t * dx + cos_t * dy) * max_curvature
    phi = wrap_angle(goal[2] - start[2])
    return _candidates(x, y, phi)


def shortest_reeds_shepp(start: Pose, goal: Pose, max_curvature: float) -> RSPath | None:
    """The shortest candidate curve, or ``None`` if no word applies."""
    paths = reeds_shepp_paths(start, goal, max_curvature)
    return min(paths, key=lambda p: p.length) if paths else None


def interpolate(
    start: Pose, path: RSPath, car: CarModel, step: float = 0.2
) -> tuple[list[Pose], list[int]]:
    """Sample a curve into poses and per-step gears (``+1`` forward, ``-1`` reverse).

    The first pose returned is ``start``; ``directions[i]`` is the gear used to
    get from ``poses[i]`` to ``poses[i + 1]``.
    """
    radius = car.min_turning_radius
    poses: list[Pose] = [start]
    directions: list[int] = []
    pose = start
    for segment in path.segments:
        distance_m = abs(segment.length) * radius
        gear = 1 if segment.length >= 0.0 else -1
        steer = segment.steering * car.max_steer
        substeps = max(1, int(math.ceil(distance_m / step)))
        piece = gear * distance_m / substeps
        for _ in range(substeps):
            pose = car.step(pose, steer, piece)
            poses.append(pose)
            directions.append(gear)
    return poses, directions


def path_length_m(path: RSPath, car: CarModel) -> float:
    return path.length * car.min_turning_radius
