"""Helpers for turning the battery CSVs into REPORT.md.

Nothing here computes a planner metric: every number comes from a CSV that was
written by actually running the code.
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Callable, Sequence

import pandas as pd


def format_value(value: Any, spec: str = "") -> str:
    """Render one cell, turning missing numbers into an em dash."""
    if value is None:
        return "—"
    if isinstance(value, float) and math.isnan(value):
        return "—"
    if isinstance(value, (bool,)):
        return "yes" if value else "no"
    if spec and isinstance(value, (int, float)):
        return format(value, spec)
    return str(value)


def markdown_table(
    frame: pd.DataFrame,
    columns: Sequence[tuple[str, str, str]],
    align: str = "left",
) -> str:
    """Render a DataFrame as a GitHub markdown table.

    ``columns`` is a sequence of ``(column_name, header, format_spec)``.
    """
    headers = [header for _, header, _ in columns]
    separator = [":--" if align == "left" else "--:"] * len(headers)
    lines = ["| " + " | ".join(headers) + " |", "| " + " | ".join(separator) + " |"]
    for _, row in frame.iterrows():
        cells = [format_value(row.get(name), spec) for name, _, spec in columns]
        lines.append("| " + " | ".join(cells) + " |")
    return "\n".join(lines)


def read_csv(path: Path, description: str) -> pd.DataFrame:
    if not Path(path).is_file():
        raise FileNotFoundError(
            f"{description} is missing ({path}). Run `make battery` before `make report`."
        )
    return pd.read_csv(path)


def relative_to_repo(path: Path, repo_root: Path) -> str:
    """Repo-relative POSIX path, for embedding in markdown."""
    return Path(path).resolve().relative_to(repo_root.resolve()).as_posix()


def combine_tiers(frames: dict[str, pd.DataFrame]) -> pd.DataFrame:
    """Concatenate per-tier frames, adding the tier column if a CSV lacks one."""
    parts = []
    for tier, frame in frames.items():
        part = frame.copy()
        if "tier" not in part.columns:
            part["tier"] = tier
        parts.append(part)
    return pd.concat(parts, ignore_index=True)


def summary_line(values: Sequence[float], formatter: Callable[[float], str] = lambda v: f"{v:.2f}") -> str:
    if len(values) == 0:
        return "—"
    return f"{formatter(float(min(values)))} – {formatter(float(max(values)))}"
