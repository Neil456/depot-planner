"""Checks that keep the documentation honest and its links intact."""

from __future__ import annotations

import re
from pathlib import Path

import pytest

from depot_planner.config import REPO_ROOT, results_path
from depot_planner.eval.battery import BATTERY_CSV
from depot_planner.eval.readme import MARKER, REGION_NAMES, fill, markers_in

README = REPO_ROOT / "README.md"
DOCS = REPO_ROOT / "docs"

#: Markdown link/image targets are checked against these, with anchors stripped.
LINK_PATTERN = re.compile(r"!?\[[^\]]*\]\(([^)\s]+)\)")

needs_battery = pytest.mark.skipif(
    not results_path(BATTERY_CSV).is_file(),
    reason="battery CSVs are not present; run `make battery`",
)


def _readme() -> str:
    return README.read_text(encoding="utf-8")


# ------------------------------------------------------------------ the files


def test_the_project_documents_all_exist():
    assert README.is_file()
    assert (REPO_ROOT / "REPORT.md").is_file()
    assert (REPO_ROOT / "LICENSE").is_file()
    for name in ("TASK.md", "PROGRESS.md", "DECISIONS.md"):
        assert (DOCS / name).is_file(), f"docs/{name} is missing"


def test_the_licence_is_mit_and_names_a_holder():
    text = (REPO_ROOT / "LICENSE").read_text(encoding="utf-8")
    assert "MIT License" in text
    assert re.search(r"Copyright \(c\) \d{4} \S", text), "no copyright holder"


def test_no_document_still_points_at_the_old_top_level_paths():
    """TASK/PROGRESS/DECISIONS moved into docs/; nothing may link to the old spots."""
    for path in [README, REPO_ROOT / "REPORT.md", REPO_ROOT / "CLAUDE.md"]:
        text = path.read_text(encoding="utf-8")
        for name in ("TASK.md", "PROGRESS.md", "DECISIONS.md"):
            for match in re.finditer(re.escape(name), text):
                prefix = text[max(0, match.start() - 5):match.start()]
                assert prefix.endswith("docs/"), f"{path.name} references bare {name}"


# --------------------------------------------------------------- the links


def _broken_links(text: str) -> list[str]:
    broken = []
    for target in LINK_PATTERN.findall(text):
        if target.startswith(("http://", "https://", "#", "mailto:")):
            continue
        if not (REPO_ROOT / target.split("#")[0]).exists():
            broken.append(target)
    return broken


def test_every_local_readme_link_resolves():
    missing = _broken_links(_readme())
    assert not missing, f"README links to missing files: {missing}"


@needs_battery
def test_every_local_report_link_resolves():
    """REPORT.md embeds a frame per failure; none of those images may be missing."""
    report = (REPO_ROOT / "REPORT.md").read_text(encoding="utf-8")
    missing = _broken_links(report)
    assert not missing, f"REPORT.md links to missing files: {missing}"


@needs_battery
def test_the_report_failure_frames_are_committed():
    report = (REPO_ROOT / "REPORT.md").read_text(encoding="utf-8")
    frames = [t for t in LINK_PATTERN.findall(report) if "/report/failure_" in t]
    assert frames, "the report lists no failure frames"
    for frame in frames:
        assert (REPO_ROOT / frame).is_file()


def test_the_readme_embeds_the_committed_showcase_gifs():
    text = _readme()
    for asset in ("results/README_assets/hero.gif", "results/README_assets/side_by_side.gif",
                  "results/README_assets/parking.gif"):
        assert asset in text, f"README does not embed {asset}"
        assert (REPO_ROOT / asset).is_file(), f"{asset} is not committed"


def test_the_readme_has_the_sections_a_reader_expects():
    text = _readme()
    for heading in ("## What this is", "## Planners", "## Results",
                    "## C++ core", "## Quickstart", "## Layout"):
        assert heading in text, f"README is missing the section {heading!r}"


def test_the_readme_shows_the_ci_badge():
    assert "workflows/tests.yml/badge.svg" in _readme()


# ------------------------------------------------- the generated regions


def test_the_readme_declares_exactly_the_generated_regions():
    assert markers_in(_readme()) == list(REGION_NAMES)


def test_every_generated_region_is_closed():
    text = _readme()
    for name in REGION_NAMES:
        assert text.count(MARKER.format("BEGIN", name)) == 1
        assert text.count(MARKER.format("END", name)) == 1


def test_fill_replaces_only_the_region_body():
    document = (
        "prose before\n"
        + MARKER.format("BEGIN", "grid_table") + "\nstale\n"
        + MARKER.format("END", "grid_table") + "\nprose after\n"
    )
    filled = fill(document, {"grid_table": "| a |\n| :-- |\n| 1 |"})
    assert "stale" not in filled
    assert filled.startswith("prose before\n")
    assert filled.endswith("prose after\n")
    assert fill(filled, {"grid_table": "| a |\n| :-- |\n| 1 |"}) == filled


def test_filling_an_unknown_region_is_an_error():
    with pytest.raises(KeyError):
        fill("nothing here", {"grid_table": "x"})


@needs_battery
def test_the_committed_readme_matches_what_the_generator_produces():
    """`make report` must leave README.md unchanged, so it can never go stale."""
    from depot_planner.eval.readme import build_regions

    text = _readme()
    assert fill(text, build_regions()) == text, (
        "README.md is out of date; run `python3 scripts/make_readme.py`"
    )
