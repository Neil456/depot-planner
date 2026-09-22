#!/usr/bin/env python3
"""Fill the generated regions of README.md from the battery CSVs.

Run after `make battery`; `make report` runs it for you.
"""

from __future__ import annotations

from depot_planner.eval.readme import README_PATH, update_readme


def main() -> None:
    changed = update_readme()
    print(f"{'updated' if changed else 'unchanged'}: {README_PATH}")


if __name__ == "__main__":
    main()
