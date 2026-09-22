"""Loading of the YAML configuration files.

Every tunable number in the project lives in ``configs/*.yaml``; code only ever
reads it from here.
"""

from __future__ import annotations

import copy
from functools import lru_cache
from pathlib import Path
from typing import Any

import yaml

PACKAGE_ROOT = Path(__file__).resolve().parent
REPO_ROOT = PACKAGE_ROOT.parent
CONFIG_DIR = REPO_ROOT / "configs"
RESULTS_DIR = REPO_ROOT / "results"


@lru_cache(maxsize=None)
def _load_cached(name: str, config_dir: str) -> dict[str, Any]:
    path = Path(config_dir) / f"{name}.yaml"
    if not path.is_file():
        raise FileNotFoundError(f"no config file {path}")
    with path.open("r", encoding="utf-8") as handle:
        data = yaml.safe_load(handle)
    if not isinstance(data, dict):
        raise ValueError(f"config {path} must contain a mapping")
    return data


def load_config(name: str, config_dir: Path | str | None = None) -> dict[str, Any]:
    """Return the parsed contents of ``configs/<name>.yaml`` as a fresh dict."""
    directory = Path(config_dir) if config_dir is not None else CONFIG_DIR
    return copy.deepcopy(_load_cached(name, str(directory)))


def results_path(*parts: str) -> Path:
    """Path inside ``results/``, creating the parent directory on the way."""
    path = RESULTS_DIR.joinpath(*parts)
    path.parent.mkdir(parents=True, exist_ok=True)
    return path
