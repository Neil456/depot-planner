"""Shared fixtures for the depot planner test suite."""

from __future__ import annotations

import numpy as np
import pytest

from depot_planner.world.depot_maps import generate_depot
from depot_planner.world.grid import Grid


@pytest.fixture(scope="session")
def depot() -> Grid:
    """A single deterministic depot map shared by the read-only tests."""
    return generate_depot(seed=0)


@pytest.fixture
def rng() -> np.random.Generator:
    return np.random.default_rng(12345)
