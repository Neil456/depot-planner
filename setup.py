"""Builds the optional C++ core (step 7).

The extension is a speed-up, not a requirement: if no compiler or pybind11 is
available the build is skipped with a warning and
``depot_planner.grid_astar`` falls back to the pure-Python search.
"""

from __future__ import annotations

import sys

from setuptools import setup
from setuptools.command.build_ext import build_ext as _build_ext

try:
    from pybind11.setup_helpers import Pybind11Extension
    from pybind11.setup_helpers import build_ext as pybind11_build_ext

    EXTENSIONS = [
        Pybind11Extension(
            "depot_planner._cpp",
            ["cpp/grid_astar.cpp"],
            cxx_std=17,
        )
    ]
    BASE_BUILD_EXT = pybind11_build_ext
except ImportError:  # pragma: no cover - only hit without pybind11 installed
    EXTENSIONS = []
    BASE_BUILD_EXT = _build_ext


class OptionalBuildExt(BASE_BUILD_EXT):
    """Never fail the install because the C++ core would not compile."""

    def run(self) -> None:
        try:
            super().run()
        except Exception as error:  # pragma: no cover - environment dependent
            print(f"warning: skipping the optional C++ core ({error})", file=sys.stderr)

    def build_extension(self, ext) -> None:
        try:
            super().build_extension(ext)
        except Exception as error:  # pragma: no cover - environment dependent
            print(f"warning: skipping the optional C++ core ({error})", file=sys.stderr)


setup(ext_modules=EXTENSIONS, cmdclass={"build_ext": OptionalBuildExt})
