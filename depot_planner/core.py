"""Access to the compiled planning core (``depot_planner._cpp``).

The C++ library under ``cpp/`` is the production implementation of every
performance-critical planner. The pure-Python modules stay in the tree as the
reference implementation: they define the behaviour the core is tested against,
and they are what ``backend="python"`` runs.

Every entry point that has both implementations takes a ``backend`` argument:

``"cpp"``     the compiled core; an error if it was not built
``"python"``  the reference implementation
``"auto"``    the core when it is built and the call can be served by it,
              otherwise the reference
"""

from __future__ import annotations

from typing import Any

try:  # pragma: no cover - depends on whether the extension was built
    from depot_planner import _cpp
except ImportError:  # pragma: no cover
    _cpp = None

#: Every value ``backend=`` accepts.
BACKENDS: tuple[str, ...] = ("auto", "cpp", "python")

#: What ``backend=None`` means. ``"auto"`` is the C++ core whenever it is built,
#: which it always is after ``pip install -e .``, and the Python reference
#: otherwise, so an in-tree checkout without a build still runs.
DEFAULT_BACKEND = "auto"

_MISSING = (
    "the C++ core is not built; reinstall with `pip install -e .` or pass backend='python'"
)


def available() -> bool:
    """True if the compiled core is importable."""
    return _cpp is not None


def module() -> Any:
    """The compiled core, or ``None`` if it was not built."""
    return _cpp


def require() -> Any:
    """The compiled core; raises ``RuntimeError`` if it was not built."""
    if _cpp is None:  # pragma: no cover - only without the extension
        raise RuntimeError(_MISSING)
    return _cpp


def resolve(backend: str | None) -> str:
    """Turn a caller's ``backend`` into ``"cpp"`` or ``"python"``.

    ``None`` and ``"auto"`` prefer the core and fall back to the reference;
    ``"cpp"`` raises when the core is missing rather than silently degrading.
    """
    name = DEFAULT_BACKEND if backend is None else str(backend).lower()
    if name not in BACKENDS:
        raise ValueError(f"unknown backend {backend!r}; expected one of {BACKENDS}")
    if name == "auto":
        return "cpp" if available() else "python"
    if name == "cpp" and not available():  # pragma: no cover - only without the extension
        raise RuntimeError(_MISSING)
    return name
