# Progress

Read this first. Status of each step from `TASK.md`.

- [x] **Step 1: Grid A* with driving cost map** — done. Occupancy grid + EDT proximity
      penalty, one generic best-first search (Dijkstra / A* / weighted A*), deterministic
      depot generator, renderer; `results/step1/compare.png` written. 15 tests green.
- [ ] Step 2: Scenarios with moving vehicles
- [ ] Step 3: Space-time A*
- [ ] Step 4: Closed-loop simulator and scenario battery
- [ ] Step 5: Hybrid A* parking
- [ ] Step 6: Report and README
- [ ] Step 7 (optional): C++ core for grid A*

## Notes

- Run the suite with `make test` (= `python3 -m pytest -q`); see `DECISIONS.md` for why
  the bare `pytest` binary on this image is the wrong interpreter.
