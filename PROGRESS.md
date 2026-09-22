# Progress

Read this first. Status of each step from `TASK.md`.

- [x] **Step 1: Grid A* with driving cost map** — done. Occupancy grid + EDT proximity
      penalty, one generic best-first search (Dijkstra / A* / weighted A*), deterministic
      depot generator, renderer; `results/step1/compare.png` written. 15 tests green.
- [x] **Step 2: Scenarios with moving vehicles** — done. 2x2 agents on precomputed aisle
      timelines; `empty`, `crossing`, `head_on`, `blocked_then_clears`, `congested`, all
      seed-reproducible and verified solvable once agents rest;
      `results/step2/*.png` written. 37 tests green.
- [x] **Step 3: Space-time A*** — done. A* over (x, y, t) with a wait action, agent
      margin + swap rules, admissible Dijkstra-field heuristic, and the replanning
      baseline. The independent collision checker was pulled forward from step 4
      because step 3's check needs it. 57 tests green.
- [ ] Step 4: Closed-loop simulator and scenario battery
- [ ] Step 5: Hybrid A* parking
- [ ] Step 6: Report and README
- [ ] Step 7 (optional): C++ core for grid A*

## Notes

- Run the suite with `make test` (= `python3 -m pytest -q`); see `DECISIONS.md` for why
  the bare `pytest` binary on this image is the wrong interpreter.
