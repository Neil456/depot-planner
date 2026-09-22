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
- [x] **Step 4: Closed-loop simulator and scenario battery** — done. Runner replans on a
      cadence and checks every executed step against the independent checker; battery of
      300 episodes (30 per type x 5 types x 2 planners) in 33 s ->
      `results/battery_spacetime.csv`; 10 GIFs in `results/gifs/`, largest 0.15 MB.
      Space-time A*: 100% success, 0 collisions. Baseline: 5 collisions on `crossing`,
      2 on `congested`. 79 tests green.
- [x] **Step 5: Hybrid A* parking** — done. Kinematic bicycle car, disc/distance-field
      collision checking, Reeds-Shepp analytic expansion, four seeded parking scenario
      types. Battery of 120 scenarios in 57 s -> `results/battery_parking.csv`:
      100% success on all four types, every plan verified by an independent exact
      rectangle checker. GIFs in `results/gifs/parking/`. 120 tests green.
- [ ] Step 6: Report and README
- [ ] Step 7 (optional): C++ core for grid A*

## Notes

- Run the suite with `make test` (= `python3 -m pytest -q`); see `DECISIONS.md` for why
  the bare `pytest` binary on this image is the wrong interpreter.
