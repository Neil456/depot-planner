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
- [x] **Step 6: Report and README** — done. `scripts/make_report.py` builds `REPORT.md`
      from the CSVs; three GIFs and every report figure are committed under
      `results/README_assets/` so they render on GitHub. The failure section lists all
      62 failed runs with a frame and a one-line diagnosis each.
- [x] **Step 7 (optional): C++ core for grid A*** — done. `cpp/grid_astar.cpp` via
      pybind11, built by `pip install -e .` and skipped gracefully without a compiler.
      600 comparisons over 200 start/goal pairs return identical paths, costs and
      expansion counts; 13-30x faster.
- [x] **Hard tier** (added on request after step 5) — done. `congested_hard`,
      `head_on_narrow` with a 50 ms per-replan budget; `parallel_minimal`,
      `perpendicular_minimal` with minimum-clearance gaps and a 5000-expansion cap.
      30 episodes per type, nothing tuned afterwards.
- [x] **CI** — `.github/workflows/tests.yml` runs `pytest -q` on push and pull request.

## How to run

```
make setup     # pip install -e ".[dev]"
make test      # 120 tests, ~55 s
make step1     # results/step1/compare.png
make battery   # both batteries, ~90 s
make gifs      # all GIFs and demo PNGs, ~75 s
```

Run the suite with `make test` (= `python3 -m pytest -q`); see `DECISIONS.md` for why the
bare `pytest` binary on this image is the wrong interpreter.

## Measured timings on this machine

| stage | wall clock |
| --- | --- |
| `make test` (159 tests) | 84 s |
| space-time battery, normal tier (300 episodes) | 25 s |
| space-time battery, hard tier (120 episodes) | 261 s |
| parking battery, normal tier (120 scenarios) | 58 s |
| parking battery, hard tier (60 scenarios) | 23 s |
| space-time GIFs (10) | 31 s |
| parking GIFs (4) | 39 s |
| `make report` (incl. re-running 62 failures) | ~170 s |

`make all` therefore runs in roughly 12 minutes, inside the 30-minute budget.

## Headline numbers

(From the generated CSVs under `results/`, which `.gitignore` excludes by design — rerun
`make battery` to regenerate them. Full tables are in `REPORT.md`.)

**Normal tier**

- Space-time A*: 100% success and **zero collisions** over 150 episodes.
- Replanning baseline: 5 collisions on `crossing`, 2 on `congested` (150 episodes).
- Hybrid A* parking: 100% success on all four types over 120 scenarios, every plan
  verified collision free by the independent exact-rectangle checker.

**Hard tier** (30 episodes/scenarios per type)

- `congested_hard`: space-time A* 66.7% (1 collision, 9 timeouts), baseline 66.7%
  (10 collisions).
- `head_on_narrow`: space-time A* 53.3% (0 collisions, 14 timeouts), baseline **100%**.
- `parallel_minimal`: 33.3%. `perpendicular_minimal`: 96.7%.

**C++ core**: 13-30x faster than the Python search, identical paths on all 600 comparisons.

## Things worth knowing

- The first parking battery run failed loudly: the planner returned a path the independent
  checker rejected. That was a real bug in the distance field, fixed at the source rather
  than papered over (see `DECISIONS.md`, step 5).
- Writing step 6's failure section caught two more bugs in step 4's reporting: a
  window-relative collision index that made the failure frames show the wrong moment, and a
  bogus "plan staleness" figure. Both fixed; see `DECISIONS.md`, step 6.
- The C++ core's first version matched every cost but differed on 34 of 600 *paths*,
  because it was missing the Python heap's secondary tie-break on `g`. Fixed and pinned by
  a test.
- **The hard tier found a result worth reading twice**: in narrow aisles under a 50 ms
  replan budget the cheap myopic baseline scores 100% while optimal space-time A* scores
  53%, because the space-time search does not fit in the budget and the ego stalls. Nothing
  was tuned to change this.
- The hard space-time tier is **not bit-reproducible** — a wall-clock budget depends on the
  machine. Re-running it left every success/collision/timeout value identical but moved
  `nodes_expanded` on 40 of 120 episodes. See `REPORT.md` section 5.
