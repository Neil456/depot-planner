# Progress

Read this first. Status of each step from [`TASK.md`](TASK.md), then of the
follow-up C++ port in [`TASK_CPP.md`](TASK_CPP.md).

## Original brief ([`TASK.md`](TASK.md))

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
- [x] **CI** — `.github/workflows/tests.yml` runs `pytest -q` on push.
- [x] **Public-repo polish** — hero and side-by-side GIFs, README rewritten with
      generated tables, MIT licence, dead code removed, docstrings completed, project
      docs moved into `docs/`. No planner behaviour or recorded result changed.

## C++ port ([`TASK_CPP.md`](TASK_CPP.md))

The planners move into a real C++17 library; Python keeps scenario generation,
orchestration, the independent collision checkers, plotting and reports.

- [x] **C++ step 1: project foundation** — done. `cpp/` holds a CMake project building
      the static library `depot_core`, the pybind11 module `depot_planner._cpp`, a
      GoogleTest binary and a Google Benchmark binary. `pip install -e .` builds the
      library and the extension through scikit-build-core; GoogleTest and Google
      Benchmark are fetched by CMake only for the standalone `make cpp-test` /
      `make cpp-bench` build, so an install never downloads them. `.clang-format`,
      `make format`, `make format-check` and `make cpp-test` added. 11 GoogleTest
      cases and 170 pytest tests green.
- [x] **C++ step 2: shared core types, templated grid search, distance field** — done.
      `include/depot/types.hpp` holds `Cell`, `Pose`, `Path`, the owning `Grid2D<T>` and the
      borrowing `GridView<T>`, `SearchLimits` and `SearchResult`. One templated
      `BestFirstSearch` serves Dijkstra, A* and weighted A* through heuristic functors,
      with the Python heap's `(f, g, insertion order)` tie-break preserved. The Dijkstra
      cost-to-go field and an exact integer Euclidean distance transform are in C++ too,
      and `Grid.obstacle_distance`, `dijkstra_field` and the parking `DistanceField` all
      use them. 18 GoogleTest cases; pytest compares the two backends on 200 random
      start/goal pairs x 3 variants (identical costs, paths and node counts), 200 random
      masks against scipy, and every parking distance field, all bit-identical.
- [x] **C++ step 3: space-time A* and the snapshot baseline** — done. `(x, y, t)` states,
      the wait action, the margin and swap rules with the margin relaxation, the per-step
      time cost, the Dijkstra cost-to-go heuristic, the node/time/horizon caps and the
      per-search wall-clock budget are all in C++, as is the baseline's "freeze the agents
      where they stand" snapshot. 16 more GoogleTest cases.
      `scripts/check_equivalence.py` compares both planners on **every** battery seed of
      both tiers from three start steps: 1260 plans, all identical, with no
      equal-cost tie-break differences at all. The normal-tier closed-loop battery is
      byte-identical on every non-timing column across 300 episodes, and mean planning
      time drops from 4.54 ms to 0.11 ms (40x).
- [x] **C++ step 4: hybrid A* and Reeds-Shepp** — done. The kinematic bicycle model,
      the motion primitives, the heading discretisation, every cost term, the disc-based
      collision check against the distance field, the `max(Euclidean, obstacle-aware grid)`
      heuristic and the Reeds-Shepp analytic expansion are all in C++. 25 more GoogleTest
      cases, including the numerical Reeds-Shepp validation the Python version was held to
      (every candidate over 3000 random pose pairs lands on its goal to 1e-7).
      `scripts/check_equivalence.py --section hybrid` plans **all 180** parking battery
      scenarios of both tiers on both backends: identical success, cost, expansion counts,
      collision-check counts and pose sequences, and every C++ plan passes the Python
      independent rectangle checker. 20-45x faster.
- [x] **C++ step 5: the pipeline runs on C++ by default** — done. Every planner resolves to
      the core unless told otherwise, and `--backend python` on both battery scripts runs the
      reference. All four battery tiers were rerun on both backends and `REPORT.md` and
      `README.md` regenerated. **The hard tier's headline result inverted**: under the same
      50 ms budget, space-time A* went from 66.7%/53.3% on the reference to 100%/100% on the
      core, because the search now fits in the budget. Both backends are reported side by
      side in REPORT.md section 2 and README.md.
- [x] **C++ step 6: benchmarks and correctness tooling** — done. `cpp/bench/` is a Google
      Benchmark suite over grid A*, the Dijkstra field, space-time A* (both heuristics), the
      snapshot baseline, hybrid A* (with and without the analytic expansion) and the distance
      transform, all on fixed scenarios built in C++. `scripts/bench_cpp.py` times the same
      planners through the bindings against the Python reference, five runs per case with the
      median kept and scenario setup excluded; the table is in REPORT.md section 7 and in the
      README. Median speed-ups: 12-30x on the grid searches, **41x** on space-time A*, 25x on
      hybrid A*, with identical plans on every case. CI now has three jobs: the Python suite
      (which asserts the core was actually built), a C++ job running `clang-format --Werror`,
      ctest and a benchmark smoke run, and a third running the same 59 GoogleTest cases under
      AddressSanitizer and UndefinedBehaviorSanitizer.
- [x] **C++ step 7 (optional): closed-loop runner in C++** — done. `cpp/src/runner.cpp`
      runs a whole episode — replan cadence, plan execution, plan-failure holding, the
      per-step legality check and every metric — without returning to Python. It is opt-in
      (`run_episode(..., engine="cpp")`, `scripts/run_battery.py --engine cpp`); the Python
      loop stays the default because it hands every executed step to the untouched
      independent checker as the episode runs, which is the project's strongest safety
      property. `scripts/check_equivalence.py --section runner` compares 420 episodes across
      both tiers and both planners: identical on every metric. The loop itself is **1.6x**
      faster — a modest number, and the honest one: with the planners already in C++, the
      per-step Python overhead was only about 40% of an episode.
- [x] **C++ step 8: docs** — done. The README's "C++ core" section is now an architecture
      section: a table of what is C++ and what is Python, why the split falls there, why the
      Python planners are kept as the reference, and why the two independent checkers stay in
      Python. Build instructions cover `pip install -e .`, the standalone `make cpp-test` /
      `make cpp-bench` / `make format` targets and the `--backend` / `--engine` flags. The
      benchmark table, every results table and both findings are generated from the CSVs, and
      the GIFs regenerated byte-identically.

## How to run

```
make setup      # pip install -e ".[dev]"  (builds the C++ core via scikit-build-core)
make test       # the Python test suite
make cpp-test   # configure, build and ctest the C++ core (fetches GoogleTest)
make cpp-bench  # the Google Benchmark suite
make format     # clang-format the C++ sources in place
make step1      # results/step1/compare.png
make battery    # both tiers of both batteries
make gifs       # every GIF and demo figure, including the README showcase
make report     # regenerates REPORT.md and the README's generated tables
make all        # setup, tests, batteries, GIFs and the report, in order
```

Run the suite with `make test` (= `python3 -m pytest -q`); see `DECISIONS.md` for why the
bare `pytest` binary on this image is the wrong interpreter.

## Measured timings on this machine

All planners run on the C++ core; the Python reference column is what the same work costs
on `--backend python`.

| stage | C++ core | Python reference |
| --- | --- | --- |
| `make test` (219 tests, exercises both backends) | 84 s | — |
| `make cpp-test` (59 GoogleTest cases, after the first configure) | 2 s | — |
| space-time battery, normal tier (300 episodes) | 6 s | 30 s |
| space-time battery, hard tier (120 episodes) | 23 s | 267 s |
| parking battery, normal tier (120 scenarios) | 12 s | 60 s |
| parking battery, hard tier (60 scenarios) | 4 s | 23 s |
| demo figures + space-time GIFs (10) | 25 s | — |
| parking GIFs (4) + showcase GIFs (2) | 63 s | — |
| `make report` (incl. re-running 38 failures) | 29 s | — |

`make all` now runs in roughly 4 minutes, against about 12 before the port.

## Headline numbers

(From the generated CSVs under `results/`, which `.gitignore` excludes by design — rerun
`make battery` to regenerate them. Full tables are in `REPORT.md`.)

**Normal tier**

- Space-time A*: 100% success and **zero collisions** over 150 episodes.
- Replanning baseline: 5 collisions on `crossing`, 2 on `congested` (150 episodes).
- Hybrid A* parking: 100% success on all four types over 120 scenarios, every plan
  verified collision free by the independent exact-rectangle checker.

**Hard tier** (30 episodes/scenarios per type), on the C++ core

- `congested_hard`: space-time A* **100%**, baseline 66.7% (10 collisions).
- `head_on_narrow`: space-time A* **100%**, baseline 100%.
- `parallel_minimal`: 33.3%. `perpendicular_minimal`: 96.7%.

The same tier on the Python reference, same seeds and same 50 ms budget: space-time A*
63.3% on `congested_hard` (2 collisions, 9 timeouts) and 50.0% on `head_on_narrow`
(15 timeouts). The baseline is unmoved, because the budget never binds for it. The parking
tiers are capped by expansions rather than wall clock, so both backends score identically.

**C++ core**: 13-30x on the grid search, ~40x on space-time A*, 20-45x on hybrid A*, with
identical plans on every comparison `scripts/check_equivalence.py` makes.

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
- **The hard tier found a result worth reading twice, and then the C++ port reversed it.**
  On the Python reference, in narrow aisles under a 50 ms replan budget, the cheap myopic
  baseline scored 100% while optimal space-time A* scored 50%: the search did not fit in the
  budget and the ego stalled. On the C++ core, the identical planner on the identical seeds
  scores 100%. The algorithm was never the problem; the implementation was. Both tiers are
  reported side by side rather than the slower one being quietly dropped.
- The hard space-time tier is **not bit-reproducible** — a wall-clock budget depends on the
  machine and on how fast the planner runs. See `REPORT.md` section 5.
