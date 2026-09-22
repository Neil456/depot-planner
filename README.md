# Depot Planner

Motion planning for a vehicle depot seen from above: grid A\* for routing, space-time A\* for driving through moving traffic, and hybrid A\* for parking a car-shaped vehicle. The planners are a C++17 library; Python drives the seeded evaluation harness that scores them and the independent collision checkers that audit every plan.

[![tests](https://github.com/Neil456/depot-planner/actions/workflows/tests.yml/badge.svg)](https://github.com/Neil456/depot-planner/actions/workflows/tests.yml)

![space-time A* yielding to crossing traffic in a depot](results/README_assets/hero.gif)

## What this is

A depot is a hard, small planning problem: narrow aisles, parked cars, and other
vehicles crawling through the same lanes you need. This repository builds three
planners for that world and then spends most of its effort on the part that
usually gets skipped — measuring them. Every scenario comes from a seed, every
planner runs in closed loop against traffic that does not care about it, and
every trajectory is re-checked by a collision checker that shares no code with
any planner. When the checker and a planner disagree, the harness raises rather
than recording a pass. It has caught real bugs, which are listed below.

The planners themselves are a C++17 library, with the original Python ones kept
as the reference implementation and tested against on every battery seed. That
turned out to matter for more than speed: under a 50 ms per-replan budget, the
same planner on the same seeds goes from half the episodes to all of them purely
because the search now fits in the budget. See [C++ core](#c-core).

## Planners

**Grid A\*.** The depot becomes a 1 m grid: aisles are cheap, parking areas cost
three times as much, walls and curbs are not drivable, and a soft penalty near
obstacles keeps routes off the paintwork. One search routine covers Dijkstra (no
guidance), A\* (guided by an octile distance scaled so it can never overestimate,
so the route stays optimal) and weighted A\* (guidance exaggerated by 1.5×, which
finds a slightly worse route far faster).

**Space-time A\*.** Traffic moves, so a plan has to say *where* and *when*. This
searches over `(x, y, time)` and can choose to **wait** in place for a step.
Waiting costs something, so it only waits when that beats going around. It will
not enter a cell a vehicle will occupy at that instant, keeps a one-cell buffer
around every vehicle, and will not swap places with one.

**The replanning baseline.** The honest straw man. Plain grid A\*, treating other
vehicles as walls standing wherever they are at this instant, replanning every
single step. It is orders of magnitude cheaper and often fine — but it has no idea
anything is moving, so it drives into the gap a crossing vehicle is about to
fill.

**Hybrid A\*.** For parking, the ego stops being a dot and becomes a 4.5 × 1.9 m
car with a 2.7 m wheelbase that steers like a car. The search expands 1 m arcs at
five steering angles, forward and in reverse, paying extra for reversing and for
changing direction. The body is covered with four discs checked against a
distance field of the obstacles. Because 1 m arcs will essentially never land
exactly on a target pose, the search periodically fires a **Reeds-Shepp curve**
at the goal and takes it if it is collision-free.

The two space-time planners on the same depot, the same traffic and the same
seed:

![space-time A* reaching the goal beside the baseline colliding on the same seed](results/README_assets/side_by_side.gif)

## Results

Everything below is generated from the CSVs the batteries write — see
[REPORT.md](REPORT.md) for the full tables, the plots, and every failure case
with the frame where it failed.

### Grid search, 100 start/goal pairs

Timed on whichever backend is the default here; the `backend` column says which,
and the [C++ core](#c-core) section below compares the two directly.

<!-- BEGIN GENERATED: grid_table -->
| algorithm | backend | cost / optimal | nodes expanded | mean ms |
| :-- | :-- | :-- | :-- | :-- |
| Dijkstra | cpp | 1.0000 | 1479 | 0.26 |
| A* | cpp | 1.0000 | 348 | 0.08 |
| Weighted A* (w=1.5) | cpp | 1.0297 | 96 | 0.03 |
<!-- END GENERATED: grid_table -->

A\* and Dijkstra agree on cost to the last decimal on every pair, which is the
admissibility check; A\* just gets there having looked at a quarter as much.

### Space-time A\* vs the baseline — normal tier

Five scenario types, 30 episodes each, both planners on the same seeded
scenarios.

<!-- BEGIN GENERATED: spacetime_normal -->
| scenario | planner | success % | collisions | timeouts | mean waits | mean plan ms |
| :-- | :-- | :-- | :-- | :-- | :-- | :-- |
| empty | spacetime_astar | 100.0 | 0 | 0 | 0.0 | 0.07 |
| empty | baseline_replan | 100.0 | 0 | 0 | 0.0 | 0.05 |
| crossing | spacetime_astar | 100.0 | 0 | 0 | 2.7 | 0.07 |
| crossing | baseline_replan | 83.3 | 5 | 0 | 0.2 | 0.05 |
| head_on | spacetime_astar | 100.0 | 0 | 0 | 1.3 | 0.18 |
| head_on | baseline_replan | 100.0 | 0 | 0 | 0.0 | 0.06 |
| blocked_then_clears | spacetime_astar | 100.0 | 0 | 0 | 9.6 | 0.14 |
| blocked_then_clears | baseline_replan | 100.0 | 0 | 0 | 0.0 | 0.05 |
| congested | spacetime_astar | 100.0 | 0 | 0 | 9.4 | 0.42 |
| congested | baseline_replan | 93.3 | 2 | 0 | 1.8 | 0.08 |
<!-- END GENERATED: spacetime_normal -->

### Hard tier

`congested_hard` puts 12–16 vehicles in the depot; `head_on_narrow` shrinks the
aisles to three cells, where one vehicle plus its safety buffer fills the lane
completely. Both planners also get a hard wall-clock budget per replan, so this
is the one tier where how fast the planner runs changes what it achieves.
Running on the C++ core:

<!-- BEGIN GENERATED: spacetime_hard -->
| scenario | planner | success % | collisions | timeouts | mean waits | mean plan ms |
| :-- | :-- | :-- | :-- | :-- | :-- | :-- |
| congested_hard | spacetime_astar | 100.0 | 0 | 0 | 15.3 | 3.11 |
| congested_hard | baseline_replan | 66.7 | 10 | 0 | 6.3 | 0.09 |
| head_on_narrow | spacetime_astar | 100.0 | 0 | 0 | 3.1 | 0.52 |
| head_on_narrow | baseline_replan | 100.0 | 0 | 0 | 0.0 | 0.07 |
<!-- END GENERATED: spacetime_hard -->

And the identical tier on the pure-Python reference — same scenarios, same
seeds, same budget, the slower implementation:

<!-- BEGIN GENERATED: spacetime_hard_python -->
| scenario | planner | success % | collisions | timeouts | mean waits | mean plan ms |
| :-- | :-- | :-- | :-- | :-- | :-- | :-- |
| congested_hard | spacetime_astar | 63.3 | 2 | 9 | 60.7 | 29.15 |
| congested_hard | baseline_replan | 66.7 | 10 | 0 | 6.3 | 1.21 |
| head_on_narrow | spacetime_astar | 50.0 | 0 | 15 | 107.8 | 28.73 |
| head_on_narrow | baseline_replan | 100.0 | 0 | 0 | 0.0 | 1.46 |
<!-- END GENERATED: spacetime_hard_python -->

<!-- BEGIN GENERATED: finding_budget -->
**Under a 50 ms budget per replan, the implementation decides the outcome.** Space-time A\* on the C++ core reaches the goal in 100.0% of `congested_hard` episodes and 100.0% of `head_on_narrow` episodes. The same planner on the Python reference, with the same budget and the same seeds, manages 63.3% of `congested_hard` with 9 timeouts and 50.0% of `head_on_narrow` with 15 timeouts: the search does not fit in the budget, returns no plan, and the ego stalls in the aisle until the episode times out. The replanning baseline is cheap enough either way and is unmoved at 66.7% and 100.0%, but on `congested_hard` it fails by driving into vehicles (10 collisions) rather than by running out of time. Optimality is worthless if it does not fit in the control loop — and here making it fit was an implementation problem, not an algorithmic one.
<!-- END GENERATED: finding_budget -->

### Hybrid A\* parking

Four types in the normal tier, 30 scenarios each. Every successful plan is
re-verified by the independent exact-rectangle checker.

<!-- BEGIN GENERATED: parking_normal -->
| parking type | success % | mean plan ms | nodes expanded | direction switches |
| :-- | :-- | :-- | :-- | :-- |
| perpendicular_forward | 100.0 | 7.4 | 1448 | 1.10 |
| perpendicular_reverse | 100.0 | 3.3 | 623 | 0.73 |
| parallel | 100.0 | 34.8 | 6372 | 2.83 |
| tight | 100.0 | 19.5 | 3973 | 2.47 |
<!-- END GENERATED: parking_normal -->

Two minimal-clearance types in the hard tier, 30 scenarios each:

<!-- BEGIN GENERATED: parking_hard -->
| parking type | success % | mean plan ms | nodes expanded | direction switches |
| :-- | :-- | :-- | :-- | :-- |
| parallel_minimal | 33.3 | 19.5 | 4029 | 1.50 |
| perpendicular_minimal | 96.7 | 5.6 | 1092 | 0.72 |
<!-- END GENERATED: parking_hard -->

<!-- BEGIN GENERATED: finding_gaps -->
**Parallel parking falls apart in minimal gaps.** The narrowest gap this collision model accepts for a 4.5 m car is 5.72 m, computed from the geometry rather than picked by hand. Given gaps of only 6.02–6.32 m and a 5000-expansion cap, `parallel_minimal` succeeds 33.3% of the time against 96.7% for `perpendicular_minimal`, which needs far less search.
<!-- END GENERATED: finding_gaps -->

Nothing was tuned after the hard tier was first run.

## Bugs found by the evaluation harness

Three defects the harness caught that reading the code did not. All are fixed;
the reasoning is in [docs/DECISIONS.md](docs/DECISIONS.md).

**A 1 cm clip through a parked car.** The first parking battery run aborted: the
planner returned a path the independent checker rejected. `distance_transform_edt`
measures centre-to-centre and a lookup snaps the query to its cell, so the
distance field was reporting up to a full cell diagonal more clearance than
existed — enough for a plan to shave a parked car's corner. Fixed at the source
by storing `max(edt - √2, 0) · resolution`, which makes the stored value a true
lower bound, rather than by loosening the checker.

**The C++ port diverged on paths, not costs.** The first version of the C++
grid search matched the Python one's cost on every comparison but returned a
different path on 34 of 600 of them. The Python heap pushes
`(f, g, counter, cell)`, so ties on `f` break towards the smaller `g`; the C++
comparator only had `(f, order)`. Comparing cost alone would have called that a
success. The equivalence tests now pin path, cost and expansion count together,
which is what caught it and what every later port was held to.

**Failure frames showing the wrong moment.** The runner handed the collision
checker a two-step window, so the recorded collision index was always 0 or 1
instead of a position in the trajectory — every failure image in the report was
rendering the wrong frame. Found only when the images were first looked at.

**A benchmark measuring a hopeless search.** The first C++ benchmark scenario
parked traffic in both vertical connectors, which left the goal unreachable, so
"space-time A\* on a congested depot" was really 717k expansions of a search
that could never finish. The giveaway was two different heuristics reporting the
same expansion count. The agents now pull into the parking band beside their
lane, as the scenario generator's traffic does.

## C++ core

Everything on the critical path is a C++17 library under `cpp/` (`depot_core`),
exposed through pybind11 and built by `pip install -e .` via scikit-build-core.

| in C++ | in Python |
| :-- | :-- |
| grid search — Dijkstra, A\*, weighted A\*, the Dijkstra cost-to-go field | scenario generation: depot maps, traffic timelines, parking layouts |
| the exact Euclidean distance transform behind both distance fields | the two **independent collision checkers** that audit every plan |
| space-time A\* over `(x, y, t)` and the snapshot baseline | orchestration: tiers, seeds, batteries, CSVs |
| hybrid A\* — bicycle model, primitives, disc collision, Reeds-Shepp | plotting, GIFs, and generating REPORT.md and this file |
| the closed-loop episode runner (opt-in, see below) | the pure-Python planners, kept as the **reference implementation** |

**Why this split.** The planners are where the time goes and where a tight inner
loop pays: an A\* expansion is a handful of arithmetic operations and a heap
push, and in Python the interpreter overhead dwarfs the work. Everything else —
generating a scenario once, writing a CSV, drawing a GIF — is glue, and moving
it to C++ would buy nothing while making it harder to change.

**Why the Python planners are still here.** They are the specification. Every
C++ plan is verified against them on the same inputs, and `--backend python`
runs them end to end. `scripts/check_equivalence.py` compares the two on every
battery seed of both tiers; it reports identical plans, costs and expansion
counts, and hands every C++ parking plan to the independent checker as well.

**Why the collision checkers are Python and stay Python.** They exist to
disagree with the planners. `sim/collision.py` re-derives from the grid and the
agent timelines whether an executed trajectory was legal, and
`sim/car_collision.py` does the same for a parking plan using exact rectangles
and the separating-axis test rather than the planner's discs. Neither shares a
line with any planner, and both caught real bugs. Moving them next to the code
they audit would quietly weaken that.

**Getting "identical" to mean identical.** Three CPython-specific numerics had
to be reimplemented rather than assumed. `math.hypot` is not the platform
`hypot` — CPython has its own correctly-rounded norm, and over 200k random pairs
`std::hypot` disagreed with it on 0.6% of them by one ulp, which is enough to
reorder hybrid A\*'s open list and return a different (equally good) path.
Python's float `%` takes the sign of the divisor where C's `fmod` takes the sign
of the dividend, and Python's `//` is derived from the remainder rather than
being `floor(a / b)`. numpy's array `cos` and `sin` were measured against the
scalar ones rather than assumed, and do agree here.

Each case below is run five times per backend and the median kept, on the same
prepared inputs with scenario setup excluded from the clock.

<!-- BEGIN GENERATED: cpp_table -->
| planner | cases | Python ms | C++ ms | speedup | identical plans |
| :-- | :-- | :-- | :-- | :-- | :-- |
| grid A* (Dijkstra) | 40 | 9.0459 | 0.2947 | 30.4 | yes |
| grid A* | 40 | 1.7967 | 0.0864 | 22.5 | yes |
| grid A* (weighted, w=1.5) | 40 | 0.5276 | 0.0480 | 11.6 | yes |
| space-time A* | 10 | 35.5036 | 0.8480 | 41.0 | yes |
| hybrid A* | 8 | 212.9811 | 8.1482 | 25.0 | yes |
<!-- END GENERATED: cpp_table -->

## Quickstart

```bash
pip install -e ".[dev]"   # builds depot_core and the extension via scikit-build-core
make test                 # the Python suite (exercises both backends)
make all                  # setup, tests, batteries, GIFs and the report (~4 min)
```

A compiler, CMake 3.20+ and pybind11 are all `pip install -e .` needs; it never
downloads GoogleTest or Google Benchmark. Those are fetched by CMake only for
the standalone developer build:

```bash
make cpp-test             # configure, build and ctest the core (fetches GoogleTest)
make cpp-bench            # the Google Benchmark suite
make format               # clang-format the C++ sources in place
make format-check         # the CI-friendly dry run
```

Both implementations are reachable from the command line:

```bash
python3 scripts/run_battery.py --backend python   # the reference planners
python3 scripts/run_battery.py --engine cpp       # the closed loop in C++ too
python3 scripts/check_equivalence.py              # compare them on every seed
python3 scripts/bench_cpp.py                      # regenerate the table above
```

Individual stages: `make step1`, `make battery`, `make gifs`, `make report`.
Every tunable number — costs, penalties, resolutions, limits, scenario
parameters — lives in `configs/*.yaml`, never in the code.

## Layout

```
cpp/                    the C++17 planning core
  include/depot/        public headers: types, grid search, distance field,
                        space-time A*, car model, Reeds-Shepp, hybrid A*, runner
  src/                  their implementations
  bindings/             the pybind11 module (depot_planner._cpp)
  tests/                GoogleTest suite
  bench/                Google Benchmark suite, on scenarios built in C++
depot_planner/          the Python side
  core.py               which backend a call runs on
  world/                maps, cost maps, moving-agent scenarios, parking scenarios
  grid_astar/           the reference best-first search and the backend dispatch
  spacetime/            the reference (x, y, t) search and the planner wrappers
  hybrid/               reference car model, disc collision, Reeds-Shepp, hybrid A*
  sim/                  closed-loop runner and two independent collision checkers
  viz/                  top-down renderer, episode GIFs, parking GIFs, showcase GIFs
  eval/                 batteries, benchmarks, report and README generation
configs/                every tunable parameter
scripts/                one CLI entry point per Makefile target
tests/                  the Python test suite, including the equivalence checks
docs/                   the two briefs, the progress log and the decision log
results/                generated; gitignored except results/README_assets/
```

[docs/DECISIONS.md](docs/DECISIONS.md) records every judgement call and why.
[docs/PROGRESS.md](docs/PROGRESS.md) tracks what is done. [docs/TASK.md](docs/TASK.md)
is the original brief and [docs/TASK_CPP.md](docs/TASK_CPP.md) the follow-up one
that moved the planners into C++.

## Limitations

Worth being straight about what these scenarios do and do not model.

- **Other vehicles' trajectories are known exactly and never react.** Each agent
  follows a timeline fixed before planning starts. That is what makes space-time
  A\* clean and also what makes it optimistic: no prediction error, no
  negotiation, no vehicle braking because the ego moved. Real prediction
  uncertainty would need a different formulation.
- **The traffic world is a grid.** Until the parking step the ego is a point that
  moves one cell per step; it has no shape, no heading and no acceleration limit.
  The grid is 1 m and the timestep 0.25 s, so a "collision" is a cell overlap,
  not a swept-volume intersection.
- **The car model is kinematic, not dynamic.** Hybrid A\* respects the steering
  limit and the wheelbase, but there is no mass, no tyre slip, no jerk limit and
  no speed profile — the plan is a geometric path, not a trajectory a controller
  could track directly.
- **Hybrid A\* is not claimed to be optimal.** Its obstacle-aware grid heuristic
  is the standard one and is not a proven lower bound in every obstacle layout,
  and the Reeds-Shepp word set covers the CSC, CCC and SCS families rather than
  all 48 words.
- **The C++ core is verified equal, not verified fast.** The equivalence checks
  say the two implementations return the same plans, not that either is the
  fastest possible: the speed-ups are what this port achieved on this machine
  with this compiler, not an upper bound. The Python side was not deliberately
  slowed down either — it is the same reference implementation the project
  shipped before the port.
- **The hard tier's wall-clock budget is not bit-reproducible, and it is the
  one place the implementation changes the result.** How much search fits in
  50 ms depends on how fast the planner runs and on the machine it runs on. One
  hard seed re-run four times on the same build gives four different expansion
  counts and four different paths. That is why the hard tier is reported on both
  backends rather than only the fast one; see REPORT.md section 5.
