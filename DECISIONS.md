# Decisions

One line per decision: what was chosen and why.

## Step 1 — grid A*

- Grid indexed `[y, x]`, cells addressed as `(x, y)` tuples — keeps numpy row/column order
  while letting the planners talk in map coordinates.
- Three cell types only (aisle 1.0, parking 3.0, obstacle) as the spec lists; curbs inside
  parking bands are obstacle cells so the depot has interior structure to plan around.
- Obstacle proximity penalty = `weight * (1 - d/radius)^exponent` for `0 < d < radius`,
  with `d` from `distance_transform_edt`; quadratic decay keeps aisle centres at cost 1.0
  so the octile heuristic stays admissible against a meaningful scale.
- Cells outside the grid are treated as free by the distance transform: the enclosing wall
  already supplies the penalty near the border, and this avoids double counting.
- Heuristic scale = minimum finite cell cost of the actual cost map (not the configured
  aisle cost), so admissibility holds for any cost map the config produces.
- One generic `search()` implements Dijkstra / A* / weighted A*; `dijkstra`, `astar` and
  `weighted_astar` are thin wrappers, so the three variants cannot drift apart.
- Ties in the open list are broken by insertion order (a monotone counter), which makes
  every search reproducible run to run.
- "No path" is a `SearchResult(success=False, reason=...)`, never an exception — the
  closed-loop runner in step 4 has to handle it every replan.
- `make test` runs `python3 -m pytest -q` rather than bare `pytest`: this image has a
  standalone `uv`-installed `pytest` earlier on `PATH` that cannot see the project's
  numpy. `python3 -m pytest -q` is the same run against the right interpreter.

## Step 2 — scenarios with moving vehicles

- Agents are 2x2-cell rectangles anchored at their lower-index corner: a 4 m aisle and a
  3 m cross aisle both take one, so traffic can use the whole aisle network.
- Agent routes are planned only over anchors whose entire footprint lands on **aisle**
  cells, which keeps other vehicles driving in the lanes instead of across parking bands.
- Ego start and goal are single aisle cells at least 30 m apart; the ego is a point on the
  grid until step 5 gives it a car shape.
- The `crossing` and `head_on` agents run with zero pause probability so their meeting with
  the ego happens at a computed step rather than by luck; `blocked_then_clears` and
  `congested` traffic pauses with probability 0.12 to look less mechanical.
- `blocked_then_clears` releases its blocker `5..40` steps **after the step the ego would
  naturally arrive**, not after t=0. Measured from t=0 the blocker was sometimes gone
  before the ego reached it, which made the scenario a no-op in 1 of 20 instances.
- Agent conflicts are rejected, not repaired: a candidate agent that overlaps a wall or an
  already-placed agent (including a swap through each other) is resampled.
- "Solvable in principle" is checked by marking every agent's **final resting** footprint,
  inflated by 1 cell, as an obstacle and running step-1 A* from start to goal. This is
  stronger than checking the empty map: it rules out a blocker that parks in the only gap.
- A failed generation raises `ScenarioGenerationError` after a bounded attempt budget
  rather than looping forever.

## Step 3 — space-time A*

- Default heuristic is `grid_dijkstra`: the exact step-1 cost-to-go to the goal on the
  static map, computed once per scenario with the step-1 Dijkstra code and reused across
  replans. It is admissible (agents can only add cost or waiting) and consistent (it is an
  exact distance under the same cost model), and it dominates the plain octile heuristic:
  over 15 scenarios it gave **identical optimal costs** with 15k expansions instead of
  176k, and 0.4 s instead of 5.2 s. Plain `octile` stays selectable and a test asserts the
  two agree on cost. Step 5 uses the same obstacle-aware field, so this keeps the project
  consistent rather than introducing a second idea.
- Heuristic adds `chebyshev(cell, goal) * time_cost`: any path needs at least that many
  steps, so the time term stays admissible too.
- A wait costs `time_cost` and nothing else (it moves nowhere); a move costs the step-1
  movement cost plus `time_cost`. The result reports `cost`, `movement_cost` and
  `wait_steps` separately.
- The spec's "with no agents its cost equals step-1 A*" is tested with `time_cost = 0`,
  because a non-zero per-step time cost is by construction a different objective. With the
  default `time_cost = 0.05` the test instead asserts no waiting and a movement cost that
  is never below and at most 2% above the step-1 optimum.
- Safety margin: the ego may not enter an agent footprint inflated by 1 cell. If the ego
  already stands inside that margin (the closed loop can produce this — the margin is a
  comfort buffer, not a collision) the margin is relaxed for one move so the ego can get
  out, but an agent *body* is never enterable. Without this the runner can dead-end on a
  state that is not actually a collision.
- The baseline replans every step (`replan_every = 1`) as the spec describes, while
  space-time A* uses the runner's default cadence of 4. That gives the baseline the more
  reactive schedule, so the comparison is conservative in its favour.
- When the baseline's A* finds nothing (an agent is sitting on the only corridor) it holds
  position for one step instead of failing the episode. A baseline that gives up the moment
  traffic appears would not be a fair comparison.
- `sim/collision.py` (the independent checker) was written during step 3 rather than step 4,
  because step 3's own check requires it. It shares no code with the planners and uses the
  true footprints, with no safety margin.

## Step 4 — closed-loop simulator and battery

- Collisions are detected during the episode, not afterwards: each executed step is passed
  to the independent checker, and the episode stops on the first violation. The whole
  trajectory is re-checked afterwards, and the battery raises if a planner reported success
  on a trajectory the checker rejects.
- `success`, `collision` and `timeout` are mutually exclusive by construction, so
  "successful episodes have zero collisions" is enforced rather than merely observed.
- `time_to_goal_s` is NaN for failed episodes, so a mean over failures is never silently
  reported as a fast run.
- The space-time planner's horizon is clamped to the scenario's own time limit. Planning
  past the end of the episode cannot help, and searching that far is exactly what made a
  hopeless replan (an agent parked on the goal) cost seconds per step. `max_nodes` was
  cut from 400k to 120k for the same reason; no plan in the battery hits either cap.
- When a planner returns no plan the ego holds position and retries next step, and the
  episode counts a `plan_failure`. Aborting instead would hide the difference between
  "temporarily boxed in" and "genuinely stuck".
- Planner-derived data (cost map, Dijkstra heuristic field) is cached per scenario for the
  duration of an episode, keyed on the scenario object itself rather than `id()`, so a
  recycled address cannot serve a stale cost map.
- GIFs pick one seed per scenario type and render *both* planners on it, so the two
  animations show the same traffic. The seed is chosen by the battery slice: prefer a seed
  where the baseline fails, then the one where space-time A* waits most.

## Step 5 — hybrid A* parking

- Coordinates keep the project convention (`x` right, `y` **down**) so the parking renderer
  and the grid renderer agree; the heading is measured from `+x` towards `+y`. The bicycle
  equations are unchanged by this, only the picture is mirrored.
- The reference point is the centre of the rear axle, and the 4.5 x 1.9 m footprint runs
  from 0.9 m behind it to 3.6 m in front.
- **The distance field is a deliberate underestimate.** `distance_transform_edt` measures
  centre to centre, and a lookup snaps the query to its cell, so the raw transform can
  report up to one cell diagonal more clearance than exists. The first parking battery run
  caught exactly this: the planner returned a path that clipped a parked car's corner by
  ~1 cm and the independent checker rejected it. Fixed at the source by storing
  `max(edt - sqrt(2), 0) * resolution` and halving the field resolution to 0.05 m, which
  leaves the stored value a true lower bound at ~7 cm of conservatism.
- Reeds-Shepp covers the CSC (`LSL`, `LSR`), CCC (`LRL`) and SCS (`SLS`) families with the
  standard timeflip/reflect symmetries, not all 48 Reeds-Shepp words. The curve returned is
  therefore the shortest *candidate*, not provably the global optimum. Every candidate was
  validated numerically: 7891 curves over random pose pairs all land on the goal to within
  1e-6 in position and heading.
- Without the analytic expansion the 0.3 m / 5 degree goal tolerance is essentially
  unreachable from 1 m arcs on a 0.5 m / 5 degree lattice, so Reeds-Shepp is enabled by
  default. It is a configuration switch (`analytic.enabled`), and the base search alone
  still solves the open-lot tests.
- The heuristic is `max(Euclidean, obstacle-aware grid distance)`. The grid term comes from
  the step-1 Dijkstra code on a 1 m grid whose cells are drivable when their centre is not
  inside an obstacle (the optimistic choice). An 8-connected grid overestimates a straight
  line by up to `1/cos(pi/8)`, so the term is scaled by `cos(pi/8)`. Even so this is the
  standard hybrid A* "holonomic with obstacles" heuristic and is not a proven lower bound in
  every obstacle configuration, so hybrid A* here is not claimed to be optimal.
- Motion primitives are precomputed once in the car's own frame and rotated onto each node,
  and the whole fan of 10 arcs x 5 sub-steps x 4 discs is collision checked in a single
  distance-field lookup. That took the twelve smoke-test scenarios from 47 s to 4.3 s with
  identical outcomes.
- `sim/car_collision.py` is the independent checker for step 5: exact car rectangle against
  exact obstacle rectangles by the separating-axis test, on a path interpolated to 0.05 m.
  It shares nothing with the planner's disc-and-field model, and a test asserts the disc
  model never accepts a pose the exact test rejects.
- A `tight` scenario picks one of the three base layouts at random and applies the
  minimal-clearance overrides, rather than being a fourth layout of its own.

## Hard tier (added after step 5)

- Hard types are **additive**, never edits to the normal tier. `configs/scenarios.yaml`,
  `configs/parking.yaml` and `configs/eval.yaml` each gained a separate block; resolving a
  hard type deep-merges its overrides into a *copy*, and a test asserts the normal configs
  are byte-identical afterwards. Both normal tiers were re-run and match their previous
  numbers exactly.
- Hard space-time types: `congested_hard` (12-16 agents rather than 6-10) and
  `head_on_narrow` (aisles cut from 4 cells to 3, where a 2-cell vehicle plus its 1-cell
  margin fills the lane completely, so the ego cannot squeeze past at all).
- The per-replan budget is **50 ms**, chosen as one fifth of the 250 ms simulation step —
  a real-time argument, not a number picked to produce a particular score. It applies to
  both planners so the comparison stays fair.
- The parking hard tier draws its gap from `minimum + 0.3 .. 0.6 m`, where the minimum is
  **computed** in `world/parking.py` from the car geometry and the collision model
  (`minimum_parallel_gap`, `minimum_spot_width`), not written down by hand. A test pins
  that derivation to reality: at `minimum + 0.1 m` the goal pose is collision free, at
  `minimum - 0.1 m` it is not.
- The parking expansion cap is **5000**, about the normal tier's mean for `parallel`
  (6372) and a sixth of the normal 30000: "you get roughly an average normal budget".
- Nothing was tuned after the hard tier was first run. Two of its results are worth stating
  plainly rather than engineering away:
  - **The cheap baseline beats space-time A\* in narrow aisles under a budget.** On
    `head_on_narrow` the baseline scores 100% and space-time A* 53%, because a 50 ms
    budget is not enough for the space-time search in a narrow aisle, so it returns no
    plan, holds position and times out. Optimality is worthless if it does not fit in the
    control loop.
  - **`perpendicular_minimal` is not as hard as it sounds.** Its computed band
    (2.43-2.73 m) is *wider* than the existing `tight` type's fixed 2.3 m spot, so its
    difficulty comes almost entirely from the expansion cap, and it scores 97%. The band
    follows the specification; the honest observation is that the spec's own `tight` tier
    was already below it.

## Step 6 — report and README

- `REPORT.md` is generated from the CSVs. The one thing `make_report.py` computes itself is
  the step-1 benchmark, which it runs and writes to `results/grid_benchmark.csv` before
  reading it back, so "every number comes from running the code" still holds.
- Report figures and failure frames are written to `results/README_assets/report/` rather
  than a gitignored directory, so the images in `REPORT.md` and `README.md` actually render
  on GitHub. `.gitignore` already un-ignores `results/README_assets/`.
- Failure diagnoses are derived from what the episode recorded (plan-failure reasons and
  counts, wait steps, distance covered, plan age at the moment of collision), not written
  by hand per case, so they stay correct when the batteries are re-run.
- Writing the failure section caught two bugs in step 4's reporting:
  - `run_episode` passed a two-step window to the collision checker, so the reported
    `collision_step` was 0 or 1 rather than a position in the trajectory. The failure
    frames were rendering the wrong moment. The runner now translates the index back.
  - The first draft "staleness" figure was `steps - collision_step`, which is not
    staleness at all. It now comes from `EpisodeResult.plan_age_at`, the gap between the
    collision and the last replan. For the baseline it is typically 1 step, which is the
    real point: replanning every single step does not help if you freeze a moving vehicle
    where it currently stands.

## Step 7 — C++ core

- Built with a plain `setup.py` `Pybind11Extension` rather than scikit-build-core, because
  the project already uses the setuptools backend and needed no other build machinery.
- `build_ext` is wrapped so a missing compiler warns instead of failing the install, and
  `depot_planner/grid_astar/backend.py` falls back to the Python search. `pip install -e .`
  therefore works with or without a toolchain.
- The core reproduces the Python search *exactly*, not approximately. The first version
  matched every cost but differed on 34 of 600 paths: the Python heap pushes
  `(f, g, counter, cell)`, so ties on `f` break towards the smaller `g`, and the C++
  comparator only had `(f, order)`. With `g` added, all 600 comparisons return the same
  path, the same cost and the same expansion count.
- A second parity bug: C++ used `time_limit_ms = 0.0` as its "unlimited" sentinel while
  Python treats an explicit `0.0` as "expire immediately". The sentinel is now negative,
  and a test runs both backends at `time_limit_ms=0.0` and expects both to give up.
- `search(backend=...)` defaults to `"auto"`: the core whenever it is built and the call
  does not need the expanded-cell set, Python otherwise. To make sure this did not quietly
  change any published number, all four battery tiers were re-run and diffed against the
  pre-extension CSVs on every non-timing column.
- The core is not used where the caller wants `expanded` (the step-1 figures) or a
  heuristic scale that disagrees with the grid minimum, because either would change the
  search rather than just speed it up.
