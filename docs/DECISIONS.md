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

## Public-repo polish pass

- **README numbers are generated, not typed.** Every metric in `README.md` sits between
  `<!-- BEGIN GENERATED: name -->` markers and is rewritten by
  `depot_planner/eval/readme.py` from the same CSVs the report reads; `make report` runs it
  after `make_report.py` so the two documents cannot disagree. A test asserts that the
  committed README is byte-identical to what the generator produces, so a stale hand-edited
  table fails CI.
- The two "findings" paragraphs in the README are generated too, including their wording.
  The first draft asserted that the ordering "reverses" on `congested_hard` when both
  planners in fact score 66.7%; the generator now picks its verb from the comparison and
  describes the differing *failure modes* instead, which is the real point.
- Prose keeps no metrics at all. An earlier draft called the baseline "20x cheaper"; that
  ratio changed the moment the C++ core landed, so the claim is gone and the tables carry it.
- **Hero GIF** is the space-time crossing episode rather than a parking manoeuvre: it is the
  one frame that shows the thing the project is actually about (planning in time), and the
  same seed gives the side-by-side against the baseline. The seed is chosen by
  `eval/showcase.best_contrast_episode` from the battery's own seed slice, so the GIFs
  always show a real battery episode and regenerate identically.
- The showcase animations fix the axes rectangle instead of using `tight_layout`, because a
  per-frame layout pass changes the figure's pixel size when the title text changes and GIF
  writers require every frame to match.
- A yielding ego gets an amber halo and a status line; the halo is suppressed on a collision
  frame so it never stacks under the red collision ring.
- `docs/`: `TASK.md`, `PROGRESS.md` and `DECISIONS.md` moved there, with `CLAUDE.md` and
  `configs/hybrid.yaml` updated to match. `TASK.md`'s own contents are left verbatim — it is
  the original brief, an input to the project rather than documentation the project
  maintains. A test asserts no document references the old top-level paths.
- Removed `setup.sh`: it duplicated `make setup` and its only distinctive content was an
  instruction for pasting into a hosted-environment setup field, which is noise in a public
  repository.
- Dead code removed rather than documented: `agent_config`, `AgentOccupancy.blocked_cells`,
  `CarCollisionChecker.first_collision`, `DistanceField.occupied_mask`, `generate_batch`,
  `scenario_generators`, `path_cells`, `summary_line` and `Agent.positions` had no callers.
  The `CellType` re-export in `viz/render.py` was also dropped.
- No planner code, config default or recorded result was changed in this pass. The battery
  CSVs were regenerated from the code so the report and README agree, and the test suite is
  unchanged apart from the new documentation checks.

# C++ port ([`TASK_CPP.md`](TASK_CPP.md))

## C++ step 1 — project foundation

- The follow-up brief is committed verbatim as `docs/TASK_CPP.md`, next to the original
  `docs/TASK.md`, so `PROGRESS.md` and this file can cite the step they are answering.
  Like `TASK.md` it is an input to the project and is never edited afterwards.
- **scikit-build-core replaces the `setup.py` `Pybind11Extension`.** Step 7 of the
  original brief chose setuptools because the C++ was one file and the project already
  used that backend; a real CMake library with its own test and benchmark targets needs
  CMake to drive the build, and scikit-build-core is what the brief asks for.
- **The graceful "no compiler, no problem" fallback is gone.** `pip install -e .` now
  fails if the core will not compile, because from step 5 the C++ backend is the default
  and a silently missing core would mean silently running the reference implementation
  at a tenth of the speed. `grid_astar/backend.py` still falls back to Python when the
  extension is absent, so an in-tree checkout without a build still runs, but nothing
  hides a failed build any more.
- **GoogleTest and Google Benchmark are fetched, never vendored**, and only by the
  standalone developer build (`DEPOT_BUILD_TESTS` / `DEPOT_BUILD_BENCH`, both `OFF` by
  default). `pip install -e .` therefore needs no network beyond PyPI, and CI fetches
  them once for the C++ job. No third-party source enters this repository.
- The pybind11 module keeps the name `depot_planner._cpp` and the existing
  `grid_astar(...)` signature, so step 1 is a move rather than a behaviour change; the
  Python-side tests that pin the two backends together are unchanged and still pass.
- `depot_core` is a static library with `POSITION_INDEPENDENT_CODE`, so the same objects
  link into the extension module, the GoogleTest binary and the benchmark binary. Nothing
  is compiled twice with different flags.
- The C++ tests are *unit* tests of the core's own invariants (cost accounting,
  neighbourhood, corner cutting, budgets, weighted-A* bound). Equivalence with the Python
  reference stays a pytest concern, because that is where both implementations can be run
  on the same inputs.
- Benchmarks build their scenarios in C++ rather than loading them from the Python
  generators, so `make cpp-bench` needs no interpreter and every run times the same work.
- `.clang-format` is Google style at 100 columns, matching the Python side's line length.
  `make format` rewrites in place; `make format-check` is the CI-friendly dry run.

## C++ step 2 — core types, templated search, distance field

- `Grid2D<T>` owns its storage and `GridView<T>` borrows somebody else's (a numpy buffer,
  or a `Grid2D`). Nothing in the core takes or returns a raw owning pointer: a numpy array
  is borrowed for the duration of a call, and anything the core produces is returned by
  value and copied into a fresh array at the binding boundary.
- **One templated `BestFirstSearch`, three heuristic functors.** `ZeroHeuristic` gives
  Dijkstra and `OctileHeuristic` gives A* and weighted A*, mirroring the Python module's
  single `search()` with three wrappers, so the variants cannot drift apart. The Python
  code reaches Dijkstra by multiplying the octile distance by a zero factor; because the
  octile distance is always finite and non-negative, `octile * 0.0` is exactly `0.0`, so
  skipping the distance entirely is numerically identical rather than merely equivalent.
- The heap comparator keeps the Python tuple order `(f, g, insertion counter)`. `heapq` is
  a min-heap and `std::priority_queue` a max-heap, but the counter makes the order total,
  so the two pop the same sequence.
- **The Euclidean distance transform is exact integer arithmetic.** The first pass reduces
  each column to the row distance to the nearest obstacle; the second takes the lower
  envelope of the resulting parabolas (Felzenszwalb-Huttenlocher). The usual formulation
  evaluates the parabola intersections in floating point; here they are kept as
  `(numerator, denominator)` pairs and compared by cross-multiplication, so every squared
  distance is exactly the integer scipy computes and the final square root is the only
  inexact operation. On 200 random masks the result is bit-identical to
  `scipy.ndimage.distance_transform_edt`.
- **One deliberate divergence, pinned by a test.** A mask with no obstacle at all has no
  nearest obstacle, so the core reports infinity. scipy returns finite distances measured
  from a point outside the array, which is an artefact of its algorithm rather than an
  answer. No map in this project is obstacle-free (every depot and every parking lot is
  walled), and infinity is what the proximity penalty wants anyway: it yields a zero
  penalty everywhere instead of a phantom one near a corner.
- `DijkstraField` mirrors the Python `dijkstra_field` down to its heap ordering
  `(distance, x, y)` and its `1e-12` relaxation epsilon. The epsilon means a marginally
  cheaper relaxation can be refused, which makes the field in principle order-dependent,
  so the order is copied rather than improved on.
- `depot_planner/core.py` is the single place that knows whether the extension exists and
  what `backend="auto"` means. `grid_astar/backend.py` keeps its own public helpers
  (`can_use_cpp`, `effective_cost_map`, ...) because the step-1 tests pin them.
- The C++ core is now the default for the distance field and the Dijkstra field as well as
  the grid search: `backend="python"` still selects scipy and the pure-Python expansions,
  and the equivalence tests run both.

## C++ step 3 — space-time A* and the snapshot baseline

- **Agent occupancy is a rectangle test, not a cached cell set.** The Python `AgentOccupancy`
  memoises a frozenset of cells per timestep because building it is expensive in Python; in
  C++ a footprint is an anchor plus a width and height, so "is this cell inside the margin"
  is four comparisons per agent. With at most sixteen agents that is cheaper than any cache
  and removes the per-timestep memory entirely. The rule is unchanged, only how it is
  evaluated.
- The open list keeps the Python tuple `(f, -g, counter)`, including the deliberate
  tie-break towards the *larger* g that walks deeper states first. `-0.0` and `0.0` compare
  equal in both languages, so the start state's entry behaves identically.
- **One record per state instead of four dictionaries.** The reference keeps `g_score`,
  `movement`, `parent` and `closed` keyed on the same `(x, y, t)` tuple. The core packs the
  state into a single `int64` (`t * cells + y * cols + x`) and stores one record per state
  in a flat open-addressing table, so an expansion does one lookup rather than four and the
  inner loop allocates nothing.
- **The baseline's heuristic scale is passed in, not derived.** `BaselineReplanPlanner`
  blocks the agents' margin by setting those cells to infinity and then calls step-1 A* with
  the *unblocked* grid's cheapest cell as the heuristic scale. Blocking can in principle
  remove every cell at that minimum, in which case the scale the core would derive from the
  blocked map disagrees with the reference. `GridSearchOptions::min_cell_cost` therefore
  takes the caller's value, with NaN meaning "derive it". This leaves the step-1 dispatch
  rule (`can_use_cpp`) and its tests untouched.
- The snapshot is built inside the core: one copy of the cost map, the agent rectangles
  written straight into it, the ego's own cell restored, then the search. The reference
  built a Python set of blocked cells and wrote them into two numpy arrays, which cost more
  than the search it was preparing.
- `collect_expanded` stays Python-only, exactly as it is for the step-1 search: the core
  does not report the expanded-cell set, so `backend="auto"` falls back to the reference for
  the calls that want it (the step-2 and step-3 figures). A test pins that.
- **The result: no divergence at all.** 1260 plan comparisons across both planners, both
  tiers, every battery seed and three start steps produce identical plans, costs, expansion
  counts and reasons — not one equal-cost tie-break difference. The 300-episode normal-tier
  battery is identical on every non-timing column.
- `scripts/check_equivalence.py` exists so that claim is reproducible rather than a note in
  a commit message; the test suite runs a fast slice of the same comparisons.

## C++ step 4 — hybrid A* and Reeds-Shepp

- **`math.hypot` is not the platform `hypot`, and that mattered.** CPython implements its
  own correctly-rounded norm rather than calling libm; over 200k random pairs `std::hypot`
  disagreed with it on 0.64% of them, always by one ulp. One ulp in the heuristic is enough
  to reorder the open list and return a different (equally good) path, so
  `depot::PythonHypot` reimplements CPython's `vector_norm` exactly. It is bit-identical to
  `math.hypot` on a million random pairs.
- **Python's `%` and `//` on floats are not C's.** `a % b` takes the sign of the divisor
  where `fmod` takes the sign of the dividend, and `a // b` is derived from the remainder
  and then corrected rather than being `floor(a / b)`. The heading bin uses the first and
  the heuristic's grid lookup the second, so `PythonMod` and `PythonFloorDiv` implement
  both. `math.fmod` *is* C's `fmod`, and the angle wrap uses that, so it needed nothing.
- **numpy's `cos` and `sin` were checked, not assumed.** The reference collision checker
  calls `np.cos`/`np.sin` on arrays while the primitives call `math.cos`/`math.sin`; on two
  million random angles the array versions agreed with the scalar ones to the last bit on
  this platform, so `std::cos` and `std::sin` serve both. (`np.tan` does *not* agree with
  `math.tan`, but nothing vectorises a tangent.)
- **The collision check runs on unwrapped headings.** The reference expands a fan of arcs
  as `theta + local_theta`, checks *that* array, and only wraps the heading when it stores
  the successor. `cos(theta)` and `cos(wrap(theta))` differ in the last bit, so the port
  keeps the same order: check unwrapped, store wrapped.
- The collision-check counter is part of the published results (`collision_checks` in the
  parking CSV), so the core counts the way the reference's vectorised call does: the whole
  fan of every expansion, whatever an early exit finds, plus the two endpoint checks.
- `min()` in Python returns the *first* of several equal minima, so the Reeds-Shepp
  candidate scan uses a strict `<` and runs in generation order. The word set, the four
  symmetries and the `1e-9` zero-length filter are all kept in their original order for the
  same reason.
- A Reeds-Shepp curve between two *identical* poses is a full 2*pi loop, not a zero-length
  path, because the zero-length filter removes every segment of the degenerate word and an
  empty candidate is never added. That is the reference's behaviour too; a GoogleTest case
  pins it, and hybrid A* never asks for it because it has already reached the goal by then.
- **The result: no divergence.** All 180 parking battery scenarios across both tiers give
  identical plans on both backends, down to the pose sequence, and every C++ plan passes
  the independent exact-rectangle checker. Success rates are therefore identical by
  construction, not merely equal in aggregate.
- The distance field and the coarse heuristic field stay in Python. They are scenario
  geometry rather than planning, the EDT behind them is already C++ (step 2), and building
  them once per scenario is not where the time goes.

## C++ step 5 — the pipeline runs on C++

- `backend=None` resolves through `depot_planner.core` to `"auto"`, which is the C++ core
  whenever it is built — and `pip install -e .` always builds it. `--backend python` on both
  battery scripts runs the reference end to end. Keeping `"auto"` rather than hard-coding
  `"cpp"` means an in-tree checkout with no build still runs, which is what the test suite
  needs when the extension is absent.
- A Python-backend run writes its CSV alongside the C++ one (`..._python.csv`) rather than
  overwriting it, so the two can be reported together instead of one replacing the other.
  Every battery row also carries a `backend` column.
- **The hard tier's headline result inverted, and that is the finding.** Under the same 50 ms
  per-replan budget and the same seeds, space-time A* goes from 63.3% / 50.0% on the Python
  reference to 100% / 100% on the core. The earlier conclusion — "the cheap myopic baseline
  beats optimal space-time A* under a real-time budget" — was a statement about the
  implementation, not the algorithm. Nothing was tuned; the planner, the scenarios, the
  seeds and the budget are unchanged.
- **Both backends are reported side by side** in REPORT.md section 2 and in README.md, and
  section 5 says plainly that the hard tier's numbers are a property of this machine and this
  implementation. Dropping the slow column would have hidden the only place in the project
  where runtime changes results.
- The normal tiers and both parking tiers are identical on every non-timing column, which is
  what the equivalence work predicted: their caps are on expansions, not wall clock.
- The generated README finding is rewritten from the numbers, as the earlier one was: it
  picks its headline by comparing the two backends' success rates rather than asserting a
  direction. If the port had made no difference it would say so.
- The report's failure section shrank from 62 runs to 38, and the 24 stale failure frames for
  space-time timeouts that no longer happen were deleted rather than left in the repository.
- The GIFs were regenerated and came out byte-identical, which is the expected result: they
  show normal-tier episodes, and those are unchanged by the backend.

## C++ step 6 — benchmarks and correctness tooling

- **Median, not mean or minimum.** Each case runs five times per backend. A mean is dragged
  around by the occasional scheduling hiccup on a shared machine, and a minimum flatters
  whichever backend happens to get the luckiest run; the median is the honest middle. The
  earlier step-7 benchmark used best-of-three, and it has been replaced rather than kept
  alongside, so the report has one timing table rather than two that disagree.
- **Setup is outside the clock.** Cost maps, drivable masks, cost-to-go fields, agent
  timelines and distance fields are built once per case and handed to both backends. What is
  timed is the search. Without that, the hybrid A* rows would mostly be measuring the same
  numpy distance-transform call twice.
- Every timed case also asserts the two backends returned the same plan, so the `identical
  plans` column in the table is produced by the benchmark rather than promised by prose. A
  speed-up bought by searching differently would show up here.
- **Two benchmark suites, deliberately.** `make cpp-bench` (Google Benchmark, scenarios built
  in C++) measures the core with no interpreter and no binding layer;
  `scripts/bench_cpp.py` measures what the project actually pays, bindings included. They
  answer different questions and the report says which is which.
- The benchmark's own traffic scenario was wrong on its first run: agents parked in both
  vertical connectors left the goal unreachable, so space-time A* exhausted a 400-step
  horizon and "benchmarked" 717k expansions of a hopeless search. The agents now pull into
  the parking band beside their lane once they have driven it, which is what the Python
  generator's traffic does. The giveaway was the grid and octile heuristics reporting the
  same expansion count.
- **The sanitizer job runs the GoogleTest suite, not a separate harness.** Those 59 cases
  already exercise every search loop, the open-addressing state table, the distance
  transform and every Reeds-Shepp word, so a sanitized build of them covers the core. Leak
  detection and stack-use-after-return are both on.
- CI grew from one job to three: the Python suite (which now asserts the core was actually
  built rather than silently testing the fallback), a C++ job that also runs
  `clang-format --Werror` and a benchmark smoke run, and the sanitizer job. The C++ jobs
  fetch GoogleTest and Google Benchmark at configure time; the Python job never does.

## C++ step 7 (optional) — the closed-loop runner

- **The Python loop stays the default.** It hands every executed step to
  `sim/collision.py` as the episode runs, so an illegal step stops the episode the moment it
  happens, checked by code that shares nothing with any planner. That is the project's
  strongest safety property and it is not worth trading for 1.6x. The C++ loop is opt-in:
  `run_episode(..., engine="cpp")` and `scripts/run_battery.py --engine cpp`.
- **The independent checker is still the auditor of record under either loop.** It is
  untouched, and `run_battery` re-checks every finished episode with it whichever loop ran,
  so a C++ episode that a planner believed was safe still has to survive it.
- The C++ loop needs *some* in-loop legality rule to know when to stop, so `runner.cpp`
  carries a transcription of the same rule set, written from the specification rather than
  from the planner's collision model. It is a third implementation, and what pins it to the
  Python one is the equivalence sweep: 420 episodes across both tiers and both planners
  agree on every metric, including `collision_step` and the formatted `collision_detail`.
- **The hard tier cannot be compared step for step, so it is compared without its budget.**
  Its 50 ms per-replan budget is wall clock, and one hard seed re-run four times on the
  *same* engine gave four different expansion counts and four different paths. The runner
  sweep therefore drops the budget for hard-tier scenarios, which keeps the harder geometry
  in the comparison and makes an exact comparison meaningful. That non-reproducibility was
  already documented in REPORT.md section 5; this is the same fact biting again.
- **1.6x, and the reason it is only 1.6x, is the interesting part.** Before the port, a
  battery episode was dominated by planning. Now the planners are 25-40x faster, and what is
  left of an episode is the per-step checker call, the list appends and the result
  marshalling — about 40% of it, and the C++ loop removes most of that. A whole normal-tier
  battery run moves from 5.9 s to 5.5 s, because scenario generation and the final audit are
  now the bulk of it. Optimising the loop further would mean moving scenario generation into
  C++, which is exactly the code the brief wants to stay in Python.
- `EpisodeResult` is rebuilt on the Python side from the core's raw fields, including
  constructing a `CollisionReport` and calling its `describe()`, so the reason strings come
  from one implementation rather than two that have to be kept in step.
- The C++ loop refuses a planner it does not implement, and refuses one pinned to
  `backend="python"`, rather than silently falling back to the Python loop.

## C++ step 8 — docs

- The README's "C++ core" section became an **architecture** section: a table of what runs
  where, then three short answers — why the split falls where it does, why the Python
  planners are kept, and why the two independent checkers are not moved next to the code
  they audit. A reader who wants to know whether this is a rewrite or a port should not have
  to read the commit log to find out.
- Build instructions say plainly that `pip install -e .` needs only a compiler, CMake and
  pybind11, and that GoogleTest and Google Benchmark are fetched by the *developer* build
  alone. That is the question someone cloning the repository actually has.
- The numerics paragraph is in the README rather than only in this file, because "the C++
  gives identical results" is the project's central claim and the three CPython-specific
  functions behind it (`math.hypot`, float `%`, float `//`) are the least obvious part of
  making it true.
- A fourth entry joins "Bugs found by the evaluation harness": the benchmark scenario that
  blocked both connectors and so measured 717k expansions of an unreachable search. It
  belongs there for the same reason the others do — it was caught by looking at a number
  that did not make sense, not by reading the code.
- The "hard tier is not bit-reproducible" limitation was rewritten rather than deleted. It is
  now the sharper statement: it is the one place where the implementation changes the result,
  which is why both backends are reported.
- `CLAUDE.md` gained the two rules this port ran on: never change the reference to make a
  port pass, and keep `sim/collision.py` independent of planner code.
