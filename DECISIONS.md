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
