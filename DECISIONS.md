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
