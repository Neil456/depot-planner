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
