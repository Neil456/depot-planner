# Task: Depot Maneuvering Planners (A* → Space-Time A* → Hybrid A* Parking)

You are building a small, complete, well-tested Python project in one unattended run.
A working end-to-end pipeline matters more than sophistication.

## Ground rules (read first, follow always)

1. Work through the steps **in order**. A step is done only when its **Check** passes.
2. **Do not ask questions.** Make a reasonable choice and record it in `DECISIONS.md`
   (one line per decision: what you chose and why).
3. `pytest` must pass after every step. Commit after every completed step with message
   `step N: <summary>`.
4. Keep `PROGRESS.md` updated: a checklist of steps with status (done / partial / skipped)
   and a one-line note each. This is the first file the human will read in the morning.
5. **Never fake results.** Every number, table, and plot in the report must be produced by
   running the code. No hardcoded metrics, no expected values baked into the report.
6. If an **optional** step fails after ~3 serious attempts, revert it cleanly, mark it
   skipped in `PROGRESS.md`, explain why in `DECISIONS.md`, and move on.
   Required steps (1–6) must not be skipped.
7. Everything runs offline. No dataset downloads, no GPU.

## Environment

- Python 3.11
- Dependencies: `numpy`, `scipy`, `matplotlib`, `imageio`, `pyyaml`, `pandas`, `pytest`.
  Add others only if truly needed and record them in `DECISIONS.md`.
- Step 7 (optional) additionally uses `pybind11` and a C++17 compiler.
- Provide `pyproject.toml` (installable with `pip install -e .`) and a `Makefile`.

## World and units (use these unless impossible)

- Top-down 2D. Meters for distance, seconds for time.
- The setting is a vehicle **depot**: aisles, parking rows, parked cars, and other
  vehicles moving slowly through aisles.
- All randomness goes through a seeded `numpy.random.Generator`. Same seed → identical
  scenario, identical plan, identical metrics.

## Repo layout

```
depot_planner/
  world/          # maps, cost maps, scenario generators
  grid_astar/     # step 1
  spacetime/      # step 3
  hybrid/         # step 5
  sim/            # closed-loop runner, independent collision checker
  viz/            # top-down renderer, GIF writer
  eval/           # scenario battery, metrics, report generation
scripts/          # CLI entry points (one per Makefile target)
configs/          # YAML configs for every tunable parameter
tests/
results/          # generated, gitignored except results/README_assets/
REPORT.md         # generated
README.md, DECISIONS.md, PROGRESS.md, Makefile, pyproject.toml
```

All tunable numbers (costs, penalties, resolutions, limits) live in `configs/*.yaml`,
not in code.

---

## Step 1 — Grid A* with a driving cost map

**Build**
- `world/grid.py`: occupancy grid, 1.0 m cells. Cell types: aisle (drivable, cost 1.0),
  parking area (drivable, cost 3.0), obstacle/wall (not drivable).
- A cost map adds a soft penalty near obstacles: extra cost that decays with distance
  (use `scipy.ndimage.distance_transform_edt`), configurable.
- `grid_astar/search.py`: one generic best-first search supporting
  - Dijkstra (heuristic = 0)
  - A* (octile distance × minimum cell cost, which is admissible)
  - Weighted A* (heuristic × w, w configurable, default 1.5)
  - 8-connected moves; diagonal step length √2; step cost = length × mean cost of the two cells;
    no corner cutting through obstacles.
- Return: path, total cost, nodes expanded, runtime ms, and the set of expanded cells
  (for visualization).
- `world/depot_maps.py`: a deterministic generator for depot layouts
  (rectangular lot, 2–4 parallel aisles, parking rows between them, entry/exit gaps),
  roughly 60 × 40 m.

**Visualize**
- `viz/render.py`: draw the map, expanded cells as a faint heatmap, the path as a bold line,
  start and goal markers. Save PNG.

**Check**
- Tests: A* and Dijkstra return equal path cost on 50 random start/goal pairs;
  A* expands ≤ Dijkstra's node count; weighted A* cost ≤ w × optimal cost;
  unreachable goal returns a clean "no path" result; paths never touch obstacle cells.
- `scripts/demo_grid.py` writes `results/step1/compare.png` showing the three
  algorithms side by side on the same start/goal.

---

## Step 2 — Scenarios with moving vehicles

**Build**
- `world/agents.py`: other vehicles, each occupying a footprint of cells, following a
  **known, precomputed** timed path along aisles (they do not react to the ego).
  Timestep dt = 0.25 s. The ego moves at most one cell per step (4 m/s, depot speed).
  Agents move at most one cell per step and may pause.
- `world/scenarios.py`: seeded generators for these scenario types:
  - `empty`: no moving agents
  - `crossing`: an agent crosses the ego's natural route
  - `head_on`: an agent drives down the same aisle toward the ego
  - `blocked_then_clears`: an agent sits in the aisle, then drives away at a random time
  - `congested`: 6–10 agents moving through the depot
- A scenario = map + ego start + ego goal + agent timelines + time limit.
- Each scenario generator must verify that a solution exists in principle
  (for example, the goal is reachable once all agents are gone); regenerate otherwise.

**Check**
- Tests: determinism under seed; agents never overlap walls or each other;
  each scenario type generates 20 valid instances without error.

---

## Step 3 — Space-time A* (the planner can wait)

**Build**
- `spacetime/search.py`: A* over states (x, y, t).
  - Actions: the 8 moves plus **wait** (stay in the cell for one step).
  - Step cost = movement cost from step 1 + a per-step time cost (configurable), so
    waiting is allowed but not free.
  - Collision: ego cell may not be inside any agent footprint inflated by 1 cell at time t,
    and ego and agent may not swap cells between t and t+1.
  - Heuristic: step-1 A* heuristic (admissible).
  - Hard cap on time horizon and node count (configurable); return "no path" cleanly.
  - Closed set keyed on (x, y, t).
- **Baseline planner** for comparison: plain grid A* that treats agents as static obstacles
  at their *current* positions and replans every step.

**Check**
- Tests: in a hand-built corridor where an agent passes through, space-time A* waits and
  then proceeds, with zero collisions; with no agents its path cost equals step-1 A*;
  returned plans pass the independent collision checker from step 4.

---

## Step 4 — Closed-loop simulator and scenario battery

**Build**
- `sim/collision.py`: an **independent** collision checker that does not reuse planner code.
  It checks the executed ego trajectory against agent footprints at each timestep.
- `sim/runner.py`: closed-loop execution. Every `replan_every` steps (configurable, default 4)
  the planner plans from the ego's current state; the ego executes the plan's next steps;
  agents follow their timelines. The episode ends on reaching the goal, collision, or timeout.
- Metrics per episode: success, collision, time to goal (s), path length (m),
  number of wait steps, total nodes expanded, mean and max planning time (ms).
- `eval/battery.py`: run **both** planners (space-time A*, baseline) on the same seeded
  scenarios: 30 per scenario type. Write `results/battery_spacetime.csv` with one row
  per episode.

**Visualize**
- `viz/animate.py`: GIF of one episode: map, agents as colored rectangles, ego highlighted,
  planned path drawn ahead of the ego and updated at each replan, a small text overlay
  with time and planner name. Target under 5 MB per GIF.
- Save one GIF per scenario type for each planner under `results/gifs/`.

**Check**
- The battery completes. Space-time A* episodes marked successful have zero collisions
  according to the independent checker.
- If the independent checker finds a collision the planner believed was safe, that is a
  bug: fix it, do not hide it.

---

## Step 5 — Hybrid A* parking (car-shaped, forward and reverse)

**Build**
- Car model: kinematic bicycle, wheelbase 2.7 m, footprint 4.5 × 1.9 m,
  maximum steering angle 0.6 rad.
- State (x, y, heading), continuous. For duplicate detection, discretize to 0.5 m cells
  and 72 heading bins (5° each).
- Motion primitives: arc length 1.0 m; steering in {−max, −max/2, 0, +max/2, +max};
  forward and reverse. Integrate each arc in small sub-steps and collision-check each one.
- Costs: arc length; multiplier for reverse (default 2.0); penalty for switching
  direction (default 5.0); small penalty for steering and for changing steering.
  All configurable.
- Heuristic: max(Euclidean distance, obstacle-aware 2D grid distance from the goal,
  precomputed once per scenario with Dijkstra on the step-1 grid code).
- Collision checking: cover the car footprint with 3–4 discs; check each disc against
  a Euclidean distance field of the obstacle map at 0.1 m resolution.
- Goal reached when within 0.3 m and 5° of the goal pose.
- Expansion cap and time limit (configurable); return "no path" cleanly.
- **Optional within this step:** a Reeds-Shepp analytic expansion to the goal, attempted
  every N expansions. Implement it only after the base search passes all checks,
  and drop it per rule 6 if it causes failures.
- `world/parking.py`: seeded parking scenarios:
  - `perpendicular_forward`: pull forward into a spot between parked cars
  - `perpendicular_reverse`: back into a spot (goal heading requires reverse)
  - `parallel`: parallel park between two cars, with gap length varied
  - `tight`: any of the above with minimal clearance
  Parked cars are static obstacles. There are no moving agents in this step.

**Visualize**
- GIF of the car executing the plan: the footprint drawn along the path,
  forward segments and reverse segments in different colors, the explored states as faint
  dots in the first frame.
- Save one GIF per parking scenario type under `results/gifs/parking/`.

**Check**
- Tests: straight-line goal in open space is found with no reverse; a reverse-in scenario
  uses at least one reverse segment; densely interpolating the final path with the
  independent checker finds no collisions; heading wraparound (−π/π) is handled.
- Battery: 30 scenarios per parking type → `results/battery_parking.csv` with success,
  planning time, nodes expanded, path length, number of direction switches.

---

## Step 6 — Report and README

**Build**
- `scripts/make_report.py` generates `REPORT.md` entirely from the CSVs:
  1. Step 1 table: Dijkstra vs A* vs weighted A* (path cost, nodes expanded, runtime)
     over 100 start/goal pairs.
  2. Space-time table: per scenario type × planner, showing success %, collisions,
     mean time to goal, mean wait steps, mean and max planning ms.
  3. Parking table: per parking type, showing success %, mean planning ms,
     mean direction switches.
  4. Two plots: nodes expanded by algorithm (step 1), and planning time
     distribution for space-time vs baseline.
  5. A "Failure cases" section listing the seed and scenario type of every failed
     episode, with a PNG of the frame where it failed.
- Copy the three best GIFs (a space-time A* waiting for traffic, the baseline failing
  or colliding on the same scenario, and a reverse parking maneuver) to
  `results/README_assets/`, and embed them at the top of `README.md`.
- `README.md`: a plain-English explanation of what each planner does, how to run
  everything, and a link to `REPORT.md`. Do not claim results beyond what the report shows.

**Check**
- `make all` from a clean clone runs setup, tests, all batteries, all GIFs, and the report
  in under 30 minutes on a laptop CPU. Record the actual runtime in `PROGRESS.md`.

---

## Step 7 (optional) — C++ core for grid A*

**Build**
- `cpp/grid_astar.cpp`: a C++17 implementation of the step-1 search
  (octile heuristic, 8-connected, same cost rules), exposed to Python with pybind11
  as `depot_planner._cpp.grid_astar(cost_grid, start, goal, weight)`.
- Build via `scikit-build-core` or a `setup.py` extension; `pip install -e .` must build it.
- Python falls back to the pure-Python search if the extension is not built.

**Check**
- Test: on 200 random start/goal pairs the C++ and Python versions return identical
  path costs (within 1e-9) and valid paths.
- `scripts/bench_cpp.py` adds a "Python vs C++ runtime" table to `REPORT.md`.
- If the build does not work after ~3 attempts, follow rule 6.

---

## Makefile targets

```
setup      pip install -e .[dev]
test       pytest -q
step1      scripts/demo_grid.py
battery    both batteries (space-time and parking)
gifs       all GIFs
report     scripts/make_report.py
all        setup test step1 battery gifs report
```

## Definition of done

- [ ] Steps 1–6 complete, `pytest` green, `make all` succeeds from a clean clone
- [ ] `REPORT.md` generated from real runs, including the failure-case section
- [ ] README shows three GIFs and explains the project in plain English
- [ ] `PROGRESS.md` and `DECISIONS.md` are accurate
- [ ] Step 7 either done with the equivalence test passing, or marked skipped with a reason
