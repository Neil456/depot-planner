# Task: make the planning core C++ (overnight, unattended)

Goal: move all performance-critical planning code into a real C++ library, with
Python used only for scenario generation, orchestration, the independent
collision checker, plotting, and reports. Preserve behavior exactly.

## Ground rules
- Work in order; each step must pass its check before moving on. Commit per step.
- Do not ask questions. Record choices in docs/DECISIONS.md; keep docs/PROGRESS.md current.
- The existing Python planners stay as the **reference implementation**. Every C++
  port must be verified against them. Never change the reference to make tests pass.
- Do NOT vendor third-party source into the repo (fetch GoogleTest / Google Benchmark
  via CMake FetchContent). The language stats must reflect code written for this project.
- Keep sim/collision.py (the independent checker) in Python and untouched, so it stays
  independent of planner code.
- Never hand-edit results; regenerate everything from code.
- If an optional step fails after ~3 serious attempts, revert cleanly and mark it skipped.

## Target layout
```
cpp/
  CMakeLists.txt
  include/depot/   # public headers
  src/             # implementations
  tests/           # GoogleTest
  bench/           # Google Benchmark
  bindings/        # pybind11 module
```
C++17 (C++20 only if clearly useful). No raw owning pointers; RAII throughout.

## Step 1: C++ project foundation
- CMake project producing a static library `depot_core`, a pybind11 module, a
  GoogleTest binary, and a benchmark binary. Build via scikit-build-core so
  `pip install -e .` builds it.
- Add .clang-format and a `make format` target. Add a `make cpp-test` target running ctest.
- Move the existing C++ grid A* into this structure.
- Check: `pip install -e .`, ctest, and pytest all pass.

## Step 2: shared core types and grid search
- C++ types for grid maps, cost maps, poses, paths, and search results.
- A single templated best-first search (Dijkstra / A* / weighted A*), with the
  existing tie-breaking preserved.
- The obstacle distance field computed in C++.
- Check: GoogleTest unit tests, plus pytest equivalence on 200 random cases
  (identical costs, paths, and node counts).

## Step 3: space-time A* in C++
- Port states (x, y, t), the wait action, the margin and swap collision rules,
  the time cost, the Dijkstra cost-to-go heuristic, the node and time caps, and
  the per-search time limit.
- Port the static-snapshot baseline planner.
- Check: equivalence on all battery seeds (identical plans or a documented
  tie-break difference with equal cost). The full closed-loop battery produces
  identical metrics with the C++ planner, apart from planning time.

## Step 4: hybrid A* in C++
- Port the kinematic bicycle model, motion primitives, heading discretization,
  cost terms, the disc-based collision check against the distance field, and
  the heuristic.
- Port Reeds-Shepp analytic expansion with its own unit tests (validate path
  endpoints and lengths numerically, as the Python version was validated).
- Check: every C++ plan on the parking battery passes the Python independent
  checker. Success rates must match the Python reference exactly. Any divergence
  is investigated and explained in DECISIONS.md, never hidden.

## Step 5: switch the pipeline to C++
- All planners default to the C++ backend, with `--backend python` available
  for the reference.
- Rerun all batteries (normal and hard tier) with the C++ backend and regenerate
  REPORT.md.
- Hard-tier outcomes may legitimately change, because faster planning under a
  50 ms budget is a real effect. Report both backends side by side and explain
  the differences.

## Step 6: benchmarks and correctness tooling
- Google Benchmark suite: grid A*, space-time A*, and hybrid A* on fixed
  scenarios. Add a "Python vs C++" table to REPORT.md and README.md
  (median of repeated runs, same inputs, setup excluded).
- CI: build the C++ code, run ctest and pytest, and run one job with
  AddressSanitizer and UndefinedBehaviorSanitizer enabled.

## Step 7 (optional): C++ closed-loop runner
- Port the closed-loop simulation loop, so a whole episode runs in C++ and
  Python only loads scenarios and writes results. Check that the metrics
  match the Python runner.

## Step 8: docs
- Update the README with the architecture (what's C++, what's Python, and why),
  build instructions, and the benchmark table.
- Keep the existing GIFs and results sections accurate after regeneration.

## Definition of done
- [ ] All planners run in C++ by default; Python reference retained and tested against
- [ ] Batteries regenerated with the C++ backend; REPORT.md updated
- [ ] GoogleTest + pytest + sanitizer CI all green
- [ ] Benchmark table in the README
- [ ] PROGRESS.md and DECISIONS.md accurate
