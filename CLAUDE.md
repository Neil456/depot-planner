# Instructions for Claude Code

- The original project spec is in docs/TASK.md and the follow-up C++ port spec in
  docs/TASK_CPP.md. Follow them exactly and in order.
- Only do the steps the session prompt asks for (for example "steps 1-4").
- Run `pytest -q` before finishing any step; do not finish with failing tests. If the
  step touches the C++ core, run `make cpp-test` too.
- Commit after each completed step with message `step N: <summary>`.
- Keep docs/PROGRESS.md and docs/DECISIONS.md up to date as docs/TASK.md describes.
- Never hardcode results; every number in REPORT.md must come from running the code.
- If dependencies are missing, add them to pyproject.toml and note it in docs/DECISIONS.md.
- The Python planners are the reference implementation: never change them to make a C++
  port pass. Keep depot_planner/sim/collision.py independent of all planner code.
