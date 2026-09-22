#include "depot/runner.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include "depot/numeric.hpp"

namespace depot {
namespace {

/// The verdict on one executed step. A C++ transcription of the rule set in
/// depot_planner/sim/collision.py: the cell must be in bounds and drivable and
/// clear of every agent *body* (no safety margin), the move must be at most one
/// cell in each axis and advance time by exactly one step, a diagonal move may
/// not cut an obstacle corner, and the ego may not swap cells with an agent.
struct StepVerdict {
  bool ok = true;
  std::string kind;
  int step = 0;  // index within the two-step window
  int time = 0;
  Cell cell{};
  int agent = -1;
  bool has_agent = false;
};

StepVerdict Illegal(const std::string& kind, int step, int time, Cell cell, int agent = -1,
                    bool has_agent = false) {
  StepVerdict verdict;
  verdict.ok = false;
  verdict.kind = kind;
  verdict.step = step;
  verdict.time = time;
  verdict.cell = cell;
  verdict.agent = agent;
  verdict.has_agent = has_agent;
  return verdict;
}

/// Check the two-step window (source at t0, target at t1), as the reference's
/// runner does: first every cell on its own, then the transition.
StepVerdict CheckWindow(const CostMapView& cost, const AgentOccupancy& agents, Cell source, int t0,
                        Cell target, int t1) {
  const auto obstacle = [&cost](Cell cell) { return !std::isfinite(cost.at(cell.x, cell.y)); };

  const Cell window_cells[2] = {source, target};
  const int window_times[2] = {t0, t1};
  for (int index = 0; index < 2; ++index) {
    const Cell cell = window_cells[index];
    const int t = window_times[index];
    if (!cost.InBounds(cell.x, cell.y)) return Illegal("out_of_bounds", index, t, cell);
    if (obstacle(cell)) return Illegal("obstacle", index, t, cell);
    for (const AgentTimeline& agent : agents.agents()) {
      const Cell anchor = agent.AnchorAt(t);
      if (cell.x >= anchor.x && cell.x < anchor.x + agent.width && cell.y >= anchor.y &&
          cell.y < anchor.y + agent.height) {
        return Illegal("overlap", index, t, cell, agent.id, true);
      }
    }
  }

  const int dx = target.x - source.x;
  const int dy = target.y - source.y;
  if (std::abs(dx) > 1 || std::abs(dy) > 1 || t1 != t0 + 1) {
    return Illegal("jump", 0, t0, target);
  }
  if (dx != 0 && dy != 0) {
    if (obstacle(Cell{target.x, source.y}) || obstacle(Cell{source.x, target.y})) {
      return Illegal("corner_cut", 0, t0, target);
    }
  }
  for (const AgentTimeline& agent : agents.agents()) {
    const Cell now = agent.AnchorAt(t0);
    const Cell next = agent.AnchorAt(t1);
    const bool target_now = target.x >= now.x && target.x < now.x + agent.width &&
                            target.y >= now.y && target.y < now.y + agent.height;
    const bool source_next = source.x >= next.x && source.x < next.x + agent.width &&
                             source.y >= next.y && source.y < next.y + agent.height;
    if (target_now && source_next) return Illegal("swap", 0, t0, target, agent.id, true);
  }
  return StepVerdict{};
}

/// One plan handed back to the loop, in the shape PlanResult has in Python.
struct PlanOutcome {
  bool success = false;
  Path cells;
  std::vector<int> times;
  std::int64_t nodes_expanded = 0;
  double runtime_ms = 0.0;
  std::string reason;
};

PlanOutcome PlanFrom(const CostMapView& cost, Cell state, Cell goal, int t,
                     const AgentOccupancy& agents, const CostMapView& heuristic_field,
                     const EpisodeOptions& options) {
  PlanOutcome outcome;
  if (options.planner == PlannerKind::kSpaceTime) {
    const SpaceTimeResult result =
        SpaceTimePlan(cost, state, goal, t, agents, heuristic_field, options.spacetime);
    outcome.nodes_expanded = result.nodes_expanded;
    outcome.runtime_ms = result.runtime_ms;
    outcome.reason = result.reason;
    if (result.success) {
      outcome.success = true;
      outcome.cells = result.cells;
      outcome.times = result.times;
    } else {
      outcome.cells = {state};
      outcome.times = {t};
    }
    return outcome;
  }

  const auto began = std::chrono::steady_clock::now();
  const SearchResult result = BaselineSnapshotPlan(cost, state, goal, t, agents,
                                                   options.min_cell_cost, options.baseline_limits);
  outcome.nodes_expanded = result.nodes_expanded;
  outcome.runtime_ms =
      std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - began).count();
  outcome.reason = result.reason;
  if (result.success) {
    outcome.success = true;
    outcome.cells = result.path;
    outcome.times.reserve(result.path.size());
    for (std::size_t i = 0; i < result.path.size(); ++i)
      outcome.times.push_back(t + static_cast<int>(i));
    return outcome;
  }
  if (options.baseline_hold_on_failure) {
    // Blocked right now: hold position for one step and try again next step.
    outcome.success = true;
    outcome.cells = {state, state};
    outcome.times = {t, t + 1};
    outcome.reason = "blocked, holding position";
    return outcome;
  }
  outcome.cells = {state};
  outcome.times = {t};
  return outcome;
}

void RecordFailure(std::vector<std::pair<std::string, int>>* reasons, const std::string& reason) {
  for (auto& entry : *reasons) {
    if (entry.first == reason) {
      ++entry.second;
      return;
    }
  }
  reasons->emplace_back(reason, 1);
}

}  // namespace

EpisodeOutcome RunEpisode(const CostMapView& cost, Cell start, Cell goal,
                          const AgentOccupancy& agents, const CostMapView& heuristic_field,
                          const EpisodeOptions& options) {
  EpisodeOutcome outcome;
  Cell state = start;
  int t = 0;
  outcome.cells.push_back(state);
  outcome.times.push_back(t);

  std::vector<double> timings;
  Path plan_cells;
  std::size_t plan_index = 0;
  int steps_since_replan = options.replan_every;  // force a plan on the first iteration
  bool aborted = false;

  while (t < options.time_limit && state != goal) {
    const bool need_plan =
        steps_since_replan >= options.replan_every || plan_index + 1 >= plan_cells.size();
    if (need_plan) {
      const PlanOutcome plan = PlanFrom(cost, state, goal, t, agents, heuristic_field, options);
      ++outcome.replans;
      outcome.nodes_expanded += plan.nodes_expanded;
      timings.push_back(plan.runtime_ms);
      if (plan.success && plan.cells.size() > 1) {
        plan_cells = plan.cells;
        plan_index = 0;
        if (options.record_plans) outcome.plans.push_back(IssuedPlan{t, plan.cells});
      } else {
        ++outcome.plan_failures;
        RecordFailure(&outcome.plan_failure_reasons, plan.reason.empty() ? "no plan" : plan.reason);
        if (!options.hold_on_plan_failure) {
          outcome.reason_override = "no plan and holding disabled";
          aborted = true;
          break;
        }
        plan_cells = {state, state};
        plan_index = 0;
        if (options.record_plans) outcome.plans.push_back(IssuedPlan{t, Path{state}});
      }
      steps_since_replan = 0;
    }

    ++plan_index;
    const Cell next_state = plan_index < plan_cells.size() ? plan_cells[plan_index] : state;
    const Cell previous = state;
    const int previous_t = t;
    ++t;
    state = next_state;
    outcome.cells.push_back(state);
    outcome.times.push_back(t);
    ++steps_since_replan;

    const StepVerdict verdict = CheckWindow(cost, agents, previous, previous_t, state, t);
    if (!verdict.ok) {
      // The checker saw a two-step window; translate its index back to the
      // position in the full trajectory.
      outcome.collision = true;
      outcome.collision_step = static_cast<int>(outcome.cells.size()) - 2 + verdict.step;
      outcome.collision_time = verdict.time;
      outcome.collision_kind = verdict.kind;
      outcome.collision_cell = verdict.cell;
      outcome.collision_agent = verdict.agent;
      outcome.collision_has_agent = verdict.has_agent;
      break;
    }
  }

  outcome.steps = static_cast<int>(outcome.cells.size()) - 1;
  for (std::size_t i = 0; i + 1 < outcome.cells.size(); ++i) {
    outcome.path_length_cells += PythonHypot(outcome.cells[i + 1].x - outcome.cells[i].x,
                                             outcome.cells[i + 1].y - outcome.cells[i].y);
    if (outcome.cells[i] == outcome.cells[i + 1]) ++outcome.wait_steps;
  }
  if (!timings.empty()) {
    double total = 0.0;
    for (double value : timings) total += value;
    outcome.mean_planning_ms = total / static_cast<double>(timings.size());
    outcome.max_planning_ms = *std::max_element(timings.begin(), timings.end());
  }
  // Matching the reference: an abort is not a success and not a collision, so it
  // falls through to a timeout carrying its own reason.
  (void)aborted;
  outcome.success = !outcome.collision && outcome.cells.back() == goal;
  outcome.timeout = !outcome.collision && !outcome.success;
  return outcome;
}

}  // namespace depot
