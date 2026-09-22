// Closed-loop execution of a planner on a scenario (depot_planner/sim/runner.py).
//
// Every `replan_every` steps the planner is asked for a plan from the ego's
// current state; the ego then executes the next steps of that plan while the
// agents follow their fixed timelines. The episode ends when the ego reaches
// the goal, when a step is found to be illegal, or at the time limit.
//
// The legality check in the loop is a C++ transcription of the rule set in
// depot_planner/sim/collision.py, written against the same specification and
// sharing no code with the planners. The Python checker remains the auditor of
// record: it is untouched, it re-checks every finished episode, and the
// equivalence tests compare this loop's metrics against the Python loop's on
// the same scenarios, which is what pins the two checks to each other.

#ifndef DEPOT_RUNNER_HPP_
#define DEPOT_RUNNER_HPP_

#include <string>
#include <utility>
#include <vector>

#include "depot/spacetime.hpp"
#include "depot/types.hpp"

namespace depot {

/// Which planner the episode drives with.
enum class PlannerKind { kSpaceTime, kBaselineSnapshot };

/// Everything configs/sim.yaml, configs/spacetime.yaml and the scenario control.
struct EpisodeOptions {
  int replan_every = 4;
  bool record_plans = true;
  bool hold_on_plan_failure = true;
  int time_limit = 160;

  PlannerKind planner = PlannerKind::kSpaceTime;
  SpaceTimeOptions spacetime;

  /// The baseline holds position for one step when it finds no path.
  bool baseline_hold_on_failure = true;
  SearchLimits baseline_limits;
  /// Heuristic scale for the baseline's A*, taken from the unblocked grid.
  double min_cell_cost = 1.0;
};

/// One plan the runner issued, and the step it was issued at.
struct IssuedPlan {
  int issued_at = 0;
  Path cells;
};

/// What the loop observed. Metric names match EpisodeResult in runner.py.
struct EpisodeOutcome {
  bool success = false;
  bool collision = false;
  bool timeout = false;
  int steps = 0;
  Path cells;
  std::vector<int> times;
  std::vector<IssuedPlan> plans;
  int wait_steps = 0;
  std::int64_t nodes_expanded = 0;
  int replans = 0;
  int plan_failures = 0;
  double mean_planning_ms = 0.0;
  double max_planning_ms = 0.0;
  /// Geometric path length in cells; the caller scales by the grid resolution.
  double path_length_cells = 0.0;
  std::string reason_override;

  /// The first illegal step, when there was one.
  int collision_step = -1;
  int collision_time = -1;
  std::string collision_kind;
  Cell collision_cell{};
  int collision_agent = -1;
  bool collision_has_agent = false;

  /// Plan-failure reasons in the order they were first seen, with counts.
  std::vector<std::pair<std::string, int>> plan_failure_reasons;
};

/// Run one closed-loop episode.
///
/// `heuristic_field` is the cost-to-go field the space-time planner uses; pass
/// an empty view to fall back to the octile heuristic.
EpisodeOutcome RunEpisode(const CostMapView& cost, Cell start, Cell goal,
                          const AgentOccupancy& agents, const CostMapView& heuristic_field,
                          const EpisodeOptions& options);

}  // namespace depot

#endif  // DEPOT_RUNNER_HPP_
