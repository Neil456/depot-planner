// Space-time A*: A* over states (x, y, t) where the ego may also wait, plus the
// static-snapshot baseline it is compared against.
//
// This is the C++ core of depot_planner/spacetime/search.py and of
// BaselineReplanPlanner in depot_planner/spacetime/planners.py. Actions are the
// eight grid moves of the step-1 search plus a wait that keeps the ego in its
// cell for one step; every action advances time by one step and is charged the
// step-1 movement cost plus a per-step time cost.
//
// Collision rules, matching the reference exactly:
//   * the ego may not enter a cell inside an agent footprint inflated by
//     `inflate` cells at time t + 1;
//   * the ego and an agent may not swap cells between t and t + 1;
//   * if the ego already stands inside the inflated margin the margin is
//     relaxed for one move, but an agent body is never enterable.

#ifndef DEPOT_SPACETIME_HPP_
#define DEPOT_SPACETIME_HPP_

#include <string>
#include <vector>

#include "depot/grid_search.hpp"
#include "depot/types.hpp"

namespace depot {

/// One other vehicle: a rectangular footprint on a fixed timeline of anchors.
/// Beyond the end of the timeline the agent stays where it stopped.
struct AgentTimeline {
  std::vector<Cell> anchors;
  int width = 2;
  int height = 2;

  Cell AnchorAt(int t) const {
    if (anchors.empty()) return Cell{};
    const int last = static_cast<int>(anchors.size()) - 1;
    const int index = t < 0 ? 0 : (t > last ? last : t);
    return anchors[static_cast<std::size_t>(index)];
  }
};

/// What the agents cover, and when. Membership is a rectangle test rather than
/// a cached cell set: the footprints are rectangles, so there is nothing to
/// look up.
class AgentOccupancy {
 public:
  AgentOccupancy() = default;
  AgentOccupancy(std::vector<AgentTimeline> agents, int inflate)
      : agents_(std::move(agents)), inflate_(inflate) {}

  const std::vector<AgentTimeline>& agents() const { return agents_; }
  int inflate() const { return inflate_; }

  /// True if (x, y) is covered by an agent body at step t.
  bool InBody(int x, int y, int t) const { return Covered(x, y, t, 0); }

  /// True if (x, y) is covered by an agent body grown by the safety margin.
  bool InMargin(int x, int y, int t) const { return Covered(x, y, t, inflate_); }

  /// True if the ego and an agent would trade places between t and t + 1.
  bool Swaps(Cell source, Cell target, int t) const {
    for (const AgentTimeline& agent : agents_) {
      if (Inside(agent, agent.AnchorAt(t), target.x, target.y, 0) &&
          Inside(agent, agent.AnchorAt(t + 1), source.x, source.y, 0)) {
        return true;
      }
    }
    return false;
  }

 private:
  static bool Inside(const AgentTimeline& agent, Cell anchor, int x, int y, int grow) {
    return x >= anchor.x - grow && x < anchor.x + agent.width + grow && y >= anchor.y - grow &&
           y < anchor.y + agent.height + grow;
  }

  bool Covered(int x, int y, int t, int grow) const {
    for (const AgentTimeline& agent : agents_) {
      if (Inside(agent, agent.AnchorAt(t), x, y, grow)) return true;
    }
    return false;
  }

  std::vector<AgentTimeline> agents_;
  int inflate_ = 1;
};

/// Everything configs/spacetime.yaml controls, plus the caller's caps.
struct SpaceTimeOptions {
  double time_cost = 0.05;
  bool relax_margin_when_inside = true;
  bool heuristic_includes_time = true;
  /// Scale for the octile heuristic, used only when no cost-to-go field is given.
  double min_cell_cost = 1.0;
  /// Hard cap on t.
  int horizon_cap = 700;
  SearchLimits limits;
};

/// Outcome of one space-time search.
struct SpaceTimeResult {
  bool success = false;
  Path cells;
  std::vector<int> times;
  double cost = std::numeric_limits<double>::infinity();
  double movement_cost = std::numeric_limits<double>::infinity();
  int wait_steps = 0;
  std::int64_t nodes_expanded = 0;
  double runtime_ms = 0.0;
  std::string reason;
};

/// Plan a timed path from `start` at `start_time` to `goal`.
///
/// `heuristic_field` is the exact step-1 cost-to-go to the goal on the static
/// map (see DijkstraField). Pass an empty view to use the octile heuristic
/// scaled by `options.min_cell_cost` instead.
SpaceTimeResult SpaceTimePlan(const CostMapView& cost, Cell start, Cell goal, int start_time,
                              const AgentOccupancy& agents, const CostMapView& heuristic_field,
                              const SpaceTimeOptions& options);

/// The replanning baseline: grid A* with the agents frozen where they stand at
/// step `t`, their footprints inflated by `agents.inflate()`.
///
/// The ego's own cell is never declared unusable, whatever the margin says.
SearchResult BaselineSnapshotPlan(const CostMapView& cost, Cell start, Cell goal, int t,
                                  const AgentOccupancy& agents, double min_cell_cost,
                                  const SearchLimits& limits);

}  // namespace depot

#endif  // DEPOT_SPACETIME_HPP_
