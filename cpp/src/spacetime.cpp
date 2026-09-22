#include "depot/spacetime.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <queue>

#include "depot/state_table.hpp"

namespace depot {
namespace {

/// The eight grid moves plus the wait action, in the order ACTIONS lists them.
constexpr Move kActions[9] = {
    {1, 0, 1.0},     {-1, 0, 1.0},    {0, 1, 1.0},      {0, -1, 1.0}, {1, 1, kSqrt2},
    {1, -1, kSqrt2}, {-1, 1, kSqrt2}, {-1, -1, kSqrt2}, {0, 0, 0.0},
};

int Chebyshev(Cell a, Cell b) {
  return std::max(std::abs(a.x - b.x), std::abs(a.y - b.y));
}

/// One open-list entry. The Python heap pushes (f, -g, counter, state): ties on
/// f go to the *larger* g, which walks deeper states first, then to whichever
/// was pushed first.
struct Entry {
  double f;
  double neg_g;
  std::int64_t order;
  std::int64_t key;
};

struct EntryGreater {
  bool operator()(const Entry& a, const Entry& b) const {
    if (a.f != b.f) return a.f > b.f;
    if (a.neg_g != b.neg_g) return a.neg_g > b.neg_g;
    return a.order > b.order;
  }
};

SpaceTimeResult Failure(const std::string& reason, std::int64_t nodes, double runtime_ms) {
  SpaceTimeResult result;
  result.nodes_expanded = nodes;
  result.runtime_ms = runtime_ms;
  result.reason = reason;
  return result;
}

}  // namespace

SpaceTimeResult SpaceTimePlan(const CostMapView& cost, Cell start, Cell goal, int start_time,
                              const AgentOccupancy& agents, const CostMapView& heuristic_field,
                              const SpaceTimeOptions& options) {
  const auto began = std::chrono::steady_clock::now();
  const auto elapsed_ms = [&began]() {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - began)
        .count();
  };

  const int rows = cost.rows();
  const int cols = cost.cols();
  const auto drivable = [&cost](int x, int y) { return std::isfinite(cost.at(x, y)); };

  if (!cost.InBounds(start.x, start.y) || !drivable(start.x, start.y)) {
    return Failure("start not drivable", 0, elapsed_ms());
  }
  if (!cost.InBounds(goal.x, goal.y) || !drivable(goal.x, goal.y)) {
    return Failure("goal not drivable", 0, elapsed_ms());
  }
  if (start_time > options.horizon_cap) {
    return Failure("start time beyond the horizon", 0, elapsed_ms());
  }

  const bool has_field = !heuristic_field.empty();
  const auto heuristic = [&](Cell cell) {
    double h;
    if (!has_field) {
      h = Octile(cell, goal) * options.min_cell_cost;
    } else {
      h = heuristic_field.at(cell.x, cell.y);
      if (!std::isfinite(h)) return std::numeric_limits<double>::infinity();
    }
    if (options.heuristic_includes_time) h += Chebyshev(cell, goal) * options.time_cost;
    return h;
  };

  const std::int64_t cells = static_cast<std::int64_t>(rows) * cols;
  const auto key_of = [cells, cols](int x, int y, int t) {
    return static_cast<std::int64_t>(t) * cells + static_cast<std::int64_t>(y) * cols + x;
  };

  StateTable table(4096);
  const std::int64_t start_key = key_of(start.x, start.y, start_time);
  StateTable::Record& start_record = table.Emplace(start_key);
  start_record.g = 0.0;
  start_record.movement = 0.0;

  std::priority_queue<Entry, std::vector<Entry>, EntryGreater> open;
  open.push(Entry{heuristic(start), -0.0, 0, start_key});
  std::int64_t order = 0;
  std::int64_t nodes_expanded = 0;

  while (!open.empty()) {
    const Entry entry = open.top();
    open.pop();
    StateTable::Record& record = table.Emplace(entry.key);
    if (record.closed) continue;
    record.closed = true;
    ++nodes_expanded;

    const double g = -entry.neg_g;
    const int ct = static_cast<int>(entry.key / cells);
    const std::int64_t index = entry.key % cells;
    const int cx = static_cast<int>(index % cols);
    const int cy = static_cast<int>(index / cols);

    if (cx == goal.x && cy == goal.y) {
      SpaceTimeResult result;
      result.success = true;
      std::int64_t node = entry.key;
      while (node != -1) {
        const int t = static_cast<int>(node / cells);
        const std::int64_t at = node % cells;
        result.cells.push_back(Cell{static_cast<int>(at % cols), static_cast<int>(at / cols)});
        result.times.push_back(t);
        const StateTable::Record* parent = table.Find(node);
        node = parent == nullptr ? -1 : parent->parent;
      }
      std::reverse(result.cells.begin(), result.cells.end());
      std::reverse(result.times.begin(), result.times.end());
      for (std::size_t i = 0; i + 1 < result.cells.size(); ++i) {
        if (result.cells[i] == result.cells[i + 1]) ++result.wait_steps;
      }
      result.cost = g;
      result.movement_cost = record.movement;
      result.nodes_expanded = nodes_expanded;
      result.runtime_ms = elapsed_ms();
      result.reason = "goal reached";
      return result;
    }

    if (options.limits.max_nodes > 0 && nodes_expanded >= options.limits.max_nodes) {
      return Failure("node limit reached", nodes_expanded, elapsed_ms());
    }
    if (options.limits.time_limit_ms >= 0.0 && elapsed_ms() > options.limits.time_limit_ms) {
      return Failure("time limit reached", nodes_expanded, elapsed_ms());
    }
    if (ct >= options.horizon_cap) continue;

    const int next_t = ct + 1;
    const bool inside_margin = options.relax_margin_when_inside && agents.InMargin(cx, cy, ct);
    const double current_cost = cost.at(cx, cy);
    const double movement_so_far = record.movement;

    for (const Move& action : kActions) {
      const int nx = cx + action.dx;
      const int ny = cy + action.dy;
      if (nx < 0 || nx >= cols || ny < 0 || ny >= rows || !drivable(nx, ny)) continue;
      if (action.dx != 0 && action.dy != 0 && (!drivable(nx, cy) || !drivable(cx, ny))) {
        continue;  // no corner cutting
      }
      const bool forbidden =
          inside_margin ? agents.InBody(nx, ny, next_t) : agents.InMargin(nx, ny, next_t);
      if (forbidden) continue;
      if ((action.dx != 0 || action.dy != 0) && agents.Swaps(Cell{cx, cy}, Cell{nx, ny}, ct)) {
        continue;
      }
      const std::int64_t successor = key_of(nx, ny, next_t);
      StateTable::Record& next = table.Emplace(successor);
      if (next.closed) continue;
      const double move_cost = action.length * 0.5 * (current_cost + cost.at(nx, ny));
      const double tentative = g + move_cost + options.time_cost;
      const double h = heuristic(Cell{nx, ny});
      if (!std::isfinite(h)) continue;  // the goal is unreachable from there
      if (tentative < next.g - kRelaxEpsilon) {
        next.g = tentative;
        next.movement = movement_so_far + move_cost;
        next.parent = entry.key;
        ++order;
        open.push(Entry{tentative + h, -tentative, order, successor});
      }
    }
  }

  return Failure("no timed path to the goal", nodes_expanded, elapsed_ms());
}

SearchResult BaselineSnapshotPlan(const CostMapView& cost, Cell start, Cell goal, int t,
                                  const AgentOccupancy& agents, double min_cell_cost,
                                  const SearchLimits& limits) {
  const int rows = cost.rows();
  const int cols = cost.cols();
  CostMap snapshot(rows, cols);
  std::copy(cost.data(), cost.data() + cost.size(), snapshot.data());

  const int inflate = agents.inflate();
  for (const AgentTimeline& agent : agents.agents()) {
    const Cell anchor = agent.AnchorAt(t);
    const int x0 = std::max(0, anchor.x - inflate);
    const int x1 = std::min(cols, anchor.x + agent.width + inflate);
    const int y0 = std::max(0, anchor.y - inflate);
    const int y1 = std::min(rows, anchor.y + agent.height + inflate);
    for (int y = y0; y < y1; ++y) {
      for (int x = x0; x < x1; ++x) {
        snapshot.at(x, y) = std::numeric_limits<double>::infinity();
      }
    }
  }
  // The ego is where it is: never declare its own cell unusable.
  if (snapshot.InBounds(start.x, start.y)) {
    snapshot.at(start.x, start.y) = cost.at(start.x, start.y);
  }

  GridSearchOptions options;
  options.weight = 1.0;
  options.min_cell_cost = min_cell_cost;
  options.limits = limits;
  return GridSearch(CostMapView(snapshot), start, goal, options);
}

}  // namespace depot
