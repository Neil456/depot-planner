// One generic best-first search over an 8-connected grid: the C++ core of the
// step-1 planner (depot_planner/grid_astar/search.py).
//
// The same routine implements Dijkstra (ZeroHeuristic), A* (OctileHeuristic
// with weight 1) and weighted A* (weight > 1), exactly as the Python module
// does with one `search()` and three wrappers.
//
// It reproduces the Python search exactly: the same neighbourhood in the same
// order, the same "length x mean of the two cell costs" step cost, the same
// no-corner-cutting rule, the same octile heuristic scaled by the cheapest
// drivable cell, and the same tie-breaking (lowest f, then lowest g, then
// insertion order). Given the same inputs it returns the same path, not merely
// the same cost.

#ifndef DEPOT_GRID_SEARCH_HPP_
#define DEPOT_GRID_SEARCH_HPP_

#include <algorithm>
#include <chrono>
#include <cmath>
#include <queue>
#include <vector>

#include "depot/types.hpp"

namespace depot {

/// One (dx, dy) step and its geometric length.
struct Move {
  int dx;
  int dy;
  double length;
};

constexpr double kSqrt2 = 1.4142135623730951;

/// The relaxation epsilon the Python search uses when comparing g values.
constexpr double kRelaxEpsilon = 1e-12;

/// The 8-connected neighbourhood, in the same order as MOVES in search.py.
extern const Move kMoves[8];

/// Octile distance in cell units.
double Octile(Cell a, Cell b);

/// Cheapest finite cell in a cost map; the scale that keeps the octile
/// heuristic admissible. Zero when nothing is drivable.
double MinFiniteCost(const CostMapView& cost);

/// h = 0: turns the search into Dijkstra.
struct ZeroHeuristic {
  double operator()(Cell) const { return 0.0; }
};

/// h = octile distance to the goal, scaled by `factor`.
struct OctileHeuristic {
  Cell goal;
  double factor = 0.0;

  double operator()(Cell cell) const { return Octile(cell, goal) * factor; }
};

/// Weight applied to the octile heuristic. Zero gives Dijkstra, one gives A*,
/// more than one gives weighted A*.
struct GridSearchOptions {
  double weight = 1.0;
  /// Scale the octile heuristic is multiplied by, before the weight. NaN means
  /// "the cheapest drivable cell of this map", which is what the step-1 search
  /// derives when its caller does not supply one.
  double min_cell_cost = std::numeric_limits<double>::quiet_NaN();
  SearchLimits limits;
};

namespace detail {

/// One open-list entry. `order` is the insertion counter that makes the pop
/// sequence reproducible.
struct OpenEntry {
  double f;
  double g;
  std::int64_t order;
  std::int64_t index;
};

/// Mirrors the Python heap, which pushes the tuple (f, g, counter, cell): ties
/// on f go to the smaller g, then to whichever was pushed first.
struct OpenEntryGreater {
  bool operator()(const OpenEntry& a, const OpenEntry& b) const {
    if (a.f != b.f) return a.f > b.f;
    if (a.g != b.g) return a.g > b.g;
    return a.order > b.order;
  }
};

inline SearchResult Failure(const std::string& reason, std::int64_t expanded, double runtime_ms) {
  SearchResult result;
  result.nodes_expanded = expanded;
  result.runtime_ms = runtime_ms;
  result.reason = reason;
  return result;
}

}  // namespace detail

/// Best-first search from `start` to `goal` under an arbitrary heuristic.
///
/// `cost` is borrowed for the duration of the call. A cell is drivable when its
/// cost is finite.
template <typename Heuristic>
SearchResult BestFirstSearch(const CostMapView& cost, Cell start, Cell goal,
                             const Heuristic& heuristic, const SearchLimits& limits) {
  const auto began = std::chrono::steady_clock::now();
  const auto elapsed_ms = [&began]() {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - began)
        .count();
  };

  const int rows = cost.rows();
  const int cols = cost.cols();
  const auto drivable = [&cost](int x, int y) { return std::isfinite(cost.at(x, y)); };

  if (!cost.InBounds(start.x, start.y)) {
    return detail::Failure("start out of bounds", 0, elapsed_ms());
  }
  if (!cost.InBounds(goal.x, goal.y)) {
    return detail::Failure("goal out of bounds", 0, elapsed_ms());
  }
  if (!drivable(start.x, start.y)) {
    return detail::Failure("start not drivable", 0, elapsed_ms());
  }
  if (!drivable(goal.x, goal.y)) return detail::Failure("goal not drivable", 0, elapsed_ms());

  const std::size_t cells = cost.size();
  std::vector<double> g(cells, std::numeric_limits<double>::infinity());
  std::vector<std::int64_t> parent(cells, -1);
  std::vector<char> closed(cells, 0);
  std::priority_queue<detail::OpenEntry, std::vector<detail::OpenEntry>, detail::OpenEntryGreater>
      open;

  const auto index_of = [cols](int x, int y) { return static_cast<std::int64_t>(y) * cols + x; };

  const std::int64_t start_index = index_of(start.x, start.y);
  g[static_cast<std::size_t>(start_index)] = 0.0;
  std::int64_t order = 0;
  open.push(detail::OpenEntry{heuristic(start), 0.0, order, start_index});
  std::int64_t expanded = 0;

  while (!open.empty()) {
    const detail::OpenEntry entry = open.top();
    open.pop();
    const auto index = static_cast<std::size_t>(entry.index);
    if (closed[index]) continue;
    closed[index] = 1;
    ++expanded;

    const int cx = static_cast<int>(entry.index % cols);
    const int cy = static_cast<int>(entry.index / cols);

    if (cx == goal.x && cy == goal.y) {
      SearchResult result;
      result.success = true;
      std::int64_t node = entry.index;
      while (node != -1) {
        result.path.push_back(Cell{static_cast<int>(node % cols), static_cast<int>(node / cols)});
        node = parent[static_cast<std::size_t>(node)];
      }
      std::reverse(result.path.begin(), result.path.end());
      result.cost = g[index];
      result.nodes_expanded = expanded;
      result.runtime_ms = elapsed_ms();
      result.reason = "goal reached";
      return result;
    }

    if (limits.max_nodes > 0 && expanded >= limits.max_nodes) {
      return detail::Failure("node limit reached", expanded, elapsed_ms());
    }
    if (limits.time_limit_ms >= 0.0 && elapsed_ms() > limits.time_limit_ms) {
      return detail::Failure("time limit reached", expanded, elapsed_ms());
    }

    const double current_cost = cost[index];
    const double current_g = g[index];
    for (const Move& move : kMoves) {
      const int nx = cx + move.dx;
      const int ny = cy + move.dy;
      if (nx < 0 || nx >= cols || ny < 0 || ny >= rows || !drivable(nx, ny)) continue;
      if (move.dx != 0 && move.dy != 0) {
        if (!drivable(nx, cy) || !drivable(cx, ny)) continue;  // no corner cutting
      }
      const auto neighbour = static_cast<std::size_t>(index_of(nx, ny));
      if (closed[neighbour]) continue;
      const double tentative = current_g + move.length * 0.5 * (current_cost + cost[neighbour]);
      if (tentative < g[neighbour] - kRelaxEpsilon) {
        g[neighbour] = tentative;
        parent[neighbour] = entry.index;
        ++order;
        open.push(detail::OpenEntry{tentative + heuristic(Cell{nx, ny}), tentative, order,
                                    static_cast<std::int64_t>(neighbour)});
      }
    }
  }

  return detail::Failure("goal unreachable", expanded, elapsed_ms());
}

/// Dijkstra / A* / weighted A*, chosen by `options.weight`, with the heuristic
/// scale derived from the cheapest drivable cell of `cost`.
SearchResult GridSearch(const CostMapView& cost, Cell start, Cell goal,
                        const GridSearchOptions& options);

/// Cost-to-come from `sources` to every drivable cell; infinite elsewhere.
///
/// Mirrors ``dijkstra_field`` in search.py, including its (distance, x, y) heap
/// ordering and its relaxation epsilon.
CostMap DijkstraField(const CostMapView& cost, const std::vector<Cell>& sources);

}  // namespace depot

#endif  // DEPOT_GRID_SEARCH_HPP_
