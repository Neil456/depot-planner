// Best-first search over an 8-connected grid: the C++ core of the step-1
// planner (depot_planner/grid_astar/search.py).
//
// It reproduces the Python search exactly: the same neighbourhood in the same
// order, the same "length x mean of the two cell costs" step cost, the same
// no-corner-cutting rule, the same octile heuristic scaled by the cheapest
// drivable cell, and the same tie-breaking (lowest f, then lowest g, then
// insertion order). Given the same inputs it returns the same path, not merely
// the same cost.

#ifndef DEPOT_GRID_SEARCH_HPP_
#define DEPOT_GRID_SEARCH_HPP_

#include <cstdint>
#include <limits>
#include <string>
#include <vector>

namespace depot {

/// A grid cell addressed as (x, y); the backing arrays are indexed [y, x].
struct Cell {
  int x = 0;
  int y = 0;

  friend bool operator==(const Cell& a, const Cell& b) { return a.x == b.x && a.y == b.y; }
  friend bool operator!=(const Cell& a, const Cell& b) { return !(a == b); }
};

/// One (dx, dy) step and its geometric length.
struct Move {
  int dx;
  int dy;
  double length;
};

/// The 8-connected neighbourhood, in the same order as MOVES in search.py.
extern const Move kMoves[8];

/// Budgets and heuristic weight for one search.
struct GridSearchOptions {
  /// Scales the octile heuristic: 0 gives Dijkstra, 1 gives A*, >1 weighted A*.
  double weight = 1.0;
  /// Hard cap on expansions; <= 0 means unlimited.
  std::int64_t max_nodes = 0;
  /// Wall-clock budget in milliseconds; negative means unlimited, and 0 expires
  /// at once (which is what the Python side does with time_limit_ms=0.0).
  double time_limit_ms = -1.0;
};

/// Outcome of one search.
struct GridSearchResult {
  bool success = false;
  std::vector<Cell> path;
  double cost = std::numeric_limits<double>::infinity();
  std::int64_t nodes_expanded = 0;
  double runtime_ms = 0.0;
  std::string reason;
};

/// Search `cost` (a row-major rows x cols map, non-finite on obstacles).
///
/// `cost` is borrowed for the duration of the call and never freed here.
GridSearchResult GridSearch(const double* cost, int rows, int cols, Cell start, Cell goal,
                            const GridSearchOptions& options);

}  // namespace depot

#endif  // DEPOT_GRID_SEARCH_HPP_
