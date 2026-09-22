#include "depot/grid_search.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <queue>
#include <stdexcept>

namespace depot {
namespace {

constexpr double kSqrt2 = 1.4142135623730951;
constexpr double kEpsilon = 1e-12;

struct Entry {
  double f;
  double g;
  std::int64_t order;
  std::int64_t index;
};

// Mirrors the Python heap, which pushes the tuple (f, g, counter, cell): ties on
// f go to the smaller g, then to whichever was pushed first.
struct Greater {
  bool operator()(const Entry& a, const Entry& b) const {
    if (a.f != b.f) return a.f > b.f;
    if (a.g != b.g) return a.g > b.g;
    return a.order > b.order;
  }
};

double Octile(int ax, int ay, int bx, int by) {
  const double dx = std::abs(ax - bx);
  const double dy = std::abs(ay - by);
  return (dx + dy) + (kSqrt2 - 2.0) * std::min(dx, dy);
}

GridSearchResult Failure(const std::string& reason, std::int64_t expanded, double runtime_ms) {
  GridSearchResult result;
  result.success = false;
  result.nodes_expanded = expanded;
  result.runtime_ms = runtime_ms;
  result.reason = reason;
  return result;
}

}  // namespace

const Move kMoves[8] = {
    {1, 0, 1.0},    {-1, 0, 1.0},    {0, 1, 1.0},     {0, -1, 1.0},
    {1, 1, kSqrt2}, {1, -1, kSqrt2}, {-1, 1, kSqrt2}, {-1, -1, kSqrt2},
};

GridSearchResult GridSearch(const double* cost, int rows, int cols, Cell start, Cell goal,
                            const GridSearchOptions& options) {
  const auto began = std::chrono::steady_clock::now();
  const auto elapsed_ms = [&began]() {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - began)
        .count();
  };

  if (cost == nullptr || rows <= 0 || cols <= 0) {
    throw std::invalid_argument("cost map must be a non-empty 2D array");
  }

  const auto in_bounds = [rows, cols](int x, int y) {
    return x >= 0 && x < cols && y >= 0 && y < rows;
  };
  const auto drivable = [&](int x, int y) {
    return std::isfinite(cost[static_cast<std::size_t>(y) * cols + x]);
  };

  if (!in_bounds(start.x, start.y)) return Failure("start out of bounds", 0, elapsed_ms());
  if (!in_bounds(goal.x, goal.y)) return Failure("goal out of bounds", 0, elapsed_ms());
  if (!drivable(start.x, start.y)) return Failure("start not drivable", 0, elapsed_ms());
  if (!drivable(goal.x, goal.y)) return Failure("goal not drivable", 0, elapsed_ms());

  // Cheapest drivable cell: the scale that keeps the octile heuristic admissible.
  double min_cell_cost = std::numeric_limits<double>::infinity();
  const std::size_t cells = static_cast<std::size_t>(rows) * cols;
  for (std::size_t i = 0; i < cells; ++i) {
    if (std::isfinite(cost[i]) && cost[i] < min_cell_cost) min_cell_cost = cost[i];
  }
  if (!std::isfinite(min_cell_cost)) min_cell_cost = 0.0;
  const double factor = min_cell_cost * options.weight;

  std::vector<double> g(cells, std::numeric_limits<double>::infinity());
  std::vector<std::int64_t> parent(cells, -1);
  std::vector<char> closed(cells, 0);
  std::priority_queue<Entry, std::vector<Entry>, Greater> open;

  const auto index_of = [cols](int x, int y) { return static_cast<std::int64_t>(y) * cols + x; };
  const auto heuristic = [&](int x, int y) { return Octile(x, y, goal.x, goal.y) * factor; };

  const std::int64_t start_index = index_of(start.x, start.y);
  g[static_cast<std::size_t>(start_index)] = 0.0;
  std::int64_t order = 0;
  open.push(Entry{heuristic(start.x, start.y), 0.0, order, start_index});
  std::int64_t expanded = 0;

  while (!open.empty()) {
    const Entry entry = open.top();
    open.pop();
    const auto index = static_cast<std::size_t>(entry.index);
    if (closed[index]) continue;
    closed[index] = 1;
    ++expanded;

    const int cx = static_cast<int>(entry.index % cols);
    const int cy = static_cast<int>(entry.index / cols);

    if (cx == goal.x && cy == goal.y) {
      GridSearchResult result;
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

    if (options.max_nodes > 0 && expanded >= options.max_nodes) {
      return Failure("node limit reached", expanded, elapsed_ms());
    }
    if (options.time_limit_ms >= 0.0 && elapsed_ms() > options.time_limit_ms) {
      return Failure("time limit reached", expanded, elapsed_ms());
    }

    const double current_cost = cost[index];
    const double current_g = g[index];
    for (const Move& move : kMoves) {
      const int nx = cx + move.dx;
      const int ny = cy + move.dy;
      if (!in_bounds(nx, ny) || !drivable(nx, ny)) continue;
      if (move.dx != 0 && move.dy != 0) {
        if (!drivable(nx, cy) || !drivable(cx, ny)) continue;  // no corner cutting
      }
      const auto neighbour = static_cast<std::size_t>(index_of(nx, ny));
      if (closed[neighbour]) continue;
      const double tentative = current_g + move.length * 0.5 * (current_cost + cost[neighbour]);
      if (tentative < g[neighbour] - kEpsilon) {
        g[neighbour] = tentative;
        parent[neighbour] = entry.index;
        ++order;
        open.push(Entry{tentative + heuristic(nx, ny), tentative, order,
                        static_cast<std::int64_t>(neighbour)});
      }
    }
  }

  return Failure("goal unreachable", expanded, elapsed_ms());
}

}  // namespace depot
