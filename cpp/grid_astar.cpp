// C++17 port of the step-1 grid search (depot_planner/grid_astar/search.py).
//
// It reproduces the Python search exactly: the same 8-connected neighbourhood
// in the same order, the same "length x mean of the two cell costs" step cost,
// the same no-corner-cutting rule, the same octile heuristic scaled by the
// cheapest drivable cell, and the same tie-breaking (lowest f, then insertion
// order). Given the same inputs it returns the same path, not merely the same
// cost.

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <string>
#include <utility>
#include <vector>

namespace py = pybind11;

namespace {

constexpr double kSqrt2 = 1.4142135623730951;
constexpr double kEpsilon = 1e-12;

struct Move {
  int dx;
  int dy;
  double length;
};

// Same order as MOVES in search.py.
constexpr Move kMoves[8] = {
    {1, 0, 1.0},      {-1, 0, 1.0},      {0, 1, 1.0},       {0, -1, 1.0},
    {1, 1, kSqrt2},   {1, -1, kSqrt2},   {-1, 1, kSqrt2},   {-1, -1, kSqrt2},
};

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

double octile(int ax, int ay, int bx, int by) {
  const double dx = std::abs(ax - bx);
  const double dy = std::abs(ay - by);
  return (dx + dy) + (kSqrt2 - 2.0) * std::min(dx, dy);
}

py::tuple make_failure(const std::string& reason, std::int64_t expanded, double runtime_ms) {
  return py::make_tuple(false, py::list(), std::numeric_limits<double>::infinity(), expanded,
                        runtime_ms, reason);
}

}  // namespace

// Returns (success, path, cost, nodes_expanded, runtime_ms, reason).
// `weight` scales the octile heuristic: 0 gives Dijkstra, 1 gives A*.
py::tuple grid_astar(py::array_t<double, py::array::c_style | py::array::forcecast> cost_grid,
                     std::pair<int, int> start, std::pair<int, int> goal, double weight,
                     std::int64_t max_nodes, double time_limit_ms) {
  // max_nodes <= 0 and time_limit_ms < 0 both mean "no limit".
  const auto began = std::chrono::steady_clock::now();
  const auto elapsed_ms = [&began]() {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - began)
        .count();
  };

  if (cost_grid.ndim() != 2) {
    throw std::invalid_argument("cost_grid must be a 2D array");
  }
  const auto rows = static_cast<int>(cost_grid.shape(0));
  const auto cols = static_cast<int>(cost_grid.shape(1));
  const double* cost = cost_grid.data();

  const auto in_bounds = [rows, cols](int x, int y) {
    return x >= 0 && x < cols && y >= 0 && y < rows;
  };
  const auto drivable = [&](int x, int y) { return std::isfinite(cost[static_cast<std::size_t>(y) * cols + x]); };

  const int sx = start.first, sy = start.second;
  const int gx = goal.first, gy = goal.second;
  if (!in_bounds(sx, sy)) return make_failure("start out of bounds", 0, elapsed_ms());
  if (!in_bounds(gx, gy)) return make_failure("goal out of bounds", 0, elapsed_ms());
  if (!drivable(sx, sy)) return make_failure("start not drivable", 0, elapsed_ms());
  if (!drivable(gx, gy)) return make_failure("goal not drivable", 0, elapsed_ms());

  // Cheapest drivable cell: the scale that keeps the octile heuristic admissible.
  double min_cell_cost = std::numeric_limits<double>::infinity();
  const std::size_t cells = static_cast<std::size_t>(rows) * cols;
  for (std::size_t i = 0; i < cells; ++i) {
    if (std::isfinite(cost[i]) && cost[i] < min_cell_cost) min_cell_cost = cost[i];
  }
  if (!std::isfinite(min_cell_cost)) min_cell_cost = 0.0;
  const double factor = min_cell_cost * weight;

  std::vector<double> g(cells, std::numeric_limits<double>::infinity());
  std::vector<std::int64_t> parent(cells, -1);
  std::vector<char> closed(cells, 0);
  std::priority_queue<Entry, std::vector<Entry>, Greater> open;

  const auto index_of = [cols](int x, int y) {
    return static_cast<std::int64_t>(y) * cols + x;
  };
  const auto heuristic = [&](int x, int y) { return octile(x, y, gx, gy) * factor; };

  const std::int64_t start_index = index_of(sx, sy);
  g[static_cast<std::size_t>(start_index)] = 0.0;
  std::int64_t order = 0;
  open.push(Entry{heuristic(sx, sy), 0.0, order, start_index});
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

    if (cx == gx && cy == gy) {
      std::vector<std::pair<int, int>> reversed;
      std::int64_t node = entry.index;
      while (node != -1) {
        reversed.emplace_back(static_cast<int>(node % cols), static_cast<int>(node / cols));
        node = parent[static_cast<std::size_t>(node)];
      }
      py::list path;
      for (auto it = reversed.rbegin(); it != reversed.rend(); ++it) {
        path.append(py::make_tuple(it->first, it->second));
      }
      return py::make_tuple(true, path, g[index], expanded, elapsed_ms(), "goal reached");
    }

    if (max_nodes > 0 && expanded >= max_nodes) {
      return make_failure("node limit reached", expanded, elapsed_ms());
    }
    // A negative budget means "unlimited"; zero means "expire at once", which is
    // what the Python side does with time_limit_ms=0.0.
    if (time_limit_ms >= 0.0 && elapsed_ms() > time_limit_ms) {
      return make_failure("time limit reached", expanded, elapsed_ms());
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

  return make_failure("goal unreachable", expanded, elapsed_ms());
}

PYBIND11_MODULE(_cpp, m) {
  m.doc() = "C++17 core for the step-1 depot grid search";
  m.def("grid_astar", &grid_astar,
        py::arg("cost_grid"), py::arg("start"), py::arg("goal"), py::arg("weight") = 1.0,
        py::arg("max_nodes") = 0, py::arg("time_limit_ms") = -1.0,
        "Best-first search over an 8-connected grid.\n\n"
        "Returns (success, path, cost, nodes_expanded, runtime_ms, reason). "
        "`weight` scales the octile heuristic: 0 is Dijkstra, 1 is A*. "
        "max_nodes <= 0 and time_limit_ms < 0 mean no limit.");
}
