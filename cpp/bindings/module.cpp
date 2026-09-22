// pybind11 bindings for the depot planning core.
//
// The module is imported as ``depot_planner._cpp``; the Python side wraps each
// entry point in the dataclasses the rest of the project already uses.

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <stdexcept>

#include "depot/grid_search.hpp"

namespace py = pybind11;

namespace {

using CostArray = py::array_t<double, py::array::c_style | py::array::forcecast>;

py::list PathToList(const std::vector<depot::Cell>& path) {
  py::list cells;
  for (const depot::Cell& cell : path) cells.append(py::make_tuple(cell.x, cell.y));
  return cells;
}

// Returns (success, path, cost, nodes_expanded, runtime_ms, reason).
py::tuple GridAstar(CostArray cost_grid, std::pair<int, int> start, std::pair<int, int> goal,
                    double weight, std::int64_t max_nodes, double time_limit_ms) {
  if (cost_grid.ndim() != 2) throw std::invalid_argument("cost_grid must be a 2D array");
  depot::GridSearchOptions options;
  options.weight = weight;
  options.max_nodes = max_nodes;
  options.time_limit_ms = time_limit_ms;

  depot::GridSearchResult result;
  {
    py::gil_scoped_release release;
    result = depot::GridSearch(cost_grid.data(), static_cast<int>(cost_grid.shape(0)),
                               static_cast<int>(cost_grid.shape(1)),
                               depot::Cell{start.first, start.second},
                               depot::Cell{goal.first, goal.second}, options);
  }
  return py::make_tuple(result.success, PathToList(result.path), result.cost, result.nodes_expanded,
                        result.runtime_ms, result.reason);
}

}  // namespace

PYBIND11_MODULE(_cpp, m) {
  m.doc() = "C++17 planning core for the depot planner";

  m.def("grid_astar", &GridAstar, py::arg("cost_grid"), py::arg("start"), py::arg("goal"),
        py::arg("weight") = 1.0, py::arg("max_nodes") = 0, py::arg("time_limit_ms") = -1.0,
        "Best-first search over an 8-connected grid.\n\n"
        "Returns (success, path, cost, nodes_expanded, runtime_ms, reason). "
        "`weight` scales the octile heuristic: 0 is Dijkstra, 1 is A*. "
        "max_nodes <= 0 and time_limit_ms < 0 mean no limit.");
}
