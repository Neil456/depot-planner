// pybind11 bindings for the depot planning core.
//
// The module is imported as ``depot_planner._cpp``; the Python side wraps each
// entry point in the dataclasses the rest of the project already uses.

#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <stdexcept>
#include <vector>

#include "depot/distance_field.hpp"
#include "depot/grid_search.hpp"
#include "depot/types.hpp"

namespace py = pybind11;

namespace {

using DoubleArray = py::array_t<double, py::array::c_style | py::array::forcecast>;
using BoolArray = py::array_t<bool, py::array::c_style | py::array::forcecast>;

/// A borrowed view over a 2D numpy array of doubles.
depot::CostMapView ViewOf(const DoubleArray& array, const char* what) {
  if (array.ndim() != 2) throw std::invalid_argument(std::string(what) + " must be a 2D array");
  return depot::CostMapView(array.data(), static_cast<int>(array.shape(0)),
                            static_cast<int>(array.shape(1)));
}

depot::MaskView MaskOf(const BoolArray& array, const char* what) {
  if (array.ndim() != 2) throw std::invalid_argument(std::string(what) + " must be a 2D array");
  return depot::MaskView(array.data(), static_cast<int>(array.shape(0)),
                         static_cast<int>(array.shape(1)));
}

py::list PathToList(const depot::Path& path) {
  py::list cells;
  for (const depot::Cell& cell : path) cells.append(py::make_tuple(cell.x, cell.y));
  return cells;
}

/// Copy an owned grid into a fresh numpy array.
py::array_t<double> ToArray(const depot::Grid2D<double>& grid) {
  py::array_t<double> out({grid.rows(), grid.cols()});
  std::copy(grid.data(), grid.data() + grid.size(), out.mutable_data());
  return out;
}

// Returns (success, path, cost, nodes_expanded, runtime_ms, reason).
py::tuple GridAstar(DoubleArray cost_grid, std::pair<int, int> start, std::pair<int, int> goal,
                    double weight, std::int64_t max_nodes, double time_limit_ms) {
  const depot::CostMapView cost = ViewOf(cost_grid, "cost_grid");
  depot::GridSearchOptions options;
  options.weight = weight;
  options.limits.max_nodes = max_nodes;
  options.limits.time_limit_ms = time_limit_ms;

  depot::SearchResult result;
  {
    py::gil_scoped_release release;
    result = depot::GridSearch(cost, depot::Cell{start.first, start.second},
                               depot::Cell{goal.first, goal.second}, options);
  }
  return py::make_tuple(result.success, PathToList(result.path), result.cost, result.nodes_expanded,
                        result.runtime_ms, result.reason);
}

py::array_t<double> DijkstraField(DoubleArray cost_grid,
                                  const std::vector<std::pair<int, int>>& sources) {
  const depot::CostMapView cost = ViewOf(cost_grid, "cost_grid");
  std::vector<depot::Cell> cells;
  cells.reserve(sources.size());
  for (const auto& source : sources) cells.push_back(depot::Cell{source.first, source.second});

  depot::Grid2D<double> field;
  {
    py::gil_scoped_release release;
    field = depot::DijkstraField(cost, cells);
  }
  return ToArray(field);
}

py::array_t<double> DistanceTransform(BoolArray mask) {
  const depot::MaskView view = MaskOf(mask, "mask");
  depot::Grid2D<double> field;
  {
    py::gil_scoped_release release;
    field = depot::EuclideanDistanceTransform(view);
  }
  return ToArray(field);
}

py::array_t<double> ObstacleDistance(BoolArray drivable, double resolution) {
  const depot::MaskView view = MaskOf(drivable, "drivable");
  depot::Grid2D<double> field;
  {
    py::gil_scoped_release release;
    field = depot::ObstacleDistance(view, resolution);
  }
  return ToArray(field);
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

  m.def("dijkstra_field", &DijkstraField, py::arg("cost_grid"), py::arg("sources"),
        "Cost-to-come from `sources` to every drivable cell; inf elsewhere.");

  m.def("distance_transform", &DistanceTransform, py::arg("mask"),
        "Exact Euclidean distance, in samples, from every true cell to the nearest false one.");

  m.def("obstacle_distance", &ObstacleDistance, py::arg("drivable"), py::arg("resolution"),
        "Distance in metres from every drivable cell to the nearest non-drivable one.");
}
