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
#include "depot/spacetime.hpp"
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

using IntArray = py::array_t<std::int64_t, py::array::c_style | py::array::forcecast>;

/// Build the C++ agent list from a sequence of (timeline, width, height).
///
/// `timeline` is the same (T, 2) array of anchor cells the Python Agent holds.
std::vector<depot::AgentTimeline> ToAgents(const py::sequence& agents) {
  std::vector<depot::AgentTimeline> out;
  out.reserve(static_cast<std::size_t>(py::len(agents)));
  for (const py::handle& item : agents) {
    auto entry = item.cast<py::tuple>();
    if (entry.size() != 3) {
      throw std::invalid_argument("each agent must be a (timeline, width, height) tuple");
    }
    auto timeline = entry[0].cast<IntArray>();
    if (timeline.ndim() != 2 || timeline.shape(1) != 2) {
      throw std::invalid_argument("an agent timeline must have shape (steps, 2)");
    }
    depot::AgentTimeline agent;
    agent.width = entry[1].cast<int>();
    agent.height = entry[2].cast<int>();
    const auto steps = static_cast<std::size_t>(timeline.shape(0));
    agent.anchors.reserve(steps);
    const std::int64_t* rows = timeline.data();
    for (std::size_t i = 0; i < steps; ++i) {
      agent.anchors.push_back(
          depot::Cell{static_cast<int>(rows[2 * i]), static_cast<int>(rows[2 * i + 1])});
    }
    if (agent.anchors.empty()) {
      throw std::invalid_argument("an agent needs at least one timeline entry");
    }
    out.push_back(std::move(agent));
  }
  return out;
}

// Returns (success, cells, times, cost, movement_cost, wait_steps,
//          nodes_expanded, runtime_ms, reason).
py::tuple SpaceTimePlan(DoubleArray cost_grid, std::pair<int, int> start, std::pair<int, int> goal,
                        int start_time, py::sequence agents, py::object heuristic_field,
                        double time_cost, int inflate, bool relax_margin_when_inside,
                        bool heuristic_includes_time, double min_cell_cost, int horizon_cap,
                        std::int64_t max_nodes, double time_limit_ms) {
  const depot::CostMapView cost = ViewOf(cost_grid, "cost_grid");
  depot::AgentOccupancy occupancy(ToAgents(agents), inflate);

  DoubleArray field_array;
  depot::CostMapView field;
  if (!heuristic_field.is_none()) {
    field_array = heuristic_field.cast<DoubleArray>();
    field = ViewOf(field_array, "heuristic_field");
  }

  depot::SpaceTimeOptions options;
  options.time_cost = time_cost;
  options.relax_margin_when_inside = relax_margin_when_inside;
  options.heuristic_includes_time = heuristic_includes_time;
  options.min_cell_cost = min_cell_cost;
  options.horizon_cap = horizon_cap;
  options.limits.max_nodes = max_nodes;
  options.limits.time_limit_ms = time_limit_ms;

  depot::SpaceTimeResult result;
  {
    py::gil_scoped_release release;
    result = depot::SpaceTimePlan(cost, depot::Cell{start.first, start.second},
                                  depot::Cell{goal.first, goal.second}, start_time, occupancy,
                                  field, options);
  }
  py::list times;
  for (int t : result.times) times.append(t);
  return py::make_tuple(result.success, PathToList(result.cells), times, result.cost,
                        result.movement_cost, result.wait_steps, result.nodes_expanded,
                        result.runtime_ms, result.reason);
}

// Returns (success, path, cost, nodes_expanded, runtime_ms, reason).
py::tuple BaselinePlan(DoubleArray cost_grid, std::pair<int, int> start, std::pair<int, int> goal,
                       int step, py::sequence agents, int inflate, double min_cell_cost,
                       std::int64_t max_nodes, double time_limit_ms) {
  const depot::CostMapView cost = ViewOf(cost_grid, "cost_grid");
  depot::AgentOccupancy occupancy(ToAgents(agents), inflate);
  depot::SearchLimits limits;
  limits.max_nodes = max_nodes;
  limits.time_limit_ms = time_limit_ms;

  depot::SearchResult result;
  {
    py::gil_scoped_release release;
    result = depot::BaselineSnapshotPlan(cost, depot::Cell{start.first, start.second},
                                         depot::Cell{goal.first, goal.second}, step, occupancy,
                                         min_cell_cost, limits);
  }
  return py::make_tuple(result.success, PathToList(result.path), result.cost, result.nodes_expanded,
                        result.runtime_ms, result.reason);
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

  m.def("spacetime_plan", &SpaceTimePlan, py::arg("cost_grid"), py::arg("start"), py::arg("goal"),
        py::arg("start_time"), py::arg("agents"), py::arg("heuristic_field"), py::arg("time_cost"),
        py::arg("inflate"), py::arg("relax_margin_when_inside"), py::arg("heuristic_includes_time"),
        py::arg("min_cell_cost"), py::arg("horizon_cap"), py::arg("max_nodes"),
        py::arg("time_limit_ms"),
        "Space-time A* over (x, y, t) with a wait action.\n\n"
        "`agents` is a sequence of (timeline, width, height); `heuristic_field` is the "
        "cost-to-go field from the goal, or None for the octile heuristic. Returns "
        "(success, cells, times, cost, movement_cost, wait_steps, nodes_expanded, "
        "runtime_ms, reason).");

  m.def("baseline_plan", &BaselinePlan, py::arg("cost_grid"), py::arg("start"), py::arg("goal"),
        py::arg("step"), py::arg("agents"), py::arg("inflate"), py::arg("min_cell_cost"),
        py::arg("max_nodes"), py::arg("time_limit_ms"),
        "Grid A* with the agents frozen where they stand at `step`.\n\n"
        "Returns (success, path, cost, nodes_expanded, runtime_ms, reason).");
}
