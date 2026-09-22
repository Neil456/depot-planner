// Fixed scenarios shared by the benchmarks.
//
// They are built in C++ rather than loaded from the Python generators so a
// benchmark run needs no interpreter, and they are deterministic so repeated
// runs time the same work.

#ifndef DEPOT_BENCH_SCENARIOS_HPP_
#define DEPOT_BENCH_SCENARIOS_HPP_

#include <cstddef>
#include <limits>
#include <vector>

#include "depot/grid_search.hpp"

namespace depot {
namespace bench {

struct GridScenario {
  int rows = 0;
  int cols = 0;
  std::vector<double> cost;
  Cell start;
  Cell goal;
};

/// A 60 x 40 depot: an enclosing wall, three horizontal aisles separated by
/// parking bands, and two vertical connectors. Cheap in the aisles, expensive
/// in the parking bands, mirroring configs/grid.yaml.
inline GridScenario MakeDepotGrid() {
  constexpr double kInf = std::numeric_limits<double>::infinity();
  GridScenario scenario;
  scenario.rows = 40;
  scenario.cols = 60;
  scenario.cost.assign(static_cast<std::size_t>(scenario.rows) * scenario.cols, 3.0);

  const auto at = [&scenario](int x, int y) -> double& {
    return scenario.cost[static_cast<std::size_t>(y) * scenario.cols + x];
  };

  for (int x = 0; x < scenario.cols; ++x) {
    at(x, 0) = kInf;
    at(x, scenario.rows - 1) = kInf;
  }
  for (int y = 0; y < scenario.rows; ++y) {
    at(0, y) = kInf;
    at(scenario.cols - 1, y) = kInf;
  }

  const int aisle_rows[3] = {6, 18, 30};
  for (int aisle : aisle_rows) {
    for (int y = aisle; y < aisle + 4; ++y) {
      for (int x = 1; x < scenario.cols - 1; ++x) at(x, y) = 1.0;
    }
  }
  const int connector_cols[2] = {14, 42};
  for (int connector : connector_cols) {
    for (int x = connector; x < connector + 3; ++x) {
      for (int y = 1; y < scenario.rows - 1; ++y) at(x, y) = 1.0;
    }
  }

  // Curbs inside the parking bands, with a gap every seventh cell.
  const int curb_rows[3] = {12, 24, 36};
  for (int curb : curb_rows) {
    for (int x = 1; x < scenario.cols - 1; ++x) {
      bool in_connector = false;
      for (int connector : connector_cols) {
        if (x >= connector && x < connector + 3) in_connector = true;
      }
      if (!in_connector && x % 7 != 0) at(x, curb) = kInf;
    }
  }

  scenario.start = Cell{2, 7};
  scenario.goal = Cell{57, 32};
  return scenario;
}

}  // namespace bench
}  // namespace depot

#endif  // DEPOT_BENCH_SCENARIOS_HPP_
