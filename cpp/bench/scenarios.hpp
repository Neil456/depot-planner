// Fixed scenarios shared by the benchmarks.
//
// They are built in C++ rather than loaded from the Python generators so a
// benchmark run needs no interpreter, and they are deterministic so repeated
// runs time the same work. Everything here is setup: the benchmarks build a
// scenario once, outside the timed region, and then measure only the search.

#ifndef DEPOT_BENCH_SCENARIOS_HPP_
#define DEPOT_BENCH_SCENARIOS_HPP_

#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include "depot/distance_field.hpp"
#include "depot/grid_search.hpp"
#include "depot/hybrid.hpp"
#include "depot/spacetime.hpp"

namespace depot {
namespace bench {

struct GridScenario {
  int rows = 0;
  int cols = 0;
  std::vector<double> cost;
  Cell start;
  Cell goal;

  CostMapView view() const { return CostMapView(cost.data(), rows, cols); }
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

/// The depot grid plus six 2x2 vehicles, two per aisle, driving towards each
/// other along the lane and then pulling into the parking band beside it.
///
/// Each vehicle plus its one-cell margin fills the four-cell aisle completely
/// while it is in the lane, so the ego has to time its run; once they park, the
/// lane clears and the goal stays reachable. That is the same shape of problem
/// the `congested` scenario generator produces.
inline AgentOccupancy MakeTraffic() {
  constexpr int kHorizon = 400;
  const int aisle_rows[3] = {6, 18, 30};
  std::vector<AgentTimeline> agents;

  const auto drive = [&agents](int lane_y, int park_y, int from_x, int to_x, int start_delay) {
    AgentTimeline agent;
    agent.width = 2;
    agent.height = 2;
    const int direction = to_x >= from_x ? 1 : -1;
    int x = from_x;
    int y = lane_y;
    for (int t = 0; t <= kHorizon; ++t) {
      agent.anchors.push_back(Cell{x, y});
      if (t < start_delay) continue;
      if (x != to_x) {
        x += direction;  // one cell per step along the lane
      } else if (y != park_y) {
        ++y;  // then pull into the parking band and stop
      }
    }
    agents.push_back(std::move(agent));
  };

  for (int lane = 0; lane < 3; ++lane) {
    const int lane_y = aisle_rows[lane] + 1;
    const int park_y = aisle_rows[lane] + 4;
    drive(lane_y, park_y, 6 + 4 * lane, 30 + 4 * lane, 2 * lane);
    drive(lane_y, park_y, 52 - 3 * lane, 36 + 4 * lane, 5 + 2 * lane);
  }
  return AgentOccupancy(std::move(agents), 1);
}

struct Rectangle {
  double x0, y0, x1, y1;
};

struct ParkingScenario {
  double resolution = 0.05;
  Grid2D<double> distance;
  Pose start;
  Pose goal;

  DistanceFieldView view() const { return DistanceFieldView(CostMapView(distance), resolution); }
};

/// Rasterise rectangles and build the same conservative distance field the
/// Python scenario builder produces: max(edt - sqrt(2), 0) * resolution.
inline Grid2D<double> BuildDistanceField(double width_m, double height_m,
                                         const std::vector<Rectangle>& obstacles,
                                         double resolution) {
  const int cols = static_cast<int>(std::lround(width_m / resolution));
  const int rows = static_cast<int>(std::lround(height_m / resolution));
  std::vector<char> free(static_cast<std::size_t>(rows) * cols, 1);
  for (const Rectangle& rectangle : obstacles) {
    const int i0 = std::max(0, static_cast<int>(std::floor(rectangle.y0 / resolution)));
    const int i1 = std::min(rows, static_cast<int>(std::ceil(rectangle.y1 / resolution)));
    const int j0 = std::max(0, static_cast<int>(std::floor(rectangle.x0 / resolution)));
    const int j1 = std::min(cols, static_cast<int>(std::ceil(rectangle.x1 / resolution)));
    for (int i = i0; i < i1; ++i) {
      for (int j = j0; j < j1; ++j) free[static_cast<std::size_t>(i) * cols + j] = 0;
    }
  }
  MaskView mask(reinterpret_cast<const bool*>(free.data()), rows, cols);
  Grid2D<double> field = EuclideanDistanceTransform(mask);
  for (std::size_t i = 0; i < field.size(); ++i) {
    field.data()[i] = std::max(field.data()[i] - std::sqrt(2.0), 0.0) * resolution;
  }
  return field;
}

/// A parallel-parking bay: two rows of parked cars either side of an aisle,
/// with one 7 m gap in the near row that the ego has to reverse into.
inline ParkingScenario MakeParallelParking() {
  const double car_length = 4.5, car_width = 1.9, wall = 0.5;
  const double lane_depth = 2.8, aisle = 7.0, spacing = 1.0, gap = 7.0;
  const double height = 2.0 * wall + 2.0 * lane_depth + aisle;
  const double lane_centre = wall + lane_depth / 2.0;
  const double far_centre = height - wall - lane_depth / 2.0;

  std::vector<Rectangle> near;
  double cursor = wall + spacing;
  double gap_start = 0.0;
  for (int index = 0; index < 5; ++index) {
    if (index == 2) {
      gap_start = cursor;
      cursor += gap;
    }
    near.push_back({cursor, lane_centre - car_width / 2.0, cursor + car_length,
                    lane_centre + car_width / 2.0});
    cursor += car_length + spacing;
  }
  const double width = cursor + wall;

  std::vector<Rectangle> obstacles = {{0.0, 0.0, width, wall},
                                      {0.0, height - wall, width, height},
                                      {0.0, 0.0, wall, height},
                                      {width - wall, 0.0, width, height}};
  obstacles.insert(obstacles.end(), near.begin(), near.end());
  for (double x = wall + spacing; x + car_length + wall < width; x += car_length + spacing) {
    obstacles.push_back(
        {x, far_centre - car_width / 2.0, x + car_length, far_centre + car_width / 2.0});
  }

  ParkingScenario scenario;
  scenario.resolution = 0.05;
  scenario.distance = BuildDistanceField(width, height, obstacles, scenario.resolution);
  scenario.goal = Pose{gap_start + (gap - car_length) / 2.0 + 0.9, lane_centre, 0.0};
  scenario.start = Pose{scenario.goal.x + 9.0, wall + lane_depth + aisle / 2.0, kPi};
  return scenario;
}

}  // namespace bench
}  // namespace depot

#endif  // DEPOT_BENCH_SCENARIOS_HPP_
