#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "depot/grid_search.hpp"

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

/// A rows x cols cost map of uniform cost, obstacles punched in afterwards.
std::vector<double> UniformMap(int rows, int cols, double value) {
  return std::vector<double>(static_cast<std::size_t>(rows) * cols, value);
}

void Block(std::vector<double>& map, int cols, int x, int y) {
  map[static_cast<std::size_t>(y) * cols + x] = kInf;
}

depot::SearchResult Solve(const std::vector<double>& map, int rows, int cols, depot::Cell start,
                          depot::Cell goal, double weight = 1.0) {
  depot::GridSearchOptions options;
  options.weight = weight;
  return depot::GridSearch(depot::CostMapView(map.data(), rows, cols), start, goal, options);
}

double PathCost(const std::vector<double>& map, int cols, const std::vector<depot::Cell>& path) {
  double total = 0.0;
  for (std::size_t i = 0; i + 1 < path.size(); ++i) {
    const depot::Cell& a = path[i];
    const depot::Cell& b = path[i + 1];
    const double length = std::hypot(b.x - a.x, b.y - a.y);
    total += length * 0.5 *
             (map[static_cast<std::size_t>(a.y) * cols + a.x] +
              map[static_cast<std::size_t>(b.y) * cols + b.x]);
  }
  return total;
}

TEST(GridSearch, FindsAStraightLineOnAnEmptyMap) {
  const int rows = 10, cols = 10;
  const auto map = UniformMap(rows, cols, 1.0);
  const auto result = Solve(map, rows, cols, {0, 0}, {9, 9});
  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.reason, "goal reached");
  EXPECT_EQ(result.path.size(), 10u);
  EXPECT_EQ(result.path.front(), (depot::Cell{0, 0}));
  EXPECT_EQ(result.path.back(), (depot::Cell{9, 9}));
  EXPECT_NEAR(result.cost, 9.0 * std::sqrt(2.0), 1e-9);
}

TEST(GridSearch, ReportedCostMatchesTheWalkedPath) {
  const int rows = 12, cols = 15;
  auto map = UniformMap(rows, cols, 1.0);
  for (int y = 0; y < rows; ++y) map[static_cast<std::size_t>(y) * cols + 7] = 4.0;
  const auto result = Solve(map, rows, cols, {1, 1}, {13, 10});
  ASSERT_TRUE(result.success);
  EXPECT_NEAR(result.cost, PathCost(map, cols, result.path), 1e-9);
}

TEST(GridSearch, EveryStepIsInsideTheEightNeighbourhood) {
  const int rows = 20, cols = 20;
  auto map = UniformMap(rows, cols, 1.0);
  for (int y = 2; y < 18; ++y) Block(map, cols, 10, y);
  const auto result = Solve(map, rows, cols, {1, 10}, {18, 10});
  ASSERT_TRUE(result.success);
  for (std::size_t i = 0; i + 1 < result.path.size(); ++i) {
    EXPECT_LE(std::abs(result.path[i + 1].x - result.path[i].x), 1);
    EXPECT_LE(std::abs(result.path[i + 1].y - result.path[i].y), 1);
  }
}

TEST(GridSearch, RefusesToCutTheCornerOfAnObstacle) {
  // A diagonal step from (0,0) to (1,1) needs both (1,0) and (0,1) drivable.
  const int rows = 2, cols = 2;
  auto map = UniformMap(rows, cols, 1.0);
  Block(map, cols, 1, 0);
  Block(map, cols, 0, 1);
  const auto result = Solve(map, rows, cols, {0, 0}, {1, 1});
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, "goal unreachable");
}

TEST(GridSearch, DijkstraAndAstarAgreeOnCost) {
  const int rows = 25, cols = 30;
  auto map = UniformMap(rows, cols, 1.0);
  for (int y = 0; y < 20; ++y) Block(map, cols, 12, y);
  for (int y = 6; y < rows; ++y) Block(map, cols, 21, y);
  const auto dijkstra = Solve(map, rows, cols, {1, 1}, {28, 23}, 0.0);
  const auto astar = Solve(map, rows, cols, {1, 1}, {28, 23}, 1.0);
  ASSERT_TRUE(dijkstra.success);
  ASSERT_TRUE(astar.success);
  EXPECT_NEAR(dijkstra.cost, astar.cost, 1e-9);
  EXPECT_LE(astar.nodes_expanded, dijkstra.nodes_expanded);
}

TEST(GridSearch, WeightedAstarStaysWithinItsBound) {
  const int rows = 25, cols = 30;
  auto map = UniformMap(rows, cols, 1.0);
  for (int y = 0; y < 20; ++y) Block(map, cols, 12, y);
  const auto optimal = Solve(map, rows, cols, {1, 1}, {28, 23}, 1.0);
  const auto inflated = Solve(map, rows, cols, {1, 1}, {28, 23}, 1.5);
  ASSERT_TRUE(optimal.success);
  ASSERT_TRUE(inflated.success);
  EXPECT_LE(inflated.cost, optimal.cost * 1.5 + 1e-9);
}

TEST(GridSearch, RejectsEndpointsThatAreNotUsable) {
  const int rows = 5, cols = 5;
  auto map = UniformMap(rows, cols, 1.0);
  Block(map, cols, 4, 4);
  EXPECT_EQ(Solve(map, rows, cols, {-1, 0}, {3, 3}).reason, "start out of bounds");
  EXPECT_EQ(Solve(map, rows, cols, {0, 0}, {5, 3}).reason, "goal out of bounds");
  EXPECT_EQ(Solve(map, rows, cols, {4, 4}, {0, 0}).reason, "start not drivable");
  EXPECT_EQ(Solve(map, rows, cols, {0, 0}, {4, 4}).reason, "goal not drivable");
}

TEST(GridSearch, HonoursTheNodeCap) {
  const int rows = 40, cols = 40;
  const auto map = UniformMap(rows, cols, 1.0);
  depot::GridSearchOptions options;
  options.limits.max_nodes = 12;
  const auto result =
      depot::GridSearch(depot::CostMapView(map.data(), rows, cols), {0, 0}, {39, 39}, options);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, "node limit reached");
  EXPECT_EQ(result.nodes_expanded, 12);
}

TEST(GridSearch, AZeroBudgetExpiresImmediately) {
  const int rows = 40, cols = 40;
  const auto map = UniformMap(rows, cols, 1.0);
  depot::GridSearchOptions options;
  options.limits.time_limit_ms = 0.0;
  const auto result =
      depot::GridSearch(depot::CostMapView(map.data(), rows, cols), {0, 0}, {39, 39}, options);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, "time limit reached");
}

TEST(GridSearch, ANegativeBudgetMeansUnlimited) {
  const int rows = 40, cols = 40;
  const auto map = UniformMap(rows, cols, 1.0);
  depot::GridSearchOptions options;
  options.limits.time_limit_ms = -1.0;
  const auto result =
      depot::GridSearch(depot::CostMapView(map.data(), rows, cols), {0, 0}, {39, 39}, options);
  EXPECT_TRUE(result.success);
}

TEST(GridSearch, PrefersCheapCellsOverShortPaths) {
  // A 3-row corridor: the middle row is expensive, so the search detours.
  const int rows = 3, cols = 9;
  auto map = UniformMap(rows, cols, 1.0);
  for (int x = 0; x < cols; ++x) map[static_cast<std::size_t>(1) * cols + x] = 50.0;
  const auto result = Solve(map, rows, cols, {0, 1}, {8, 1});
  ASSERT_TRUE(result.success);
  bool leaves_the_middle_row = false;
  for (const depot::Cell& cell : result.path) {
    if (cell.y != 1) leaves_the_middle_row = true;
  }
  EXPECT_TRUE(leaves_the_middle_row);
}

}  // namespace
