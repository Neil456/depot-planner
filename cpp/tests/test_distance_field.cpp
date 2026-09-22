#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <random>
#include <vector>

#include "depot/distance_field.hpp"
#include "depot/grid_search.hpp"

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

/// The obvious O(n^2) transform, used to pin the fast one.
depot::Grid2D<double> BruteForce(const std::vector<bool>& mask, int rows, int cols) {
  depot::Grid2D<double> out(rows, cols, kInf);
  for (int y = 0; y < rows; ++y) {
    for (int x = 0; x < cols; ++x) {
      if (!mask[static_cast<std::size_t>(y) * cols + x]) {
        out.at(x, y) = 0.0;
        continue;
      }
      double best = std::numeric_limits<double>::infinity();
      for (int j = 0; j < rows; ++j) {
        for (int i = 0; i < cols; ++i) {
          if (mask[static_cast<std::size_t>(j) * cols + i]) continue;
          const double squared =
              static_cast<double>(x - i) * (x - i) + static_cast<double>(y - j) * (y - j);
          if (squared < best) best = squared;
        }
      }
      out.at(x, y) = std::sqrt(best);
    }
  }
  return out;
}

depot::MaskView ViewOf(const std::vector<char>& storage, int rows, int cols) {
  return depot::MaskView(reinterpret_cast<const bool*>(storage.data()), rows, cols);
}

TEST(DistanceField, BackgroundCellsAreZero) {
  const int rows = 4, cols = 4;
  std::vector<char> mask(static_cast<std::size_t>(rows) * cols, 1);
  mask[0] = 0;
  const auto field = depot::EuclideanDistanceTransform(ViewOf(mask, rows, cols));
  EXPECT_DOUBLE_EQ(field.at(0, 0), 0.0);
  EXPECT_DOUBLE_EQ(field.at(1, 0), 1.0);
  EXPECT_DOUBLE_EQ(field.at(1, 1), std::sqrt(2.0));
  EXPECT_DOUBLE_EQ(field.at(3, 3), std::sqrt(18.0));
}

TEST(DistanceField, AMaskWithNoBackgroundIsInfinitelyFar) {
  const int rows = 3, cols = 5;
  const std::vector<char> mask(static_cast<std::size_t>(rows) * cols, 1);
  const auto field = depot::EuclideanDistanceTransform(ViewOf(mask, rows, cols));
  for (int y = 0; y < rows; ++y) {
    for (int x = 0; x < cols; ++x) EXPECT_TRUE(std::isinf(field.at(x, y)));
  }
}

TEST(DistanceField, MatchesTheBruteForceTransformOnRandomMasks) {
  std::mt19937 rng(20240617);
  for (int trial = 0; trial < 40; ++trial) {
    const int rows = 3 + static_cast<int>(rng() % 18);
    const int cols = 3 + static_cast<int>(rng() % 18);
    std::vector<char> mask(static_cast<std::size_t>(rows) * cols, 1);
    std::vector<bool> reference(static_cast<std::size_t>(rows) * cols, true);
    // Between one and a fifth of the cells are background.
    const std::size_t holes = 1 + rng() % (1 + mask.size() / 5);
    for (std::size_t i = 0; i < holes; ++i) {
      const std::size_t index = rng() % mask.size();
      mask[index] = 0;
      reference[index] = false;
    }
    const auto fast = depot::EuclideanDistanceTransform(ViewOf(mask, rows, cols));
    const auto slow = BruteForce(reference, rows, cols);
    for (int y = 0; y < rows; ++y) {
      for (int x = 0; x < cols; ++x) {
        ASSERT_DOUBLE_EQ(fast.at(x, y), slow.at(x, y))
            << "trial " << trial << " at (" << x << ", " << y << ")";
      }
    }
  }
}

TEST(DistanceField, ScalesToMetres) {
  const int rows = 3, cols = 3;
  std::vector<char> mask(static_cast<std::size_t>(rows) * cols, 1);
  mask[0] = 0;
  const auto field = depot::ObstacleDistance(ViewOf(mask, rows, cols), 0.5);
  EXPECT_DOUBLE_EQ(field.at(2, 0), 1.0);
}

// ------------------------------------------------------------ Dijkstra field

TEST(DijkstraField, IsZeroAtTheSourceAndGrowsOutwards) {
  const int rows = 5, cols = 5;
  const std::vector<double> cost(static_cast<std::size_t>(rows) * cols, 1.0);
  const auto field = depot::DijkstraField(depot::CostMapView(cost.data(), rows, cols), {{2, 2}});
  EXPECT_DOUBLE_EQ(field.at(2, 2), 0.0);
  EXPECT_DOUBLE_EQ(field.at(3, 2), 1.0);
  EXPECT_DOUBLE_EQ(field.at(3, 3), depot::kSqrt2);
}

TEST(DijkstraField, LeavesUnreachableCellsInfinite) {
  const int rows = 5, cols = 5;
  std::vector<double> cost(static_cast<std::size_t>(rows) * cols, 1.0);
  for (int x = 0; x < cols; ++x) cost[static_cast<std::size_t>(2) * cols + x] = kInf;
  const auto field = depot::DijkstraField(depot::CostMapView(cost.data(), rows, cols), {{0, 0}});
  EXPECT_TRUE(std::isfinite(field.at(4, 1)));
  EXPECT_TRUE(std::isinf(field.at(0, 3)));
  EXPECT_TRUE(std::isinf(field.at(2, 2)));
}

TEST(DijkstraField, AgreesWithTheSearchItMirrors) {
  const int rows = 12, cols = 14;
  std::vector<double> cost(static_cast<std::size_t>(rows) * cols, 1.0);
  for (int y = 0; y < 9; ++y) cost[static_cast<std::size_t>(y) * cols + 6] = kInf;
  for (int x = 0; x < cols; ++x) cost[static_cast<std::size_t>(5) * cols + x] = 4.0;
  const depot::CostMapView view(cost.data(), rows, cols);
  const auto field = depot::DijkstraField(view, {{13, 11}});
  depot::GridSearchOptions options;
  options.weight = 0.0;
  for (const depot::Cell start : {depot::Cell{0, 0}, depot::Cell{3, 7}, depot::Cell{11, 2}}) {
    const auto result = depot::GridSearch(view, start, {13, 11}, options);
    ASSERT_TRUE(result.success);
    EXPECT_NEAR(field.at(start.x, start.y), result.cost, 1e-9);
  }
}

}  // namespace
