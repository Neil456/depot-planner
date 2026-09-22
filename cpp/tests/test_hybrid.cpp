#include <gtest/gtest.h>

#include <cmath>
#include <vector>

#include "depot/distance_field.hpp"
#include "depot/hybrid.hpp"
#include "depot/numeric.hpp"

namespace {

/// A rasterised parking lot and its conservative distance field, built the same
/// way DistanceField.from_rectangles does it in Python.
struct Lot {
  double width_m;
  double height_m;
  double resolution;
  depot::Grid2D<double> distance;

  depot::DistanceFieldView view() const {
    return depot::DistanceFieldView(depot::CostMapView(distance), resolution);
  }
};

struct Rectangle {
  double x0, y0, x1, y1;
};

Lot MakeLot(double width_m, double height_m, const std::vector<Rectangle>& obstacles,
            double resolution = 0.05) {
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
  depot::MaskView mask(reinterpret_cast<const bool*>(free.data()), rows, cols);
  depot::Grid2D<double> cells = depot::EuclideanDistanceTransform(mask);
  for (std::size_t i = 0; i < cells.size(); ++i) {
    cells.data()[i] = std::max(cells.data()[i] - std::sqrt(2.0), 0.0) * resolution;
  }
  return Lot{width_m, height_m, resolution, std::move(cells)};
}

/// A 30 x 20 m lot walled on all four sides.
Lot WalledLot() {
  const double w = 30.0, h = 20.0, t = 0.5;
  return MakeLot(w, h,
                 {{0.0, 0.0, w, t}, {0.0, h - t, w, h}, {0.0, 0.0, t, h}, {w - t, 0.0, w, h}});
}

depot::HybridOptions DefaultOptions() {
  depot::HybridOptions options;
  options.heading_tolerance = 5.0 * depot::kPi / 180.0;
  return options;
}

TEST(HeadingBin, PartitionsTheCircle) {
  EXPECT_EQ(depot::HeadingBin(0.0, 72), 0);
  EXPECT_EQ(depot::HeadingBin(depot::kTwoPi, 72), 0);
  EXPECT_EQ(depot::HeadingBin(-0.01, 72), 71);
  EXPECT_EQ(depot::HeadingBin(depot::kPi, 72), 36);
  for (int i = 0; i < 72; ++i) {
    const double theta = (i + 0.5) * depot::kTwoPi / 72;
    EXPECT_EQ(depot::HeadingBin(theta, 72), i);
  }
}

TEST(AtGoal, HonoursBothTolerances) {
  const depot::Pose goal{5.0, 5.0, 0.0};
  EXPECT_TRUE(depot::AtGoal({5.1, 5.1, 0.05}, goal, 0.3, 0.1));
  EXPECT_FALSE(depot::AtGoal({5.4, 5.0, 0.0}, goal, 0.3, 0.1));
  EXPECT_FALSE(depot::AtGoal({5.0, 5.0, 0.2}, goal, 0.3, 0.1));
  // The heading wraps, so -pi and +pi are the same direction.
  EXPECT_TRUE(depot::AtGoal({5.0, 5.0, -depot::kPi}, {5.0, 5.0, depot::kPi}, 0.3, 0.1));
}

TEST(CarModel, MatchesTheGeometryTheConfigDescribes) {
  const depot::CarModel car;
  EXPECT_DOUBLE_EQ(car.front_overhang(), 3.6);
  EXPECT_NEAR(car.min_turning_radius(), 2.7 / std::tan(0.6), 1e-12);
  EXPECT_NEAR(car.max_curvature(), 1.0 / car.min_turning_radius(), 1e-12);
  const auto offsets = car.disc_offsets();
  ASSERT_EQ(offsets.size(), 4u);
  EXPECT_NEAR(offsets.front(), -0.9 + 0.5 * 1.125, 1e-12);
  EXPECT_NEAR(offsets.back(), -0.9 + 3.5 * 1.125, 1e-12);
  // Every disc covers its slice of the footprint.
  EXPECT_GT(car.disc_radius(), car.width / 2.0);
}

TEST(CarModel, DrivesStraightWithNoSteering) {
  const depot::CarModel car;
  const depot::Pose end = car.Step({1.0, 2.0, 0.0}, 0.0, 3.0);
  EXPECT_NEAR(end.x, 4.0, 1e-12);
  EXPECT_NEAR(end.y, 2.0, 1e-12);
  EXPECT_NEAR(end.theta, 0.0, 1e-12);
}

TEST(CarModel, TurnsOnItsMinimumRadiusAtFullLock) {
  const depot::CarModel car;
  const double radius = car.min_turning_radius();
  const double quarter = radius * depot::kPi / 2.0;
  const depot::Pose end = car.Step({0.0, 0.0, 0.0}, car.max_steer, quarter);
  EXPECT_NEAR(std::fabs(depot::AngleDifference(end.theta, depot::kPi / 2.0)), 0.0, 1e-9);
  // A quarter turn to the left (y grows down) lands at (r, r).
  EXPECT_NEAR(end.x, radius, 1e-9);
  EXPECT_NEAR(end.y, radius, 1e-9);
}

TEST(Hybrid, RejectsAStartInsideAWall) {
  const Lot lot = WalledLot();
  const depot::CarModel car;
  const auto result = depot::HybridPlan({0.2, 0.2, 0.0}, {15.0, 10.0, 0.0}, car, lot.view(), {},
                                        1.0, DefaultOptions());
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, "start pose is in collision");
}

TEST(Hybrid, RejectsAGoalInsideAWall) {
  const Lot lot = WalledLot();
  const depot::CarModel car;
  const auto result = depot::HybridPlan({15.0, 10.0, 0.0}, {0.2, 0.2, 0.0}, car, lot.view(), {},
                                        1.0, DefaultOptions());
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, "goal pose is in collision");
}

TEST(Hybrid, CrossesAnOpenLot) {
  const Lot lot = WalledLot();
  const depot::CarModel car;
  const auto result = depot::HybridPlan({6.0, 10.0, 0.0}, {22.0, 10.0, 0.0}, car, lot.view(), {},
                                        1.0, DefaultOptions());
  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.poses.front().x, 6.0);
  EXPECT_TRUE(depot::AtGoal(result.poses.back(), {22.0, 10.0, 0.0}, 0.3, 5.0 * depot::kPi / 180.0));
  EXPECT_EQ(result.directions.size(), result.poses.size() - 1);
  EXPECT_GT(result.path_length_m, 15.0);
}

TEST(Hybrid, EveryPoseOnThePlanIsCollisionFree) {
  const Lot lot = WalledLot();
  const depot::CarModel car;
  const auto options = DefaultOptions();
  const auto result = depot::HybridPlan({6.0, 4.0, 0.0}, {22.0, 14.0, depot::kPi / 2.0}, car,
                                        lot.view(), {}, 1.0, options);
  ASSERT_TRUE(result.success);
  const depot::CarCollisionChecker checker(lot.view(), car, options.safety_margin);
  for (const depot::Pose& pose : result.poses) {
    EXPECT_TRUE(checker.IsFree(pose)) << "pose " << pose.x << ", " << pose.y;
  }
}

TEST(Hybrid, ConsecutivePosesAreCloseTogether) {
  const Lot lot = WalledLot();
  const depot::CarModel car;
  const auto options = DefaultOptions();
  const auto result =
      depot::HybridPlan({6.0, 10.0, 0.0}, {22.0, 12.0, 0.0}, car, lot.view(), {}, 1.0, options);
  ASSERT_TRUE(result.success);
  const double longest = options.arc_length / options.substeps + 1e-9;
  for (std::size_t i = 0; i + 1 < result.poses.size(); ++i) {
    const double hop = depot::PythonHypot(result.poses[i + 1].x - result.poses[i].x,
                                          result.poses[i + 1].y - result.poses[i].y);
    EXPECT_LE(hop, std::max(longest, options.analytic_step + 1e-9));
  }
}

TEST(Hybrid, HonoursTheExpansionCap) {
  const Lot lot = WalledLot();
  const depot::CarModel car;
  auto options = DefaultOptions();
  options.max_expansions = 5;
  options.analytic_enabled = false;
  const auto result =
      depot::HybridPlan({6.0, 10.0, 0.0}, {22.0, 14.0, 1.2}, car, lot.view(), {}, 1.0, options);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, "expansion limit reached");
  EXPECT_EQ(result.nodes_expanded, 5);
}

TEST(Hybrid, TheAnalyticExpansionIsWhatReachesTheExactPose) {
  const Lot lot = WalledLot();
  const depot::CarModel car;
  auto options = DefaultOptions();
  options.max_expansions = 4000;

  const auto with_analytic =
      depot::HybridPlan({6.0, 10.0, 0.0}, {21.3, 11.7, 0.4}, car, lot.view(), {}, 1.0, options);
  options.analytic_enabled = false;
  const auto without =
      depot::HybridPlan({6.0, 10.0, 0.0}, {21.3, 11.7, 0.4}, car, lot.view(), {}, 1.0, options);
  ASSERT_TRUE(with_analytic.success);
  EXPECT_TRUE(with_analytic.used_analytic);
  EXPECT_LT(with_analytic.nodes_expanded, without.nodes_expanded);
}

TEST(Hybrid, ReverseIsChargedMoreThanForward) {
  const Lot lot = WalledLot();
  const depot::CarModel car;
  auto options = DefaultOptions();
  options.analytic_enabled = false;

  const auto forward =
      depot::HybridPlan({6.0, 10.0, 0.0}, {12.0, 10.0, 0.0}, car, lot.view(), {}, 1.0, options);
  const auto backward =
      depot::HybridPlan({12.0, 10.0, 0.0}, {6.0, 10.0, 0.0}, car, lot.view(), {}, 1.0, options);
  ASSERT_TRUE(forward.success);
  ASSERT_TRUE(backward.success);
  // Driving to a pose behind you means either reversing or turning around, and
  // both are dearer than the same distance driven forwards.
  EXPECT_GT(backward.cost, forward.cost);
}

TEST(Hybrid, CountsOnePoseCheckPerPoseItLooksAt) {
  const Lot lot = WalledLot();
  const depot::CarModel car;
  auto options = DefaultOptions();
  options.analytic_enabled = false;
  options.max_expansions = 20;
  const auto result =
      depot::HybridPlan({6.0, 10.0, 0.0}, {22.0, 14.0, 1.2}, car, lot.view(), {}, 1.0, options);
  ASSERT_FALSE(result.success);
  ASSERT_EQ(result.nodes_expanded, options.max_expansions);
  // Two endpoint checks, then the whole fan of every expansion except the last,
  // which returns as soon as it sees the cap.
  const std::int64_t fan = 10 * options.substeps;
  EXPECT_EQ(result.collision_checks, 2 + (result.nodes_expanded - 1) * fan);
}

}  // namespace
