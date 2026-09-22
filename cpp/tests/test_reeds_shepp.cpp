#include <gtest/gtest.h>

#include <cmath>
#include <random>
#include <vector>

#include "depot/car.hpp"
#include "depot/numeric.hpp"
#include "depot/reeds_shepp.hpp"

namespace {

depot::CarModel DefaultCar() {
  return depot::CarModel{};
}

/// Walk the curve with the bicycle model and report where the car ends up.
depot::Pose EndOf(const depot::Pose& start, const depot::RSPath& path, const depot::CarModel& car,
                  double step) {
  std::vector<depot::Pose> poses;
  std::vector<int> directions;
  depot::InterpolateReedsShepp(start, path, car, step, &poses, &directions);
  return poses.back();
}

TEST(ReedsShepp, AStraightShotIsOneForwardSegment) {
  const depot::CarModel car = DefaultCar();
  bool found = false;
  const auto path =
      depot::ShortestReedsShepp({0.0, 0.0, 0.0}, {5.0, 0.0, 0.0}, car.max_curvature(), &found);
  ASSERT_TRUE(found);
  ASSERT_EQ(path.segments.size(), 1u);
  EXPECT_EQ(path.segments[0].steering, depot::kStraight);
  EXPECT_GT(path.segments[0].length, 0.0);
  EXPECT_NEAR(depot::ReedsSheppLengthMetres(path, car), 5.0, 1e-9);
}

TEST(ReedsShepp, DrivingStraightBackwardsIsOneReverseSegment) {
  const depot::CarModel car = DefaultCar();
  bool found = false;
  const auto path =
      depot::ShortestReedsShepp({0.0, 0.0, 0.0}, {-3.0, 0.0, 0.0}, car.max_curvature(), &found);
  ASSERT_TRUE(found);
  ASSERT_EQ(path.segments.size(), 1u);
  EXPECT_LT(path.segments[0].length, 0.0);
  EXPECT_NEAR(depot::ReedsSheppLengthMetres(path, car), 3.0, 1e-9);
  EXPECT_EQ(path.DirectionChanges(), 0);
}

TEST(ReedsShepp, AnIdenticalPoseTakesTheShortestLoop) {
  // Zero-length segments are dropped, so the degenerate "stay put" word produces
  // no path at all and the shortest candidate is a full turn out and back. The
  // Python reference behaves the same way, and hybrid A* never asks for this
  // case because it has already reached the goal by then.
  const depot::CarModel car = DefaultCar();
  bool found = false;
  const auto path =
      depot::ShortestReedsShepp({2.0, 3.0, 0.7}, {2.0, 3.0, 0.7}, car.max_curvature(), &found);
  ASSERT_TRUE(found);
  EXPECT_NEAR(path.Length(), depot::kTwoPi, 1e-12);
  const depot::Pose end = EndOf({2.0, 3.0, 0.7}, path, car, 0.05);
  EXPECT_NEAR(end.x, 2.0, 1e-9);
  EXPECT_NEAR(end.y, 3.0, 1e-9);
}

TEST(ReedsShepp, EveryCandidateLandsOnTheGoal) {
  // The same numerical validation the Python implementation was held to: walk
  // each curve with the bicycle model and check where the car ends up.
  const depot::CarModel car = DefaultCar();
  std::mt19937_64 rng(4242);
  std::uniform_real_distribution<double> position(-12.0, 12.0);
  std::uniform_real_distribution<double> heading(-depot::kPi, depot::kPi);

  int validated = 0;
  for (int trial = 0; trial < 3000; ++trial) {
    const depot::Pose start{position(rng), position(rng), heading(rng)};
    const depot::Pose goal{position(rng), position(rng), heading(rng)};
    const auto candidates = depot::ReedsSheppPaths(start, goal, car.max_curvature());
    for (const auto& path : candidates) {
      // 0.2 m is the interpolation step the planner actually uses; a much finer
      // one only accumulates more rounding in the integration.
      const depot::Pose end = EndOf(start, path, car, 0.2);
      ASSERT_NEAR(end.x, goal.x, 1e-7) << "trial " << trial;
      ASSERT_NEAR(end.y, goal.y, 1e-7) << "trial " << trial;
      ASSERT_NEAR(std::fabs(depot::AngleDifference(end.theta, goal.theta)), 0.0, 1e-7)
          << "trial " << trial;
      ++validated;
    }
  }
  EXPECT_GT(validated, 3000);
}

TEST(ReedsShepp, TheInterpolatedLengthMatchesTheAnalyticOne) {
  const depot::CarModel car = DefaultCar();
  std::mt19937_64 rng(99);
  std::uniform_real_distribution<double> position(-10.0, 10.0);
  std::uniform_real_distribution<double> heading(-depot::kPi, depot::kPi);

  for (int trial = 0; trial < 500; ++trial) {
    const depot::Pose start{position(rng), position(rng), heading(rng)};
    const depot::Pose goal{position(rng), position(rng), heading(rng)};
    bool found = false;
    const auto path = depot::ShortestReedsShepp(start, goal, car.max_curvature(), &found);
    if (!found) continue;

    std::vector<depot::Pose> poses;
    std::vector<int> directions;
    depot::InterpolateReedsShepp(start, path, car, 0.005, &poses, &directions);
    double walked = 0.0;
    for (std::size_t i = 0; i + 1 < poses.size(); ++i) {
      walked += depot::PythonHypot(poses[i + 1].x - poses[i].x, poses[i + 1].y - poses[i].y);
    }
    // Chords undercut arcs, so the walked length is a hair short of the true one.
    const double analytic = depot::ReedsSheppLengthMetres(path, car);
    EXPECT_LE(walked, analytic + 1e-9) << "trial " << trial;
    EXPECT_GT(walked, analytic - 1e-3) << "trial " << trial;
    EXPECT_EQ(directions.size(), poses.size() - 1);
  }
}

TEST(ReedsShepp, TheShortestCandidateIsTheShortest) {
  const depot::CarModel car = DefaultCar();
  std::mt19937_64 rng(7);
  std::uniform_real_distribution<double> position(-8.0, 8.0);
  std::uniform_real_distribution<double> heading(-depot::kPi, depot::kPi);

  for (int trial = 0; trial < 500; ++trial) {
    const depot::Pose start{position(rng), position(rng), heading(rng)};
    const depot::Pose goal{position(rng), position(rng), heading(rng)};
    bool found = false;
    const auto best = depot::ShortestReedsShepp(start, goal, car.max_curvature(), &found);
    if (!found) continue;
    for (const auto& path : depot::ReedsSheppPaths(start, goal, car.max_curvature())) {
      EXPECT_LE(best.Length(), path.Length() + 1e-12);
    }
  }
}

TEST(ReedsShepp, GearsFollowTheSegmentSigns) {
  const depot::CarModel car = DefaultCar();
  depot::RSPath path;
  path.segments = {{depot::kLeft, 0.5}, {depot::kStraight, -1.0}, {depot::kRight, 0.5}};
  std::vector<depot::Pose> poses;
  std::vector<int> directions;
  depot::InterpolateReedsShepp({0.0, 0.0, 0.0}, path, car, 0.2, &poses, &directions);
  EXPECT_EQ(path.DirectionChanges(), 2);
  EXPECT_EQ(directions.front(), 1);
  EXPECT_EQ(directions.back(), 1);
  bool saw_reverse = false;
  for (int gear : directions) saw_reverse = saw_reverse || gear < 0;
  EXPECT_TRUE(saw_reverse);
}

// ------------------------------------------------------------ the numerics

TEST(Numeric, WrapsAnglesToTheHalfOpenInterval) {
  EXPECT_DOUBLE_EQ(depot::WrapAngle(0.0), 0.0);
  EXPECT_DOUBLE_EQ(depot::WrapAngle(depot::kPi), depot::kPi);
  EXPECT_DOUBLE_EQ(depot::WrapAngle(-depot::kPi), depot::kPi);
  EXPECT_NEAR(depot::WrapAngle(3.0 * depot::kPi), depot::kPi, 1e-12);
  EXPECT_NEAR(depot::AngleDifference(0.1, -0.1), 0.2, 1e-12);
}

TEST(Numeric, PythonModTakesTheSignOfTheDivisor) {
  EXPECT_DOUBLE_EQ(depot::PythonMod(-1.0, depot::kTwoPi), depot::kTwoPi - 1.0);
  EXPECT_DOUBLE_EQ(depot::PythonMod(1.0, depot::kTwoPi), 1.0);
  EXPECT_DOUBLE_EQ(std::fmod(-1.0, depot::kTwoPi), -1.0);  // what C would give
}

TEST(Numeric, HypotAgreesWithTheLibraryToAFewUlp) {
  std::mt19937_64 rng(5);
  std::uniform_real_distribution<double> value(-50.0, 50.0);
  for (int i = 0; i < 20000; ++i) {
    const double a = value(rng);
    const double b = value(rng);
    EXPECT_NEAR(depot::PythonHypot(a, b), std::hypot(a, b), 1e-12);
  }
  EXPECT_DOUBLE_EQ(depot::PythonHypot(3.0, 4.0), 5.0);
  EXPECT_DOUBLE_EQ(depot::PythonHypot(0.0, 0.0), 0.0);
}

TEST(Numeric, FloorDivisionMatchesPython) {
  EXPECT_DOUBLE_EQ(depot::PythonFloorDiv(7.0, 2.0), 3.0);
  EXPECT_DOUBLE_EQ(depot::PythonFloorDiv(-7.0, 2.0), -4.0);
  EXPECT_DOUBLE_EQ(depot::PythonFloorDiv(0.3, 0.1), 2.0);  // not 3: 0.3 / 0.1 < 3
}

}  // namespace
