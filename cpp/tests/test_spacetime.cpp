#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "depot/spacetime.hpp"

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

/// A 13 x 5 map: one horizontal corridor at y = 2 and one vertical passage at
/// x = 6, everything else wall. This is the scenario the Python test suite uses
/// to force the ego to wait.
struct Corridor {
  std::vector<double> cost;
  int rows = 5;
  int cols = 13;

  Corridor() {
    cost.assign(static_cast<std::size_t>(rows) * cols, kInf);
    for (int x = 0; x < cols; ++x) cost[static_cast<std::size_t>(2) * cols + x] = 1.0;
    for (int y = 0; y < rows; ++y) cost[static_cast<std::size_t>(y) * cols + 6] = 1.0;
  }

  depot::CostMapView view() const { return depot::CostMapView(cost.data(), rows, cols); }
};

/// One 1x1 agent that walks down the passage across the corridor.
depot::AgentOccupancy CrossingAgent(int inflate) {
  depot::AgentTimeline agent;
  agent.width = 1;
  agent.height = 1;
  for (int i = 0; i < 5; ++i) agent.anchors.push_back(depot::Cell{6, 0});
  agent.anchors.push_back(depot::Cell{6, 1});
  agent.anchors.push_back(depot::Cell{6, 2});
  agent.anchors.push_back(depot::Cell{6, 3});
  for (int i = 0; i < 40; ++i) agent.anchors.push_back(depot::Cell{6, 4});
  return depot::AgentOccupancy({agent}, inflate);
}

depot::SpaceTimeOptions DefaultOptions() {
  depot::SpaceTimeOptions options;
  options.time_cost = 0.05;
  options.relax_margin_when_inside = true;
  options.heuristic_includes_time = true;
  options.min_cell_cost = 1.0;
  options.horizon_cap = 700;
  return options;
}

TEST(AgentOccupancy, CoversItsFootprintAndItsMargin) {
  depot::AgentTimeline agent;
  agent.width = 2;
  agent.height = 2;
  agent.anchors.push_back(depot::Cell{4, 4});
  const depot::AgentOccupancy occupancy({agent}, 1);

  EXPECT_TRUE(occupancy.InBody(4, 4, 0));
  EXPECT_TRUE(occupancy.InBody(5, 5, 0));
  EXPECT_FALSE(occupancy.InBody(6, 5, 0));
  EXPECT_TRUE(occupancy.InMargin(6, 5, 0));
  EXPECT_TRUE(occupancy.InMargin(3, 3, 0));
  EXPECT_FALSE(occupancy.InMargin(7, 4, 0));
}

TEST(AgentOccupancy, HoldsItsLastAnchorBeyondTheTimeline) {
  depot::AgentTimeline agent;
  agent.width = 1;
  agent.height = 1;
  agent.anchors = {depot::Cell{0, 0}, depot::Cell{1, 0}};
  const depot::AgentOccupancy occupancy({agent}, 0);
  EXPECT_TRUE(occupancy.InBody(1, 0, 1));
  EXPECT_TRUE(occupancy.InBody(1, 0, 99));
  EXPECT_FALSE(occupancy.InBody(0, 0, 99));
}

TEST(AgentOccupancy, DetectsASwap) {
  depot::AgentTimeline agent;
  agent.width = 1;
  agent.height = 1;
  agent.anchors = {depot::Cell{3, 0}, depot::Cell{2, 0}};
  const depot::AgentOccupancy occupancy({agent}, 0);
  EXPECT_TRUE(occupancy.Swaps(depot::Cell{2, 0}, depot::Cell{3, 0}, 0));
  EXPECT_FALSE(occupancy.Swaps(depot::Cell{1, 0}, depot::Cell{2, 0}, 0));
}

TEST(SpaceTime, ReachesTheGoalOnAnEmptyCorridor) {
  const Corridor map;
  const depot::AgentOccupancy none;
  const auto result =
      depot::SpaceTimePlan(map.view(), {0, 2}, {12, 2}, 0, none, {}, DefaultOptions());
  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.cells.front(), (depot::Cell{0, 2}));
  EXPECT_EQ(result.cells.back(), (depot::Cell{12, 2}));
  EXPECT_EQ(result.wait_steps, 0);
  EXPECT_EQ(result.times.front(), 0);
  EXPECT_EQ(result.times.size(), result.cells.size());
}

TEST(SpaceTime, TimeIsContiguousAlongThePlan) {
  const Corridor map;
  const auto result =
      depot::SpaceTimePlan(map.view(), {0, 2}, {12, 2}, 3, CrossingAgent(1), {}, DefaultOptions());
  ASSERT_TRUE(result.success);
  EXPECT_EQ(result.times.front(), 3);
  for (std::size_t i = 0; i + 1 < result.times.size(); ++i) {
    EXPECT_EQ(result.times[i + 1], result.times[i] + 1);
  }
}

TEST(SpaceTime, WaitsRatherThanDrivingThroughTheAgent) {
  const Corridor map;
  const auto result =
      depot::SpaceTimePlan(map.view(), {0, 2}, {12, 2}, 0, CrossingAgent(1), {}, DefaultOptions());
  ASSERT_TRUE(result.success);
  EXPECT_GT(result.wait_steps, 0);
  // Never inside the agent's own cell at the time it is there.
  const depot::AgentOccupancy occupancy = CrossingAgent(0);
  for (std::size_t i = 0; i < result.cells.size(); ++i) {
    EXPECT_FALSE(occupancy.InBody(result.cells[i].x, result.cells[i].y, result.times[i]));
  }
}

TEST(SpaceTime, ChargesTimeAndMovementSeparately) {
  const Corridor map;
  const depot::AgentOccupancy none;
  depot::SpaceTimeOptions options = DefaultOptions();
  const auto result = depot::SpaceTimePlan(map.view(), {0, 2}, {12, 2}, 0, none, {}, options);
  ASSERT_TRUE(result.success);
  const int steps = static_cast<int>(result.cells.size()) - 1;
  EXPECT_NEAR(result.cost, result.movement_cost + steps * options.time_cost, 1e-9);
  EXPECT_NEAR(result.movement_cost, 12.0, 1e-9);
}

TEST(SpaceTime, AZeroTimeCostMakesWaitingFree) {
  const Corridor map;
  depot::SpaceTimeOptions options = DefaultOptions();
  options.time_cost = 0.0;
  const auto result =
      depot::SpaceTimePlan(map.view(), {0, 2}, {12, 2}, 0, CrossingAgent(1), {}, options);
  ASSERT_TRUE(result.success);
  EXPECT_NEAR(result.cost, result.movement_cost, 1e-12);
}

TEST(SpaceTime, RefusesAStartBeyondTheHorizon) {
  const Corridor map;
  const depot::AgentOccupancy none;
  depot::SpaceTimeOptions options = DefaultOptions();
  options.horizon_cap = 10;
  const auto result = depot::SpaceTimePlan(map.view(), {0, 2}, {12, 2}, 11, none, {}, options);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, "start time beyond the horizon");
}

TEST(SpaceTime, GivesUpWhenTheHorizonIsTooShort) {
  const Corridor map;
  const depot::AgentOccupancy none;
  depot::SpaceTimeOptions options = DefaultOptions();
  options.horizon_cap = 4;  // twelve moves are needed
  const auto result = depot::SpaceTimePlan(map.view(), {0, 2}, {12, 2}, 0, none, {}, options);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, "no timed path to the goal");
}

TEST(SpaceTime, HonoursTheNodeAndTimeCaps) {
  const Corridor map;
  const depot::AgentOccupancy none;
  depot::SpaceTimeOptions options = DefaultOptions();
  options.limits.max_nodes = 3;
  auto result = depot::SpaceTimePlan(map.view(), {0, 2}, {12, 2}, 0, none, {}, options);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, "node limit reached");
  EXPECT_EQ(result.nodes_expanded, 3);

  options = DefaultOptions();
  options.limits.time_limit_ms = 0.0;
  result = depot::SpaceTimePlan(map.view(), {0, 2}, {12, 2}, 0, none, {}, options);
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, "time limit reached");
}

TEST(SpaceTime, RejectsEndpointsThatAreNotDrivable) {
  const Corridor map;
  const depot::AgentOccupancy none;
  const auto options = DefaultOptions();
  EXPECT_EQ(depot::SpaceTimePlan(map.view(), {0, 0}, {12, 2}, 0, none, {}, options).reason,
            "start not drivable");
  EXPECT_EQ(depot::SpaceTimePlan(map.view(), {0, 2}, {12, 0}, 0, none, {}, options).reason,
            "goal not drivable");
}

TEST(SpaceTime, TheCostToGoFieldSteersTheSearch) {
  const Corridor map;
  const depot::AgentOccupancy none;
  const auto field = depot::DijkstraField(map.view(), {{12, 2}});
  const auto with_field = depot::SpaceTimePlan(map.view(), {0, 2}, {12, 2}, 0, none,
                                               depot::CostMapView(field), DefaultOptions());
  const auto without =
      depot::SpaceTimePlan(map.view(), {0, 2}, {12, 2}, 0, none, {}, DefaultOptions());
  ASSERT_TRUE(with_field.success);
  ASSERT_TRUE(without.success);
  EXPECT_NEAR(with_field.cost, without.cost, 1e-9);
  EXPECT_LE(with_field.nodes_expanded, without.nodes_expanded);
}

// ------------------------------------------------------------ the baseline

TEST(BaselineSnapshot, TreatsAgentsAsStaticObstacles) {
  const Corridor map;
  depot::AgentTimeline blocker;
  blocker.width = 1;
  blocker.height = 1;
  blocker.anchors.push_back(depot::Cell{6, 2});
  const depot::AgentOccupancy occupancy({blocker}, 1);
  const auto blocked =
      depot::BaselineSnapshotPlan(map.view(), {0, 2}, {12, 2}, 0, occupancy, 1.0, {});
  EXPECT_FALSE(blocked.success);

  const depot::AgentOccupancy none;
  const auto clear = depot::BaselineSnapshotPlan(map.view(), {0, 2}, {12, 2}, 0, none, 1.0, {});
  EXPECT_TRUE(clear.success);
}

TEST(BaselineSnapshot, NeverDeclaresTheEgosOwnCellUnusable) {
  const Corridor map;
  depot::AgentTimeline neighbour;
  neighbour.width = 1;
  neighbour.height = 1;
  neighbour.anchors.push_back(depot::Cell{1, 2});  // its margin covers the ego at (0, 2)
  const depot::AgentOccupancy occupancy({neighbour}, 1);
  const auto result =
      depot::BaselineSnapshotPlan(map.view(), {0, 2}, {12, 2}, 0, occupancy, 1.0, {});
  // The ego's cell stays usable, so this is a clean "no path", not a rejected start.
  EXPECT_FALSE(result.success);
  EXPECT_EQ(result.reason, "goal unreachable");
}

TEST(BaselineSnapshot, FollowsTheAgentsThroughTime) {
  const Corridor map;
  depot::AgentTimeline mover;
  mover.width = 1;
  mover.height = 1;
  mover.anchors = {depot::Cell{6, 2}, depot::Cell{6, 2}, depot::Cell{6, 4}};
  const depot::AgentOccupancy occupancy({mover}, 1);
  EXPECT_FALSE(
      depot::BaselineSnapshotPlan(map.view(), {0, 2}, {12, 2}, 0, occupancy, 1.0, {}).success);
  EXPECT_TRUE(
      depot::BaselineSnapshotPlan(map.view(), {0, 2}, {12, 2}, 2, occupancy, 1.0, {}).success);
}

}  // namespace
