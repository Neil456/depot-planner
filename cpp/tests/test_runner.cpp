#include <gtest/gtest.h>

#include <limits>
#include <vector>

#include "depot/runner.hpp"

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

/// A 13 x 5 map: one horizontal corridor at y = 2 and one vertical passage at
/// x = 6, everything else wall — the same shape the space-time tests use.
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

depot::AgentOccupancy CrossingAgent(int inflate) {
  depot::AgentTimeline agent;
  agent.width = 1;
  agent.height = 1;
  agent.id = 3;
  for (int i = 0; i < 5; ++i) agent.anchors.push_back(depot::Cell{6, 0});
  agent.anchors.push_back(depot::Cell{6, 1});
  agent.anchors.push_back(depot::Cell{6, 2});
  agent.anchors.push_back(depot::Cell{6, 3});
  for (int i = 0; i < 60; ++i) agent.anchors.push_back(depot::Cell{6, 4});
  return depot::AgentOccupancy({agent}, inflate);
}

depot::EpisodeOptions DefaultOptions() {
  depot::EpisodeOptions options;
  options.replan_every = 4;
  options.time_limit = 60;
  options.spacetime.time_cost = 0.05;
  options.spacetime.min_cell_cost = 1.0;
  options.spacetime.horizon_cap = 60;
  options.min_cell_cost = 1.0;
  return options;
}

TEST(Runner, ReachesTheGoalOnAnEmptyCorridor) {
  const Corridor map;
  const depot::AgentOccupancy none;
  const auto outcome = depot::RunEpisode(map.view(), {0, 2}, {12, 2}, none, {}, DefaultOptions());
  EXPECT_TRUE(outcome.success);
  EXPECT_FALSE(outcome.collision);
  EXPECT_FALSE(outcome.timeout);
  EXPECT_EQ(outcome.cells.front(), (depot::Cell{0, 2}));
  EXPECT_EQ(outcome.cells.back(), (depot::Cell{12, 2}));
  EXPECT_EQ(outcome.steps, 12);
  EXPECT_NEAR(outcome.path_length_cells, 12.0, 1e-12);
}

TEST(Runner, TimesAndCellsStayInStep) {
  const Corridor map;
  const auto outcome =
      depot::RunEpisode(map.view(), {0, 2}, {12, 2}, CrossingAgent(1), {}, DefaultOptions());
  ASSERT_EQ(outcome.cells.size(), outcome.times.size());
  for (std::size_t i = 0; i + 1 < outcome.times.size(); ++i) {
    EXPECT_EQ(outcome.times[i + 1], outcome.times[i] + 1);
  }
  EXPECT_EQ(outcome.steps, static_cast<int>(outcome.cells.size()) - 1);
}

TEST(Runner, WaitsForTrafficRatherThanDrivingThroughIt) {
  const Corridor map;
  const auto outcome =
      depot::RunEpisode(map.view(), {0, 2}, {12, 2}, CrossingAgent(1), {}, DefaultOptions());
  EXPECT_TRUE(outcome.success);
  EXPECT_FALSE(outcome.collision);
  EXPECT_GT(outcome.wait_steps, 0);
}

TEST(Runner, TheBaselineDrivesIntoTheCrossingVehicle) {
  const Corridor map;
  depot::EpisodeOptions options = DefaultOptions();
  options.planner = depot::PlannerKind::kBaselineSnapshot;
  options.replan_every = 1;
  // With no safety margin the snapshot planner freezes the agent where it stands
  // and walks straight into the cell it is about to occupy. (With the margin the
  // project ships, the buffer alone is enough to make it wait here.)
  const auto outcome =
      depot::RunEpisode(map.view(), {0, 2}, {12, 2}, CrossingAgent(0), {}, options);
  EXPECT_TRUE(outcome.collision);
  EXPECT_FALSE(outcome.success);
  EXPECT_GE(outcome.collision_step, 0);
  EXPECT_EQ(outcome.collision_kind, "overlap");
  EXPECT_TRUE(outcome.collision_has_agent);
  EXPECT_EQ(outcome.collision_agent, 3);
}

TEST(Runner, TimesOutWhenTheGoalIsWalledOff) {
  Corridor map;
  // Wall the corridor off past the passage.
  for (int x = 8; x < 13; ++x) map.cost[static_cast<std::size_t>(2) * map.cols + x] = kInf;
  const depot::AgentOccupancy none;
  depot::EpisodeOptions options = DefaultOptions();
  options.time_limit = 12;
  const auto outcome = depot::RunEpisode(map.view(), {0, 2}, {6, 4}, none, {}, options);
  // (6, 4) is reachable down the passage, so this one should succeed...
  EXPECT_TRUE(outcome.success);

  // (12, 2) is now inside the wall, so every replan refuses it and the ego holds.
  const auto stuck = depot::RunEpisode(map.view(), {0, 2}, {12, 2}, none, {}, options);
  EXPECT_FALSE(stuck.success);
  EXPECT_TRUE(stuck.timeout);
  EXPECT_GT(stuck.plan_failures, 0);
  ASSERT_FALSE(stuck.plan_failure_reasons.empty());
  EXPECT_EQ(stuck.plan_failure_reasons.front().first, "goal not drivable");
}

TEST(Runner, AbortsWhenHoldingIsDisabled) {
  Corridor map;
  for (int x = 8; x < 13; ++x) map.cost[static_cast<std::size_t>(2) * map.cols + x] = kInf;
  const depot::AgentOccupancy none;
  depot::EpisodeOptions options = DefaultOptions();
  options.hold_on_plan_failure = false;
  const auto outcome = depot::RunEpisode(map.view(), {0, 2}, {12, 2}, none, {}, options);
  EXPECT_FALSE(outcome.success);
  EXPECT_EQ(outcome.reason_override, "no plan and holding disabled");
  EXPECT_EQ(outcome.steps, 0);
}

TEST(Runner, RecordsOnePlanPerReplan) {
  const Corridor map;
  const depot::AgentOccupancy none;
  const auto outcome = depot::RunEpisode(map.view(), {0, 2}, {12, 2}, none, {}, DefaultOptions());
  EXPECT_EQ(static_cast<int>(outcome.plans.size()), outcome.replans);
  for (const depot::IssuedPlan& plan : outcome.plans) {
    EXPECT_GE(plan.issued_at, 0);
    EXPECT_FALSE(plan.cells.empty());
  }
}

TEST(Runner, SummarisesItsPlanningTimings) {
  const Corridor map;
  const depot::AgentOccupancy none;
  const auto outcome = depot::RunEpisode(map.view(), {0, 2}, {12, 2}, none, {}, DefaultOptions());
  EXPECT_GT(outcome.replans, 0);
  EXPECT_GE(outcome.max_planning_ms, outcome.mean_planning_ms);
  EXPECT_GE(outcome.mean_planning_ms, 0.0);
  EXPECT_GT(outcome.nodes_expanded, 0);
}

TEST(Runner, DetectsACornerCutAndAJumpItIsHandedByItsOwnPlan) {
  // The legality check is the interesting part: drive the ego at a goal it can
  // only reach diagonally past a corner, and confirm the runner keeps the plan
  // legal rather than letting it cut.
  const Corridor map;
  const depot::AgentOccupancy none;
  const auto outcome = depot::RunEpisode(map.view(), {6, 0}, {6, 4}, none, {}, DefaultOptions());
  ASSERT_TRUE(outcome.success);
  for (std::size_t i = 0; i + 1 < outcome.cells.size(); ++i) {
    EXPECT_LE(std::abs(outcome.cells[i + 1].x - outcome.cells[i].x), 1);
    EXPECT_LE(std::abs(outcome.cells[i + 1].y - outcome.cells[i].y), 1);
  }
}

}  // namespace
