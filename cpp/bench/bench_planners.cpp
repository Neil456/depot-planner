// Google Benchmark suite for the planning core.
//
// Every benchmark runs on a fixed, deterministic scenario built by setup code
// that is excluded from the timed region, so the numbers are comparable across
// runs and directly against the Python reference. The Python side times the
// same three searches through the bindings; see depot_planner/eval/cpp_bench.py
// and the "Python vs C++" table in REPORT.md.

#include <benchmark/benchmark.h>

#include <vector>

#include "depot/grid_search.hpp"
#include "depot/hybrid.hpp"
#include "depot/spacetime.hpp"
#include "scenarios.hpp"

namespace {

// ----------------------------------------------------------------- grid A*

void RunGrid(benchmark::State& state, double weight, const char* label) {
  const depot::bench::GridScenario scenario = depot::bench::MakeDepotGrid();
  depot::GridSearchOptions options;
  options.weight = weight;
  std::int64_t expanded = 0;
  for (auto _ : state) {
    auto result = depot::GridSearch(scenario.view(), scenario.start, scenario.goal, options);
    expanded = result.nodes_expanded;
    benchmark::DoNotOptimize(result.cost);
  }
  state.counters["nodes"] = static_cast<double>(expanded);
  state.SetLabel(label);
}

void BM_GridDijkstra(benchmark::State& state) {
  RunGrid(state, 0.0, "depot 60x40, Dijkstra");
}
BENCHMARK(BM_GridDijkstra)->Unit(benchmark::kMillisecond);

void BM_GridAstar(benchmark::State& state) {
  RunGrid(state, 1.0, "depot 60x40, A*");
}
BENCHMARK(BM_GridAstar)->Unit(benchmark::kMillisecond);

void BM_GridWeightedAstar(benchmark::State& state) {
  RunGrid(state, 1.5, "depot 60x40, weighted A* (w=1.5)");
}
BENCHMARK(BM_GridWeightedAstar)->Unit(benchmark::kMillisecond);

// ------------------------------------------------------- the Dijkstra field

void BM_DijkstraField(benchmark::State& state) {
  const depot::bench::GridScenario scenario = depot::bench::MakeDepotGrid();
  for (auto _ : state) {
    auto field = depot::DijkstraField(scenario.view(), {scenario.goal});
    benchmark::DoNotOptimize(field.data());
  }
  state.SetLabel("depot 60x40, cost-to-go from the goal");
}
BENCHMARK(BM_DijkstraField)->Unit(benchmark::kMillisecond);

// ---------------------------------------------------------- space-time A*

void RunSpaceTime(benchmark::State& state, bool with_field, const char* label) {
  const depot::bench::GridScenario scenario = depot::bench::MakeDepotGrid();
  const depot::AgentOccupancy agents = depot::bench::MakeTraffic();
  const depot::CostMap field = depot::DijkstraField(scenario.view(), {scenario.goal});

  depot::SpaceTimeOptions options;
  options.min_cell_cost = depot::MinFiniteCost(scenario.view());
  options.horizon_cap = 400;

  std::int64_t expanded = 0;
  for (auto _ : state) {
    auto result = depot::SpaceTimePlan(
        scenario.view(), scenario.start, scenario.goal, 0, agents,
        with_field ? depot::CostMapView(field) : depot::CostMapView(), options);
    expanded = result.nodes_expanded;
    benchmark::DoNotOptimize(result.cost);
  }
  state.counters["nodes"] = static_cast<double>(expanded);
  state.SetLabel(label);
}

void BM_SpaceTimeGridHeuristic(benchmark::State& state) {
  RunSpaceTime(state, true, "6 vehicles, cost-to-go heuristic");
}
BENCHMARK(BM_SpaceTimeGridHeuristic)->Unit(benchmark::kMillisecond);

void BM_SpaceTimeOctileHeuristic(benchmark::State& state) {
  RunSpaceTime(state, false, "6 vehicles, octile heuristic");
}
BENCHMARK(BM_SpaceTimeOctileHeuristic)->Unit(benchmark::kMillisecond);

void BM_BaselineSnapshot(benchmark::State& state) {
  const depot::bench::GridScenario scenario = depot::bench::MakeDepotGrid();
  const depot::AgentOccupancy agents = depot::bench::MakeTraffic();
  const double min_cost = depot::MinFiniteCost(scenario.view());
  for (auto _ : state) {
    auto result = depot::BaselineSnapshotPlan(scenario.view(), scenario.start, scenario.goal, 0,
                                              agents, min_cost, {});
    benchmark::DoNotOptimize(result.cost);
  }
  state.SetLabel("6 vehicles frozen where they stand");
}
BENCHMARK(BM_BaselineSnapshot)->Unit(benchmark::kMillisecond);

// -------------------------------------------------------------- hybrid A*

void BM_HybridParallelParking(benchmark::State& state) {
  const depot::bench::ParkingScenario scenario = depot::bench::MakeParallelParking();
  const depot::CarModel car;
  depot::HybridOptions options;
  options.heading_tolerance = 5.0 * depot::kPi / 180.0;

  std::int64_t expanded = 0;
  for (auto _ : state) {
    auto result =
        depot::HybridPlan(scenario.start, scenario.goal, car, scenario.view(), {}, 1.0, options);
    expanded = result.nodes_expanded;
    benchmark::DoNotOptimize(result.cost);
  }
  state.counters["expansions"] = static_cast<double>(expanded);
  state.SetLabel("reverse into a 7 m parallel bay");
}
BENCHMARK(BM_HybridParallelParking)->Unit(benchmark::kMillisecond);

void BM_HybridNoAnalyticExpansion(benchmark::State& state) {
  const depot::bench::ParkingScenario scenario = depot::bench::MakeParallelParking();
  const depot::CarModel car;
  depot::HybridOptions options;
  options.heading_tolerance = 5.0 * depot::kPi / 180.0;
  options.analytic_enabled = false;
  options.max_expansions = 4000;

  for (auto _ : state) {
    auto result =
        depot::HybridPlan(scenario.start, scenario.goal, car, scenario.view(), {}, 1.0, options);
    benchmark::DoNotOptimize(result.cost);
  }
  state.SetLabel("same bay, Reeds-Shepp expansion disabled");
}
BENCHMARK(BM_HybridNoAnalyticExpansion)->Unit(benchmark::kMillisecond);

// ------------------------------------------------------- the distance field

void BM_DistanceField(benchmark::State& state) {
  const depot::bench::ParkingScenario scenario = depot::bench::MakeParallelParking();
  const int rows = scenario.distance.rows();
  const int cols = scenario.distance.cols();
  // A mask of the same size, so the transform does comparable work.
  std::vector<char> mask(static_cast<std::size_t>(rows) * cols, 1);
  for (int x = 0; x < cols; ++x) mask[x] = 0;
  for (int y = 0; y < rows; ++y) mask[static_cast<std::size_t>(y) * cols] = 0;
  const depot::MaskView view(reinterpret_cast<const bool*>(mask.data()), rows, cols);

  for (auto _ : state) {
    auto field = depot::EuclideanDistanceTransform(view);
    benchmark::DoNotOptimize(field.data());
  }
  state.SetLabel("parking lot at 0.05 m");
}
BENCHMARK(BM_DistanceField)->Unit(benchmark::kMillisecond);

}  // namespace

BENCHMARK_MAIN();
