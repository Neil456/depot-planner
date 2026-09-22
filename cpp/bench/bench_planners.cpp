// Google Benchmark suite for the planning core.
//
// Every benchmark runs on a fixed, deterministic scenario built in setup code
// that is excluded from the timed region, so the numbers are comparable across
// runs and directly against the Python reference.

#include <benchmark/benchmark.h>

#include <limits>
#include <vector>

#include "depot/grid_search.hpp"
#include "scenarios.hpp"

namespace {

void BM_GridDijkstra(benchmark::State& state) {
  const depot::bench::GridScenario scenario = depot::bench::MakeDepotGrid();
  depot::GridSearchOptions options;
  options.weight = 0.0;
  for (auto _ : state) {
    auto result =
        depot::GridSearch(depot::CostMapView(scenario.cost.data(), scenario.rows, scenario.cols),
                          scenario.start, scenario.goal, options);
    benchmark::DoNotOptimize(result.cost);
  }
  state.SetLabel("depot 60x40, Dijkstra");
}
BENCHMARK(BM_GridDijkstra)->Unit(benchmark::kMillisecond);

void BM_GridAstar(benchmark::State& state) {
  const depot::bench::GridScenario scenario = depot::bench::MakeDepotGrid();
  depot::GridSearchOptions options;
  options.weight = 1.0;
  for (auto _ : state) {
    auto result =
        depot::GridSearch(depot::CostMapView(scenario.cost.data(), scenario.rows, scenario.cols),
                          scenario.start, scenario.goal, options);
    benchmark::DoNotOptimize(result.cost);
  }
  state.SetLabel("depot 60x40, A*");
}
BENCHMARK(BM_GridAstar)->Unit(benchmark::kMillisecond);

void BM_GridWeightedAstar(benchmark::State& state) {
  const depot::bench::GridScenario scenario = depot::bench::MakeDepotGrid();
  depot::GridSearchOptions options;
  options.weight = 1.5;
  for (auto _ : state) {
    auto result =
        depot::GridSearch(depot::CostMapView(scenario.cost.data(), scenario.rows, scenario.cols),
                          scenario.start, scenario.goal, options);
    benchmark::DoNotOptimize(result.cost);
  }
  state.SetLabel("depot 60x40, weighted A* (w=1.5)");
}
BENCHMARK(BM_GridWeightedAstar)->Unit(benchmark::kMillisecond);

}  // namespace

BENCHMARK_MAIN();
