# Google Benchmark is fetched rather than vendored.
FetchContent_Declare(
  benchmark
  GIT_REPOSITORY https://github.com/google/benchmark.git
  GIT_TAG v1.9.1
  GIT_SHALLOW TRUE
)
set(BENCHMARK_ENABLE_TESTING OFF CACHE BOOL "" FORCE)
set(BENCHMARK_ENABLE_GTEST_TESTS OFF CACHE BOOL "" FORCE)
set(BENCHMARK_ENABLE_INSTALL OFF CACHE BOOL "" FORCE)
set(BENCHMARK_ENABLE_WERROR OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(benchmark)

add_executable(depot_bench
  bench_planners.cpp
)
target_include_directories(depot_bench PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(depot_bench PRIVATE depot_core benchmark::benchmark)
