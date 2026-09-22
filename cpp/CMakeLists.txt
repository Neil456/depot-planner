cmake_minimum_required(VERSION 3.20)
project(depot_core LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

option(DEPOT_BUILD_PYTHON "Build the pybind11 extension module" ON)
option(DEPOT_BUILD_TESTS "Build the GoogleTest binary (fetches GoogleTest)" OFF)
option(DEPOT_BUILD_BENCH "Build the Google Benchmark binary (fetches benchmark)" OFF)
option(DEPOT_SANITIZE "Build with AddressSanitizer and UndefinedBehaviorSanitizer" OFF)

# ----------------------------------------------------------------- the library

add_library(depot_core STATIC
  src/distance_field.cpp
  src/grid_search.cpp
  src/spacetime.cpp
)
target_include_directories(depot_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_compile_options(depot_core PRIVATE -Wall -Wextra)

if(DEPOT_SANITIZE)
  target_compile_options(depot_core PUBLIC -fsanitize=address,undefined -fno-omit-frame-pointer -g)
  target_link_options(depot_core PUBLIC -fsanitize=address,undefined)
endif()

# ------------------------------------------------------------ python extension

if(DEPOT_BUILD_PYTHON)
  find_package(Python COMPONENTS Interpreter Development.Module REQUIRED)
  find_package(pybind11 CONFIG REQUIRED)

  pybind11_add_module(_cpp MODULE bindings/module.cpp)
  target_link_libraries(_cpp PRIVATE depot_core)
  install(TARGETS _cpp DESTINATION depot_planner)
endif()

# ------------------------------------------------------------- tests and bench

if(DEPOT_BUILD_TESTS OR DEPOT_BUILD_BENCH)
  include(FetchContent)
  set(FETCHCONTENT_QUIET OFF)
endif()

if(DEPOT_BUILD_TESTS)
  enable_testing()
  add_subdirectory(tests)
endif()

if(DEPOT_BUILD_BENCH)
  add_subdirectory(bench)
endif()
