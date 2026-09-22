# GoogleTest is fetched rather than vendored: no third-party source lives in
# this repository.
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG v1.15.2
  GIT_SHALLOW TRUE
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

add_executable(depot_tests
  test_distance_field.cpp
  test_grid_search.cpp
  test_hybrid.cpp
  test_reeds_shepp.cpp
  test_spacetime.cpp
)
target_link_libraries(depot_tests PRIVATE depot_core GTest::gtest_main)
target_compile_options(depot_tests PRIVATE -Wall -Wextra)

include(GoogleTest)
gtest_discover_tests(depot_tests DISCOVERY_MODE PRE_TEST)
