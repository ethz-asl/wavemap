cmake_minimum_required(VERSION 3.10)
project(wavemap VERSION 1.6.3 LANGUAGES CXX)

# CMake helpers
include(cmake/wavemap-extras.cmake)

# Enable general wavemap tooling (e.g. to run clang-tidy CI)
enable_wavemap_general_tooling()

# Libraries
add_subdirectory(src)

# Tests
if (ENABLE_TESTING OR CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif ()

# Benchmarks
if (ENABLE_BENCHMARKING)
  add_subdirectory(benchmark)
endif ()
