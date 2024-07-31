cmake_minimum_required(VERSION 3.10)
project(wavemap VERSION 2.0.0 LANGUAGES CXX)

# CMake helpers and general wavemap tooling (e.g. to run clang-tidy CI)
include(cmake/wavemap-extras.cmake)
enable_wavemap_general_tooling()

# Dependencies
option(USE_SYSTEM_EIGEN "Use system pre-installed Eigen" ON)
option(USE_SYSTEM_GLOG "Use system pre-installed glog" ON)
option(USE_SYSTEM_BOOST "Use system pre-installed Boost" ON)
include(cmake/find_dependencies.cmake)

# Libraries
add_subdirectory(src/core)
add_subdirectory(src/io)
add_subdirectory(src/pipeline)

# Tests
if (ENABLE_TESTING OR CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif ()

# Benchmarks
if (ENABLE_BENCHMARKING)
  add_subdirectory(benchmark)
endif ()
