cmake_minimum_required(VERSION 3.10)
project(wavemap VERSION 1.6.3 LANGUAGES CXX)

# User options
option(USE_CLANG_TIDY "Generate necessary files to run clang-tidy" OFF)

# CMake helpers
include(cmake/wavemap-extras.cmake)

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

# Tooling
if (USE_CLANG_TIDY)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif ()
