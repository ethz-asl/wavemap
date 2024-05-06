cmake_minimum_required(VERSION 3.10)

project(wavemap VERSION 1.6.3 LANGUAGES CXX)

# User options
option(USE_CLANG_TIDY "Generate necessary files to run clang-tidy" OFF)

# Dependencies
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PkgConfig REQUIRED)
pkg_check_modules(glog REQUIRED libglog)

# Optional dependencies
find_package(tracy QUIET)
# TODO(victorr): Check if tracy is correctly found and linked in when it is
#                available as an installed (system) package.

include(cmake/compiler_options.cmake)

# Libraries
add_subdirectory(src/core)
add_subdirectory(src/io)
add_subdirectory(src/pipeline)

## Tests
#if (CATKIN_ENABLE_TESTING)
#  add_subdirectory(test)
#endif ()

## Benchmarks
#if (ENABLE_BENCHMARKING)
#  add_subdirectory(benchmark)
#endif ()

# Tooling
if (USE_CLANG_TIDY)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif ()
