cmake_minimum_required(VERSION 3.10)

project(wavemap VERSION 1.6.3 LANGUAGES CXX)

# Dependencies
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PkgConfig REQUIRED)
pkg_check_modules(glog REQUIRED libglog)

# TODO(victorr): Make sure these deps are also available to downstream packages.
#                This might require using an alternative to FetchContent.
include(FetchContent)
FetchContent_Declare(tracy
    GIT_REPOSITORY https://github.com/wolfpld/tracy.git
    GIT_TAG v0.10
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(tracy)
FetchContent_Declare(minkindr
    GIT_REPOSITORY https://github.com/ethz-asl/minkindr.git
    GIT_TAG master
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(minkindr)
# Add minkindr as header-only library
add_library(minkindr INTERFACE)
target_include_directories(minkindr INTERFACE ${minkindr_SOURCE_DIR}/minkindr/include)

include(cmake/compiler_options.cmake)

# Libraries
add_subdirectory(core)
add_subdirectory(io)
add_subdirectory(pipeline)

## Tests
#if (CATKIN_ENABLE_TESTING)
#  add_subdirectory(test)
#endif ()
#
## Benchmarks
#if (ENABLE_BENCHMARKING)
#  add_subdirectory(benchmark)
#endif ()
