cmake_minimum_required(VERSION 3.0.2)
project(wavemap_common)

# Dependencies
find_package(catkin REQUIRED COMPONENTS glog_catkin eigen_catkin minkindr)
if (ENABLE_BENCHMARKING)
  find_package(benchmark REQUIRED)
endif ()

# Compiler definitions and options
include(cmake/wavemap_common-extras.cmake)
add_wavemap_compile_definitions_and_options()

# Setup catkin package
catkin_package(
  INCLUDE_DIRS
  include
  test/include
  LIBRARIES
  ${PROJECT_NAME}
  CATKIN_DEPENDS
  glog_catkin
  eigen_catkin
  minkindr
  CFG_EXTRAS
  wavemap_common-extras.cmake)
# NOTE: The test headers are normally not exported (e.g. by adding test/include
# to the catkin_package's INCLUDE_DIRS), but we make an exception for this base
# package's test headers, such that common tooling and test fixtures can be
# reused in downstream wavemap packages.

# For all targets
include_directories(include ${catkin_INCLUDE_DIRS})

# Libraries
add_library(${PROJECT_NAME} src/utils/eigen_format.cc)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Tests
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(
    test_${PROJECT_NAME}
    test/src/data_structure/test_aabb.cc
    test/src/data_structure/test_ndtree.cc
    test/src/data_structure/test_pointcloud.cc
      test/src/data_structure/test_volumetric_ndtree.cc
    test/src/data_structure/test_sparse_vector.cc
    test/src/data_structure/test_volumetric_cell_types.cc
    test/src/indexing/test_index_conversions.cc
    test/src/indexing/test_ndtree_index.cc
    test/src/iterator/test_subtree_iterator.cc
    test/src/utils/test_int_math.cc)
  target_include_directories(test_${PROJECT_NAME} PRIVATE test/include)
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} gtest_main)
endif ()

# Benchmarks
if (ENABLE_BENCHMARKING)
  add_executable(benchmark_sparse_vector benchmark/benchmark_sparse_vector.cc)
  target_link_libraries(benchmark_sparse_vector ${PROJECT_NAME}
                        benchmark::benchmark)
endif ()
