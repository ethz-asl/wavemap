cmake_minimum_required(VERSION 3.0.2)
project(wavemap_common)

find_package(catkin REQUIRED COMPONENTS glog_catkin eigen_catkin minkindr)

# Configure
include(cmake/wavemap_common-extras.cmake)
add_wavemap_compile_definitions_and_options()

# Setup catkin
catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_NAME}
  CATKIN_DEPENDS
  glog_catkin
  eigen_catkin
  minkindr
  CFG_EXTRAS
  wavemap_common-extras.cmake)
include_directories(include ${catkin_INCLUDE_DIRS})

# Libraries
add_library(${PROJECT_NAME} src/utils/eigen_format.cc)

# Tests
if (CATKIN_ENABLE_TESTING)
  add_library(wavemap_testing_main test/gtest_main.cc)
  target_include_directories(wavemap_testing_main PUBLIC test/include)
  target_link_libraries(wavemap_testing_main ${glog_catkin_LIBRARIES})

  catkin_add_gtest(
    test_${PROJECT_NAME}
    test/src/data_structure/test_aabb.cc
    test/src/data_structure/test_ndtree.cc
    test/src/data_structure/test_pointcloud.cc
    test/src/data_structure/test_sparse_vector.cc
    test/src/indexing/test_ndtree_index.cc
    test/src/iterator/test_subtree_iterator.cc
    test/src/utils/test_int_math.cc)
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME}
                        wavemap_testing_main)
endif ()

# Benchmarks
if (ENABLE_BENCHMARKING)
  add_library(wavemap_benchmarking_main benchmark/google_benchmark_main.cc)
  target_link_libraries(wavemap_benchmarking_main ${glog_catkin_LIBRARIES}
                        benchmark::benchmark)

  add_executable(benchmark_sparse_vector benchmark/benchmark_sparse_vector.cc)
  target_link_libraries(benchmark_sparse_vector ${PROJECT_NAME}
                        wavemap_benchmarking_main)
endif ()
