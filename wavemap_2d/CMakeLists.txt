cmake_minimum_required(VERSION 3.0.2)
project(wavemap_2d)

find_package(catkin_simple REQUIRED)
if (ENABLE_BENCHMARKING)
  find_package(benchmark REQUIRED)
endif ()
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17 -Wall -Wextra -Wno-deprecated-copy -fPIC -DEIGEN_INITIALIZE_MATRICES_BY_NAN)

############
# PROTOBUF #
############
#set(PROTO_DEFNS proto/wavemap/GridMap.proto)
#set(BASE_PATH "proto")
##set(PROTOBUF_COMPILE_PYTHON true)

#PROTOBUF_CATKIN_GENERATE_CPP2(${BASE_PATH} PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})

#############
# LIBRARIES #
#############
#cs_add_library(${PROJECT_NAME}_proto
#    ${PROTO_SRCS})

cs_add_library(${PROJECT_NAME}
    src/integrator/pointcloud_integrator.cc
    src/integrator/beam_model.cc
    src/integrator/fixed_logodds_model.cc
    src/datastructure/datastructure_base.cc)
target_link_libraries(${PROJECT_NAME}) # ${PROJECT_NAME}_proto)

############
# BINARIES #
############
add_executable(carmen_processor
    app/carmen_processor.cc)
target_link_libraries(carmen_processor ${PROJECT_NAME})

#########
# TESTS #
#########
# TODO: Check what the best way is to build and run the tests without the clutter of
#       catkin run_tests --no-deps --this # or
#       catkin build wavemap_2d --no-deps --verbose --catkin-make-args run_tests
if (CATKIN_ENABLE_TESTING)
  cs_add_library(testing_main test/gtest_main.cc)

  catkin_add_gtest(test_map
      test/test_map.cc)
  target_link_libraries(test_map ${PROJECT_NAME} testing_main)

  catkin_add_gtest(test_integrator
      test/test_integrator.cc)
  target_link_libraries(test_integrator ${PROJECT_NAME} testing_main)

  catkin_add_gtest(test_pointcloud
      test/test_pointcloud.cc)
  target_link_libraries(test_pointcloud ${PROJECT_NAME} testing_main)
endif ()

##############
# BENCHMARKS #
##############
if (ENABLE_BENCHMARKING)
  cs_add_library(benchmarking_main benchmark/google_benchmark_main.cc)
  target_link_libraries(benchmarking_main benchmark::benchmark)

  add_executable(benchmark_measurement_integrator
      benchmark/benchmark_measurement_integrator.cc)
  target_link_libraries(benchmark_measurement_integrator ${PROJECT_NAME} benchmarking_main)
endif ()

##########
# EXPORT #
##########
cs_install()
cs_export()
