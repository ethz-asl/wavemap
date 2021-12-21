cmake_minimum_required(VERSION 3.0.2)
project(wavemap_2d)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
if (ENABLE_BENCHMARKING)
  find_package(benchmark REQUIRED)
endif ()
if (CATKIN_ENABLE_TESTING AND ENABLE_COVERAGE_TESTING)
  find_package(code_coverage REQUIRED)
  APPEND_COVERAGE_COMPILER_FLAGS()
endif ()

add_definitions(-std=c++17 -Wall -Wextra -Wno-deprecated-copy -fPIC -DEIGEN_INITIALIZE_MATRICES_BY_NAN)

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
if (CATKIN_ENABLE_TESTING)
  cs_add_library(testing_main test/gtest_main.cc)

  catkin_add_gtest(test_cell_traits test/src/datastructure/test_cell_traits.cc)
  target_include_directories(test_cell_traits PRIVATE test/include)
  target_link_libraries(test_cell_traits ${PROJECT_NAME} testing_main)

  catkin_add_gtest(test_dense_grid test/src/datastructure/test_dense_grid.cc)
  target_include_directories(test_dense_grid PRIVATE test/include)
  target_link_libraries(test_dense_grid ${PROJECT_NAME} testing_main)

  catkin_add_gtest(test_pointcloud test/src/datastructure/test_pointcloud.cc)
  target_include_directories(test_pointcloud PRIVATE test/include)
  target_link_libraries(test_pointcloud ${PROJECT_NAME} testing_main)

  catkin_add_gtest(test_quadtree test/src/datastructure/test_quadtree.cc)
  target_include_directories(test_quadtree PRIVATE test/include)
  target_link_libraries(test_quadtree ${PROJECT_NAME} testing_main)

  catkin_add_gtest(test_iterators test/src/integrator/test_iterators.cc)
  target_include_directories(test_iterators PRIVATE test/include)
  target_link_libraries(test_iterators ${PROJECT_NAME} testing_main)

  catkin_add_gtest(test_measurement_models test/src/integrator/test_measurement_models.cc)
  target_include_directories(test_measurement_models PRIVATE test/include)
  target_link_libraries(test_measurement_models ${PROJECT_NAME} testing_main)

  catkin_add_gtest(test_constexpr_functions test/src/utils/test_constexpr_functions.cc)
  target_include_directories(test_constexpr_functions PRIVATE test/include)
  target_link_libraries(test_constexpr_functions ${PROJECT_NAME} testing_main)

  if (ENABLE_COVERAGE_TESTING)
    set(COVERAGE_EXCLUDES "*/${PROJECT_NAME}/test/*" "*/${PROJECT_NAME}/app/*" "*/${PROJECT_NAME}/benchmark/*")
    add_code_coverage(
        NAME ${PROJECT_NAME}_coverage_report
        DEPENDENCIES tests)
  endif ()
endif ()

##############
# BENCHMARKS #
##############
if (ENABLE_BENCHMARKING)
  cs_add_library(benchmarking_main benchmark/google_benchmark_main.cc)
  target_link_libraries(benchmarking_main benchmark::benchmark)

  add_executable(benchmark_measurement_integrator benchmark/benchmark_measurement_integrator.cc)
  target_link_libraries(benchmark_measurement_integrator ${PROJECT_NAME} benchmarking_main)
endif ()

##########
# EXPORT #
##########
cs_install()
cs_export()
