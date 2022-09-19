cmake_minimum_required(VERSION 3.0.2)
project(wavemap_2d)

# Dependencies
find_package(catkin REQUIRED COMPONENTS wavemap_common opencv3_catkin)
if (ENABLE_BENCHMARKING)
  find_package(benchmark REQUIRED)
endif ()

# Compiler definitions and options
add_wavemap_compile_definitions_and_options()

# Setup catkin package
catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} CATKIN_DEPENDS
               wavemap_common opencv3_catkin)

# For all targets
include_directories(include ${catkin_INCLUDE_DIRS})

# Libraries
add_library(
  ${PROJECT_NAME}
    src/data_structure/volumetric_data_structure_2d_factory.cc
    src/integrator/pointcloud_integrator_factory_2d.cc
    src/integrator/projective/coarse_to_fine/coarse_to_fine_integrator_2d.cc
    src/integrator/projective/coarse_to_fine/wavelet_integrator_2d.cc
    src/integrator/projective/range_image_1d.cc
    src/integrator/projective/fixed_resolution/fixed_resolution_integrator_2d.cc
  src/utils/evaluation_utils.cc)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Binaries
add_executable(carmen_processor app/carmen_processor.cc)
target_link_libraries(carmen_processor ${PROJECT_NAME})

add_executable(offline_compression_evaluation
               app/offline_compression_evaluation.cc)
target_link_libraries(offline_compression_evaluation ${PROJECT_NAME})

# Tests
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(
    test_${PROJECT_NAME}
    test/src/data_structure/test_dense_grid.cc
    test/src/data_structure/test_volumetric_data_structure.cc
      test/src/integrator/test_range_image_1d.cc
    test/src/integrator/test_pointcloud_integrators.cc
      test/src/integrator/test_range_image_1d_intersector.cc
      test/src/integrator/test_hierarchical_range_image_1d.cc
    test/src/integrator/test_measurement_models.cc
    test/src/transform/test_discrete_wavelet_transform.cc
    test/src/transform/test_naive_haar.cc)
  target_include_directories(test_${PROJECT_NAME} PRIVATE test/include)
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} gtest_main)
endif ()

# Benchmarks
if (ENABLE_BENCHMARKING)
  add_executable(benchmark_measurement_integrator
                 benchmark/benchmark_measurement_integrator.cc)
  target_link_libraries(benchmark_measurement_integrator ${PROJECT_NAME}
                        benchmark::benchmark)
endif ()
