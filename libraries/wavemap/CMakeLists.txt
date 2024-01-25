cmake_minimum_required(VERSION 3.0.2)
project(wavemap)

# Dependencies
find_package(catkin
    REQUIRED COMPONENTS glog_catkin eigen_catkin tracy_catkin minkindr)

if (ENABLE_BENCHMARKING)
  find_package(benchmark REQUIRED)
endif ()

# Compiler definitions and options
include(cmake/wavemap-extras.cmake)
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
    tracy_catkin
    minkindr
    CFG_EXTRAS
    wavemap-extras.cmake)
# NOTE: The test headers are normally not exported (e.g. by adding test/include
# to the catkin_package's INCLUDE_DIRS), but we make an exception for this base
# package's test headers, such that common tooling and test fixtures can be
# reused in downstream wavemap packages.

# For all targets
include_directories(include ${catkin_INCLUDE_DIRS})

# Libraries
# cmake-lint: disable=C0301
add_library(${PROJECT_NAME}
    src/config/value_with_unit.cc
    src/integrator/measurement_model/continuous_beam.cc
    src/integrator/measurement_model/constant_ray.cc
    src/integrator/measurement_model/continuous_ray.cc
    src/integrator/measurement_model/measurement_model_factory.cc
    src/integrator/projection_model/circular_projector.cc
    src/integrator/projection_model/ouster_projector.cc
    src/integrator/projection_model/pinhole_camera_projector.cc
    src/integrator/projection_model/spherical_projector.cc
    src/integrator/projection_model/projector_factory.cc
    src/integrator/projective/coarse_to_fine/coarse_to_fine_integrator.cc
    src/integrator/projective/coarse_to_fine/hashed_chunked_wavelet_integrator.cc
    src/integrator/projective/coarse_to_fine/hashed_wavelet_integrator.cc
    src/integrator/projective/coarse_to_fine/wavelet_integrator.cc
    src/integrator/projective/fixed_resolution/fixed_resolution_integrator.cc
    src/integrator/projective/projective_integrator.cc
    src/integrator/ray_tracing/ray_tracing_integrator.cc
    src/integrator/integrator_base.cc
    src/integrator/integrator_factory.cc
    src/map/hashed_blocks.cc
    src/map/hashed_chunked_wavelet_octree.cc
    src/map/hashed_chunked_wavelet_octree_block.cc
    src/map/hashed_wavelet_octree.cc
    src/map/hashed_wavelet_octree_block.cc
    src/map/volumetric_octree.cc
    src/map/wavelet_octree.cc
    src/map/map_base.cc
    src/map/map_factory.cc
    src/utils/query/classified_map.cc
    src/utils/query/collision_utils.cc
    src/utils/query/query_accelerator.cc
    src/utils/sdf/full_euclidean_sdf_generator.cc
    src/utils/sdf/quasi_euclidean_sdf_generator.cc
    src/utils/stopwatch.cc
    src/utils/thread_pool.cc)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Install targets
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

# Tests
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(
      test_${PROJECT_NAME}
      test/src/data_structure/test_aabb.cc
      test/src/data_structure/test_image.cc
      test/src/data_structure/test_ndtree.cc
      test/src/data_structure/test_pointcloud.cc
      test/src/data_structure/test_sparse_vector.cc
      test/src/indexing/test_index_conversions.cc
      test/src/indexing/test_ndtree_index.cc
      test/src/integrator/projection_model/test_circular_projector.cc
      test/src/integrator/projection_model/test_image_projectors.cc
      test/src/integrator/projection_model/test_spherical_projector.cc
      test/src/integrator/test_hierarchical_range_image.cc
      test/src/integrator/test_measurement_models.cc
      test/src/integrator/test_pointcloud_integrators.cc
      test/src/integrator/test_range_image_intersector.cc
      test/src/map/test_haar_cell.cc
      test/src/map/test_hashed_blocks.cc
      test/src/map/test_map.cc
      test/src/map/test_volumetric_octree.cc
      test/src/utils/bits/test_bit_operations.cc
      test/src/utils/data/test_comparisons.cc
      test/src/utils/data/test_fill.cc
      test/src/utils/iterate/test_grid_iterator.cc
      test/src/utils/iterate/test_ray_iterator.cc
      test/src/utils/iterate/test_subtree_iterator.cc
      test/src/utils/math/test_approximate_trigonometry.cc
      test/src/utils/math/test_int_math.cc
      test/src/utils/math/test_tree_math.cc
      test/src/utils/neighbors/test_adjacency.cc
      test/src/utils/neighbors/test_grid_adjacency.cc
      test/src/utils/neighbors/test_grid_neighborhood.cc
      test/src/utils/neighbors/test_ndtree_adjacency.cc
      test/src/utils/query/test_classified_map.cc
      test/src/utils/query/test_map_interpolator.cpp
      test/src/utils/query/test_occupancy_classifier.cc
      test/src/utils/query/test_probability_conversions.cc
      test/src/utils/query/test_query_accelerator.cc
      test/src/utils/sdf/test_sdf_generators.cc
      test/src/utils/test_thread_pool.cc)
  target_include_directories(test_${PROJECT_NAME} PRIVATE test/include)
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} gtest_main)
endif ()

# Benchmarks
if (ENABLE_BENCHMARKING)
  add_executable(benchmark_haar_transforms
      benchmark/benchmark_haar_transforms.cc)
  target_link_libraries(benchmark_haar_transforms ${PROJECT_NAME}
      benchmark::benchmark)

  add_executable(benchmark_sparse_vector benchmark/benchmark_sparse_vector.cc)
  target_link_libraries(benchmark_sparse_vector ${PROJECT_NAME}
      benchmark::benchmark)
endif ()
