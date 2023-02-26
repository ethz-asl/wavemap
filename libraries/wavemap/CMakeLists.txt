cmake_minimum_required(VERSION 3.0.2)
project(wavemap)

# Dependencies
find_package(catkin
    REQUIRED COMPONENTS glog_catkin eigen_catkin minkindr)
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
add_library(${PROJECT_NAME}
    src/data_structure/volumetric/volumetric_data_structure_base.cc
    src/data_structure/volumetric/volumetric_data_structure_factory.cc
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
    src/integrator/projective/coarse_to_fine/wavelet_integrator.cc
    src/integrator/projective/fixed_resolution/fixed_resolution_integrator.cc
    src/integrator/projective/projective_integrator.cc
    src/integrator/ray_tracing/ray_tracing_integrator.cc
    src/integrator/integrator_base.cc
    src/integrator/integrator_factory.cc
    src/utils/eigen_format.cc)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Tests
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(
      test_${PROJECT_NAME}
      test/src/data_structure/test_aabb.cc
      test/src/data_structure/test_haar_cell.cc
      test/src/data_structure/test_hashed_blocks.cc
      test/src/data_structure/test_image.cc
      test/src/data_structure/test_ndtree.cc
      test/src/data_structure/test_pointcloud.cc
      test/src/data_structure/test_volumetric_data_structure.cc
      test/src/data_structure/test_volumetric_octree.cc
      test/src/data_structure/test_sparse_vector.cc
      test/src/data_structure/test_volumetric_cell_types.cc
      test/src/indexing/test_index_conversions.cc
      test/src/indexing/test_ndtree_index.cc
      test/src/integrator/projection_model/test_circular_projector.cc
      test/src/integrator/projection_model/test_image_projectors.cc
      test/src/integrator/projection_model/test_spherical_projector.cc
      test/src/integrator/test_hierarchical_range_image.cc
      test/src/integrator/test_measurement_models.cc
      test/src/integrator/test_pointcloud_integrators.cc
      test/src/integrator/test_range_image_intersector.cc
      test/src/iterator/test_grid_iterator.cc
      test/src/iterator/test_ray_iterator.cc
      test/src/iterator/test_subtree_iterator.cc
      test/src/utils/test_approximate_trigonometry.cc
      test/src/utils/test_bit_manipulation.cc
      test/src/utils/test_fill_utils.cc
      test/src/utils/test_int_math.cc)
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
