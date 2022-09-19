cmake_minimum_required(VERSION 3.0.2)
project(wavemap_3d)

# Dependencies
find_package(catkin REQUIRED COMPONENTS wavemap_common)

# Compiler definitions and options
add_wavemap_compile_definitions_and_options()

# Setup catkin package
catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} CATKIN_DEPENDS
    wavemap_common)

# For all targets
include_directories(include ${catkin_INCLUDE_DIRS})

# Libraries
add_library(${PROJECT_NAME}
    src/data_structure/volumetric_data_structure_3d_factory.cc
    src/integrator/projective/coarse_to_fine/coarse_to_fine_integrator_3d.cc
    src/integrator/projective/coarse_to_fine/wavelet_integrator_3d.cc
    src/integrator/projective/fixed_resolution/fixed_resolution_integrator_3d.cc
    src/integrator/projective/range_image_2d.cc
    src/integrator/pointcloud_integrator_factory_3d.cc)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Tests
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(
      test_${PROJECT_NAME}
      test/src/data_structure/test_volumetric_data_structure_3d.cc
      test/src/integrator/test_hierarchical_range_image_2d.cc
      test/src/integrator/test_pointcloud_integrators_3d.cc
      test/src/integrator/test_range_image_2d.cc
      test/src/integrator/test_range_image_2d_intersector.cc)
  target_include_directories(test_${PROJECT_NAME} PRIVATE test/include)
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} gtest_main)
endif ()
