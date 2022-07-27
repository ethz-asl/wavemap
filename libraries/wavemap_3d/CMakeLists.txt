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
    src/integrator/measurement_model/fixed_logodds_model.cc
    src/integrator/measurement_model/measurement_model.cc
    src/integrator/pointcloud_integrator.cc)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
