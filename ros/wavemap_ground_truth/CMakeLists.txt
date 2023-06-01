cmake_minimum_required(VERSION 3.0.2)
project(wavemap_ground_truth)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# Compiler definitions and options
add_wavemap_compile_definitions_and_options()

# Libraries
cs_add_library(${PROJECT_NAME} src/occupancy_grid_creator.cc src/geometry.cc)

# Binaries
cs_add_executable(ply_converter src/user_interfaces/ply_converter.cc)
target_link_libraries(ply_converter ${PROJECT_NAME})

# Gazebo plugin
set(BUILD_GAZEBO_PLUGINS TRUE)
if (BUILD_GAZEBO_PLUGINS)
  include_directories(${GAZEBO_INCLUDE_DIRS})
  cs_add_library(${PROJECT_NAME}_plugin src/user_interfaces/gazebo_plugin.cc
                 src/geometry.cc src/occupancy_grid_creator.cc)
endif ()

# Export
cs_install()
cs_export()
