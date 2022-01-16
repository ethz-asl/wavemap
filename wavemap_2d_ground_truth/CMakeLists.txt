cmake_minimum_required(VERSION 3.0.2)
project(wavemap_2d_ground_truth)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wno-deprecated-copy -Wno-class-memaccess")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -DEIGEN_INITIALIZE_MATRICES_BY_NAN")

if (USE_CLANG_TIDY)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif()

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
    src/occupancy_grid_creator.cc
    src/geometry.cc)

##############
## BINARIES ##
##############
cs_add_executable(ply_converter
    src/user_interfaces/ply_converter.cc)
target_link_libraries(ply_converter ${PROJECT_NAME})

#################
# GAZEBO PLUGIN #
#################
set(BUILD_GAZEBO_PLUGINS TRUE)

if (BUILD_GAZEBO_PLUGINS)
  include_directories(${GAZEBO_INCLUDE_DIRS})
  cs_add_library(${PROJECT_NAME}_plugin
      src/user_interfaces/gazebo_plugin.cc
      src/geometry.cc
      src/occupancy_grid_creator.cc)
endif ()

##########
# EXPORT #
##########
cs_install()
cs_export()
