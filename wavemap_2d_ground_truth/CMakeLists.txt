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
    src/occupancy_grid_creator.cpp
    src/geometry.cpp)

##############
## BINARIES ##
##############
cs_add_executable(ply_to_occupancy_grid
    src/user_interfaces/ply_to_occupancy_script.cpp)
target_link_libraries(ply_to_occupancy_grid ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export()
