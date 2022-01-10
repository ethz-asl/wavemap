cmake_minimum_required(VERSION 2.8.3)
project(wavemap_2d_ground_truth)

add_definitions(-std=c++17)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

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
