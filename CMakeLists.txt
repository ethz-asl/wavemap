cmake_minimum_required(VERSION 3.0.2)
project(wavemap)

# NOTE: This CMakeLists file is not used by ROS and catkin, it only exists to
#       make it easy to load and edit all packages simultaneously in IDEs.

# Include the catkin workspace's include dir, for msg and proto definitions
execute_process(COMMAND catkin locate --devel
    OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE CATKIN_WS_DEVEL_PATH)
include_directories(${CATKIN_WS_DEVEL_PATH}/include)

# Libraries
add_subdirectory(libraries/wavemap)
add_subdirectory(libraries/wavemap_file_conversions)

# ROS interfaces and tooling
add_subdirectory(ros/wavemap_msgs)
add_subdirectory(ros/wavemap_ros_conversions)
add_subdirectory(ros/wavemap_ros)
add_subdirectory(ros/wavemap_rviz_plugin)
