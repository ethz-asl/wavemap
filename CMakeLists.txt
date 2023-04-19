cmake_minimum_required(VERSION 3.0.2)
project(wavemap)

# NOTE: This CMakeLists file is not used by ROS and catkin, it only exists to
#       make it easy to load and edit all packages simultaneously in IDEs.

# Libraries
add_subdirectory(libraries/wavemap)

# ROS interfaces and tooling
add_subdirectory(ros/wavemap_msgs)
add_subdirectory(ros/wavemap_ros_conversions)
add_subdirectory(ros/wavemap_ros)
add_subdirectory(ros/wavemap_rviz_plugin)
