cmake_minimum_required(VERSION 3.0.2)
project(wavemap_examples_ros1)

# Find dependencies
find_package(catkin REQUIRED
    COMPONENTS roscpp wavemap wavemap_msgs wavemap_ros_conversions)

# Register catkin package
catkin_package(
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp wavemap wavemap_msgs wavemap_ros_conversions)

# Add each set of examples
add_subdirectory(io)
