cmake_minimum_required(VERSION 3.10)
project(wavemap_ide)

# NOTE: This CMakeLists file is not used by ROS and catkin, it only exists to
#       make it easy to load and edit all packages simultaneously in IDEs.

# NOTE: When working with ROS, make sure to open your IDE from a terminal in
#       which your ROS workspace has been sourced. For example, for ROS1 and
#       clion, run: ``source ~/catkin_ws/devel/setup.bash && clion``.

if ("$ENV{ROS_VERSION}" STREQUAL "1")
  # Load as a ROS1 project
  # In this mode, introspection is available for the C++ lib and ROS1 interface

  # Include the catkin workspace's include dir, e.g. for ros msg definitions
  execute_process(COMMAND catkin locate --devel
      OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE CATKIN_WS_DEVEL_PATH)
  include_directories(${CATKIN_WS_DEVEL_PATH}/include)

  # ROS interfaces and tooling
  # NOTE: Wavemap's C++ library gets included through interfaces/ros1/wavemap.
  add_subdirectory(interfaces/ros1/wavemap)
  add_subdirectory(interfaces/ros1/wavemap_msgs)
  add_subdirectory(interfaces/ros1/wavemap_ros_conversions)
  add_subdirectory(interfaces/ros1/wavemap_ros)
  add_subdirectory(interfaces/ros1/wavemap_rviz_plugin)

  # Usage examples
  add_subdirectory(examples/cpp)
  add_subdirectory(examples/ros1)

elseif ("$ENV{ROS_VERSION}" STREQUAL "2")
  # Load as a ROS2 project
  # In this mode, introspection is available for the C++ lib and ROS2 interface
  # TODO(victorr): Populate this once the ROS2 interface is under construction

else ()
  # Load in pure CMake mode
  # In this mode, introspection is available only for the C++ library
  add_subdirectory(library/cpp)
  add_subdirectory(examples/cpp)

endif ()
