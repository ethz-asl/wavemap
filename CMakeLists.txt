cmake_minimum_required(VERSION 3.10)
project(wavemap_ide)

# NOTE: This CMakeLists file is not used by ROS and catkin, it only exists to
#       make it easy to load and edit all packages simultaneously in IDEs.

if ("$ENV{ROS_VERSION}" STREQUAL "1")
  # Load as a ROS1 project
  # NOTE: In this mode, introspection is available for
  #       wavemap's C++ library and ROS1 interfaces.

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
  # NOTE: In this mode, introspection is available for
  #       wavemap's C++ library and ROS2 interfaces.
  # TODO(victorr): Populate this once the ROS2 interface is under construction

else ()
  # Load in pure CMake mode
  # NOTE: In this mode, introspection is available
  #       only for wavemap's C++ library.
  add_subdirectory(library)
  add_subdirectory(examples/cpp)

endif ()
