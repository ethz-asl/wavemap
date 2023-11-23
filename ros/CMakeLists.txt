cmake_minimum_required(VERSION 3.16...3.26)
project(wavemap_ros VERSION 0.3.0 LANGUAGES CXX)

set(ignore ${CATKIN_INSTALL_INTO_PREFIX_ROOT})
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../wavemap ${CMAKE_CURRENT_BINARY_DIR}/wavemap)

if("$ENV{ROS_VERSION}" STREQUAL "1")
  message(STATUS "wavemap ROS 1 wrapper will be compiled")
  find_package(
    catkin REQUIRED
    COMPONENTS 
      roscpp
      rosbag
      std_msgs)
  catkin_package()
  # ROS 1 node
  add_executable(wavemap_node ros1/src/wavemap_node.cpp)
  target_compile_features(wavemap_node PUBLIC cxx_std_20)
  target_include_directories(wavemap_node PUBLIC ros1/include ${catkin_INCLUDE_DIRS})
  target_link_libraries(wavemap_node wavemap::core ${catkin_LIBRARIES})
  install(TARGETS wavemap_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

elseif("$ENV{ROS_VERSION}" STREQUAL "2")
  message(STATUS "wavemap ROS 2 wrapper will be compiled")
  find_package(ament_cmake REQUIRED)
  find_package(rcutils REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(rclcpp_components REQUIRED)

  # ROS 2 node
  add_library(wavemap_component SHARED ros2/src/wavemap_node.cpp)
  target_compile_features(wavemap_component PUBLIC cxx_std_20)
  target_include_directories(wavemap_component PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/ros2/include)
  target_link_libraries(wavemap_component wavemap::core)
  ament_target_dependencies(
    wavemap_component
    rcutils
    rclcpp
    rclcpp_components)

  rclcpp_components_register_node(wavemap_component PLUGIN "wavemap_ros::MyCoreClass" EXECUTABLE wavemap_node)

  install(TARGETS wavemap_component LIBRARY DESTINATION lib RUNTIME DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME}/)

  ament_package()

else()
  message(FATAL_ERROR "catkin or colcon not found wavemap_ros disabled")
endif()