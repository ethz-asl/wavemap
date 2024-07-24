cmake_minimum_required(VERSION 3.0.2)
project(wavemap_ros_conversions)

# Find dependencies
find_package(catkin REQUIRED
    COMPONENTS roscpp eigen_conversions wavemap wavemap_msgs)

# Register catkin package
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS roscpp eigen_conversions wavemap wavemap_msgs)

# Libraries
add_library(${PROJECT_NAME}
    src/config_conversions.cc
    src/map_msg_conversions.cc
    src/time_conversions.cc)
set_wavemap_target_properties(${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}
    PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})

# Tests
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(
      test_${PROJECT_NAME}
      test/src/test_map_msg_conversions.cc)
  set_wavemap_target_properties(test_${PROJECT_NAME})
  target_include_directories(test_${PROJECT_NAME} PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/../../../library/test/include)
  target_link_libraries(test_${PROJECT_NAME}
      ${PROJECT_NAME} glog GTest::gtest_main)
endif ()
