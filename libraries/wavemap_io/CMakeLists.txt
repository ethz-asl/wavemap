cmake_minimum_required(VERSION 3.0.2)
project(wavemap_io)

find_package(catkin REQUIRED COMPONENTS wavemap wavemap_msgs wavemap_ros_conversions)
#catkin_simple(ALL_DEPS_REQUIRED)
# TODO(victorr): Switch to regular catkin

# Compiler definitions and options
add_wavemap_compile_definitions_and_options()

catkin_package(
    INCLUDE_DIRS
    include
    LIBRARIES
    ${PROJECT_NAME}
    CATKIN_DEPENDS
    wavemap
    wavemap_msgs
    wavemap_ros_conversions
)

# Libraries
# cmake-lint: disable=C0301
add_library(${PROJECT_NAME}
    src/file_conversions.cc
    src/stream_conversions.cc)
include_directories(include ${catkin_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Tests
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(
      test_${PROJECT_NAME}
      test/src/test_file_conversions.cc)
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} gtest_main)
endif ()

# Export
install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
# cs_export()
