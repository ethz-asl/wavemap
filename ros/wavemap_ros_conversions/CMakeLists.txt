cmake_minimum_required(VERSION 3.0.2)
project(wavemap_ros_conversions)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# Compiler definitions and options
add_wavemap_compile_definitions_and_options()

# Libraries
cs_add_library(${PROJECT_NAME}
    src/config_conversions.cc
    src/map_msg_conversions.cc
    src/time_conversions.cc)

# Tests
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(
      test_${PROJECT_NAME}
      test/src/test_map_msg_conversions.cc)
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} gtest_main)
endif ()

# Export
cs_install()
cs_export()
