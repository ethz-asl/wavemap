cmake_minimum_required(VERSION 3.0.2)
project(wavemap_io)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)
# TODO(victorr): Switch to regular catkin

# Compiler definitions and options
add_wavemap_compile_definitions_and_options()

# Libraries
# cmake-lint: disable=C0301
cs_add_library(${PROJECT_NAME}
    src/file_conversions.cc
    src/stream_conversions.cc)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Tests
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(
      test_${PROJECT_NAME}
      test/src/test_file_conversions.cc)
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} gtest_main)
endif ()

# Export
cs_install()
cs_export()
