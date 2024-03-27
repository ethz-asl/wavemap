cmake_minimum_required(VERSION 3.0.2)
project(wavemap_io)

# Dependencies
find_package(catkin REQUIRED COMPONENTS wavemap)

# Compiler definitions and options
add_wavemap_compile_definitions_and_options()

# Setup catkin package
catkin_package(INCLUDE_DIRS include LIBRARIES ${PROJECT_NAME} CATKIN_DEPENDS)

# For all targets
include_directories(include ${catkin_INCLUDE_DIRS})

# Libraries
# cmake-lint: disable=C0301
add_library(${PROJECT_NAME}
    src/file_conversions.cc
    src/stream_conversions.cc)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Install targets
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h"
)

# Tests
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(
      test_${PROJECT_NAME}
      test/src/test_file_conversions.cc)
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} gtest_main)
endif ()
