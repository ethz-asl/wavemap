cmake_minimum_required(VERSION 3.0.2)
project(wavemap)

# Find dependencies
find_package(catkin REQUIRED)
find_package(Eigen3 QUIET NO_MODULE)
# Manually set the lib name for glog and remap the include dir var for Eigen3
# cmake-lint: disable=C0103
set(glog_LIBRARIES glog)
set(Eigen3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
# NOTE: These are workarounds to handle wavemap's dependencies in catkin despite
#       glog not yet supporting CMake's find_package on Ubuntu 20.04 and Eigen3
#       naming its variables differently from what catkin expects.

# Register catkin package
catkin_package(
    INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../../../library/cpp/include
    LIBRARIES wavemap_core wavemap_io wavemap_pipeline
    DEPENDS Eigen3 glog
    CFG_EXTRAS
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../library/cpp/cmake/wavemap-extras.cmake)

# Load the library
set(GENERATE_WAVEMAP_INSTALL_RULES OFF)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../library/cpp
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME})

# Include the library's tests in catkin's run_tests target
include(cmake/append_to_catkin_tests.cmake)
append_to_catkin_tests(test_wavemap_io)
append_to_catkin_tests(test_wavemap_core)

# Install targets
install(TARGETS wavemap_core wavemap_io wavemap_pipeline
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Mark header files for installation
install(DIRECTORY
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../library/cpp/include/wavemap/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h")
