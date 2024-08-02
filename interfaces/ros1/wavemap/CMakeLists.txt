cmake_minimum_required(VERSION 3.0.2)
project(wavemap)

# Setup catkin package
find_package(catkin REQUIRED)
catkin_package(
    INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../../../library/include
    LIBRARIES wavemap_core wavemap_io wavemap_pipeline
    CFG_EXTRAS
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../library/cmake/wavemap-extras.cmake)

# Load the library
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../library
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}
    EXCLUDE_FROM_ALL)

# Include the library's tests in catkin's run_tests target
include(cmake/append_to_catkin_tests.cmake)
append_to_catkin_tests(test_wavemap_io)
append_to_catkin_tests(test_wavemap_core)

# Install targets
install(TARGETS wavemap_core wavemap_io wavemap_pipeline
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Mark cpp header files for installation
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../../library/include/wavemap/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h")
