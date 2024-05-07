cmake_minimum_required(VERSION 3.0.2)
project(wavemap)

# Setup catkin package
find_package(catkin REQUIRED)
catkin_package(
    INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../library/include
    LIBRARIES wavemap_core wavemap_io
    CFG_EXTRAS
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../library/cmake/wavemap-extras.cmake
)

# Load the library
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../library
    ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME})

# Install targets
install(TARGETS wavemap_core wavemap_io
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Mark cpp header files for installation
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../../../library/include
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h")
