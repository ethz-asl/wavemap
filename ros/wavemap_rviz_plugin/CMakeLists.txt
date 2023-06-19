# NOTE: This file is based on the Rviz plugin tutorials:
#       https://github.com/ros-visualization/visualization_tutorials/blob/
#       noetic-devel/rviz_plugin_tutorials/CMakeLists.txt

cmake_minimum_required(VERSION 3.0.2)
project(wavemap_rviz_plugin)

# Dependencies
find_package(catkin
    REQUIRED COMPONENTS rviz wavemap wavemap_msgs wavemap_ros_conversions)

# Compiler definitions and options
add_wavemap_compile_definitions_and_options()
# Disable some warnings coming from Qt code in combination with C++17
add_compile_options(-Wno-pedantic -Wno-register)

# Setup catkin package
catkin_package(CATKIN_DEPENDS wavemap wavemap_msgs wavemap_ros_conversions)

# For all targets
include_directories(include ${catkin_INCLUDE_DIRS})

# This setting causes Qt's "MOC" generation to happen automatically
set(CMAKE_AUTOMOC ON)

# This plugin includes Qt widgets, so we must include Qt.
# We'll use the version that rviz used so they are compatible.
set(HEADERS_TO_MOC
    include/wavemap_rviz_plugin/wavemap_map_display.h
    include/wavemap_rviz_plugin/visuals/grid_visual.h
    include/wavemap_rviz_plugin/visuals/slice_visual.h)
if (rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  # Pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
  # Generate QT's MOC files for our project's headers
  qt4_wrap_cpp(QT_MOC ${HEADERS_TO_MOC})
else ()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  # Make target_link_libraries(${QT_LIBRARIES}) pull in
  # all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
  # Generate QT's MOC files for our project's headers
  QT5_WRAP_CPP(QT_MOC ${HEADERS_TO_MOC})
endif ()

# Avoid Qt signals and slots defining "emit", "slots", etc.
# because they can conflict with boost signals.
add_definitions(-DQT_NO_KEYWORDS)

# Libraries
add_library(${PROJECT_NAME}
    src/visuals/grid_visual.cc
    src/visuals/slice_visual.cc
    src/wavemap_map_display.cc
    ${QT_MOC})
target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})

# Install
install(TARGETS
    ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(FILES
    plugin_description.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY icons/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)
