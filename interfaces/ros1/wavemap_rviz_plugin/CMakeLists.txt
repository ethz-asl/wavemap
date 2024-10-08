# NOTE: This file is based on the Rviz plugin tutorials:
#       https://github.com/ros-visualization/visualization_tutorials/blob/
#       noetic-devel/rviz_plugin_tutorials/CMakeLists.txt

cmake_minimum_required(VERSION 3.0.2)
project(wavemap_rviz_plugin)

# Dependencies
find_package(catkin REQUIRED
    COMPONENTS rviz wavemap wavemap_msgs wavemap_ros_conversions)

# Setup catkin package
catkin_package(CATKIN_DEPENDS
    wavemap wavemap_msgs wavemap_ros_conversions)

# For all targets
include_directories(include SYSTEM ${catkin_INCLUDE_DIRS})

# This setting causes Qt's "MOC" generation to happen automatically
set(CMAKE_AUTOMOC ON)

# This plugin includes Qt widgets, so we must include Qt.
# We'll use the version that rviz used so they are compatible.
set(HEADERS_TO_MOC
    include/wavemap_rviz_plugin/wavemap_map_display.h
    include/wavemap_rviz_plugin/visuals/voxel_visual.h
    include/wavemap_rviz_plugin/visuals/slice_visual.h
    include/wavemap_rviz_plugin/visuals/cell_selector.h
    include/wavemap_rviz_plugin/utils/button_property.h)
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

# Libraries
add_library(${PROJECT_NAME}
    src/utils/color_conversions.cc
    src/utils/listeners.cc
    src/visuals/slice_visual.cc
    src/visuals/voxel_visual.cc
    src/visuals/cell_layer.cc
    src/visuals/cell_selector.cc
    src/wavemap_map_display.cc
    ${QT_MOC})
set_wavemap_target_properties(${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} PUBLIC
    ${QT_LIBRARIES} ${catkin_LIBRARIES})

# Avoid Qt signals and slots defining "emit", "slots", etc.
# because they can conflict with boost signals.
target_compile_definitions(${PROJECT_NAME} PUBLIC -DQT_NO_KEYWORDS)

# Disable some warnings coming from Qt code in combination with C++17
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-pedantic -Wno-register)

# Support installs
# Mark targets for installation
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
# Mark plugin description for installation
install(FILES plugin_description.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
# Mark icons for installation
install(DIRECTORY icons/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)
