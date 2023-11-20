cmake_minimum_required(VERSION 3.0.2)
project(wavemap_ros)

add_definitions(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
roscpp
wavemap
wavemap_io
wavemap_msgs
wavemap_ros_conversions
rosbag
cv_bridge
image_transport
tf2
tf2_ros
tf2_geometry_msgs
sensor_msgs
visualization_msgs
std_srvs
eigen_conversions
)
# catkin_simple(ALL_DEPS_REQUIRED)

# Find OpenCV explicitly to avoid issues on Jetson
find_package(OpenCV REQUIRED)
find_package(gflags REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PkgConfig REQUIRED)
pkg_check_modules(Eigen3 REQUIRED eigen3)

# Optional dependencies
find_package(livox_ros_driver2 QUIET)
if (livox_ros_driver2_FOUND)
  include_directories(${livox_ros_driver2_INCLUDE_DIRS})
  add_compile_definitions(LIVOX_AVAILABLE)
endif ()

# Compiler definitions and options
add_wavemap_compile_definitions_and_options()

include(FetchContent)
FetchContent_Declare(
    tracy
    GIT_REPOSITORY https://github.com/wolfpld/tracy.git
    GIT_TAG v0.10
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(tracy)

catkin_package(
    INCLUDE_DIRS
    include
    LIBRARIES
    ${PROJECT_NAME}
    CATKIN_DEPENDS
    roscpp
    wavemap
    wavemap_io
    wavemap_msgs
    wavemap_ros_conversions
    rosbag
    cv_bridge
    image_transport
    tf2
    tf2_ros
    tf2_geometry_msgs
    sensor_msgs
    visualization_msgs
    std_srvs
    eigen_conversions
)
include_directories(include ${catkin_INCLUDE_DIRS})

# Libraries
add_library(${PROJECT_NAME}
    src/rosbag_processor.cc
    src/tf_transformer.cc
    src/input_handler/depth_image_input_handler.cc
    src/input_handler/input_handler.cc
    src/input_handler/input_handler_factory.cc
    src/input_handler/pointcloud_input_handler.cc
    src/input_handler/pointcloud_undistorter.cc
    src/wavemap_server.cc)
# Link OpenCV explicitly to avoid issues on Jetson
target_link_libraries(${PROJECT_NAME} PUBLIC ${OpenCV_LIBRARIES} TracyClient)

# Binaries
add_executable(wavemap_server_node app/server_node.cc)
target_link_libraries(wavemap_server_node PUBLIC ${PROJECT_NAME} ${catkin_LIBRARIES} ${EIGEN3_INCLUDE_DIRS} TracyClient gflags)

# add_executable(wavemap_rosbag_processor app/rosbag_processor.cc)
# target_link_libraries(wavemap_rosbag_processor PUBLIC ${PROJECT_NAME} TracyClient gflags)

# Export
# install()
# cs_export()

# Export config files
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
