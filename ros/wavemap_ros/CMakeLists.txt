cmake_minimum_required(VERSION 3.0.2)
project(wavemap_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# Find OpenCV explicitly to avoid issues on Jetson
find_package(OpenCV REQUIRED)

# Optional dependencies
set(OPTIONAL_SRCS "")
find_package(livox_ros_driver2)
if (livox_ros_driver2_FOUND)
  include_directories(${livox_ros_driver2_INCLUDE_DIRS})
  list(APPEND OPTIONAL_SRCS src/input_handler/livox_input_handler.cc)
  add_compile_definitions(LIVOX_AVAILABLE)
endif ()

# Compiler definitions and options
add_wavemap_compile_definitions_and_options()

# Libraries
cs_add_library(${PROJECT_NAME}
    src/rosbag_processor.cc
    src/tf_transformer.cc
    src/input_handler/depth_image_input_handler.cc
    src/input_handler/input_handler.cc
    src/input_handler/input_handler_factory.cc
    src/input_handler/pointcloud_input_handler.cc
    src/wavemap_server.cc
    ${OPTIONAL_SRCS})
# Link OpenCV explicitly to avoid issues on Jetson
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})

# Binaries
cs_add_executable(wavemap_server_node app/server_node.cc)
target_link_libraries(wavemap_server_node ${PROJECT_NAME})

cs_add_executable(wavemap_rosbag_processor app/rosbag_processor.cc)
target_link_libraries(wavemap_rosbag_processor ${PROJECT_NAME})

# Export
cs_install()
cs_export()
