cmake_minimum_required(VERSION 3.0.2)
project(wavemap_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# Compiler definitions and options
add_wavemap_compile_definitions_and_options()

# Libraries
cs_add_library(${PROJECT_NAME}
    src/rosbag_processor.cc
    src/tf_transformer.cc
    src/utils/config_conversions.cc
    src/input_handler/depth_image_input_handler.cc
    src/input_handler/input_handler.cc
    src/input_handler/input_handler_factory.cc
    src/input_handler/pointcloud_input_handler.cc
    src/wavemap_server.cc)

# Binaries
add_executable(wavemap_server_node app/server_node.cc)
target_link_libraries(wavemap_server_node ${PROJECT_NAME})

cs_add_executable(wavemap_rosbag_processor app/rosbag_processor.cc)
target_link_libraries(wavemap_rosbag_processor ${PROJECT_NAME})

# Export
cs_install()
cs_export()
