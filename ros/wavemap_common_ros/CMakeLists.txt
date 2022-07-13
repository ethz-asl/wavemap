cmake_minimum_required(VERSION 3.0.2)
project(wavemap_common_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# Compiler definitions and options
add_wavemap_compile_definitions_and_options()

# Libraries
cs_add_library(${PROJECT_NAME} src/rosbag_processor.cc src/tf_transformer.cc)

# Export
cs_install()
cs_export()
