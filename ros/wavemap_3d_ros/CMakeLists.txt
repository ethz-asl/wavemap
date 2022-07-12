cmake_minimum_required(VERSION 3.0.2)
project(wavemap_3d_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# Compiler definitions and options
add_wavemap_compile_definitions_and_options()

# Libraries cs_add_library(${PROJECT_NAME} src/some_file.cc)

# Binaries add_executable(wavemap_3d_server_node app/wavemap_3d_server_node.cc)
# target_link_libraries(wavemap_3d_server_node ${PROJECT_NAME})

# cs_add_executable(rosbag_processor app/rosbag_processor.cc)
# target_link_libraries(rosbag_processor ${PROJECT_NAME})

# Export
cs_install()
cs_export()
