cmake_minimum_required(VERSION 3.0.2)
project(wavemap_examples)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# Compiler definitions and options
add_wavemap_compile_definitions_and_options()

# Binaries
cs_add_executable(save_map_to_file src/io/save_map_to_file.cc)
cs_add_executable(load_map_from_file src/io/load_map_from_file.cc)
cs_add_executable(receive_map_over_ros src/io/receive_map_over_ros.cc)
cs_add_executable(send_map_over_ros src/io/send_map_over_ros.cc)
cs_add_executable(fixed_resolution src/queries/fixed_resolution.cc)
cs_add_executable(multi_resolution src/queries/multi_resolution.cc)
cs_add_executable(accelerated_queries src/queries/accelerated_queries.cc)
cs_add_executable(interpolation src/queries/interpolation.cc)
