cmake_minimum_required(VERSION 3.0.2)
project(wavemap_examples)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# Compiler definitions and options
add_compile_options(-Wno-suggest-attribute=const)

# For all targets
include_directories(include)

# Binaries
cs_add_executable(save_map_to_file
        src/io/save_map_to_file.cc)
set_wavemap_target_properties(save_map_to_file)
target_link_libraries(save_map_to_file glog)
cs_add_executable(load_map_from_file
        src/io/load_map_from_file.cc)
set_wavemap_target_properties(load_map_from_file)
target_link_libraries(load_map_from_file glog)
cs_add_executable(receive_map_over_ros
        src/io/receive_map_over_ros.cc)
set_wavemap_target_properties(receive_map_over_ros)
target_link_libraries(receive_map_over_ros glog)
cs_add_executable(send_map_over_ros
        src/io/send_map_over_ros.cc)
set_wavemap_target_properties(send_map_over_ros)
target_link_libraries(send_map_over_ros glog)

cs_add_executable(fixed_resolution
        src/queries/fixed_resolution.cc)
set_wavemap_target_properties(fixed_resolution)
target_link_libraries(fixed_resolution glog)
cs_add_executable(multi_resolution
        src/queries/multi_resolution.cc)
set_wavemap_target_properties(multi_resolution)
target_link_libraries(multi_resolution glog)
cs_add_executable(accelerated_queries
        src/queries/accelerated_queries.cc)
set_wavemap_target_properties(accelerated_queries)
target_link_libraries(accelerated_queries glog)

cs_add_executable(nearest_neighbor_interpolation
        src/queries/nearest_neighbor_interpolation.cc)
set_wavemap_target_properties(nearest_neighbor_interpolation)
target_link_libraries(nearest_neighbor_interpolation glog)
cs_add_executable(trilinear_interpolation
        src/queries/trilinear_interpolation.cc)
set_wavemap_target_properties(trilinear_interpolation)
target_link_libraries(trilinear_interpolation glog)

cs_add_executable(classification
        src/queries/classification.cc)
set_wavemap_target_properties(classification)
target_link_libraries(classification glog)

cs_add_executable(occupancy_to_esdf
        src/planning/occupancy_to_esdf.cc)
set_wavemap_target_properties(occupancy_to_esdf)
target_link_libraries(occupancy_to_esdf glog)
