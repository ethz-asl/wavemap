cmake_minimum_required(VERSION 3.0.2)
project(wavemap_examples)

add_definitions(-std=c++17)

find_package(catkin REQUIRED COMPONENTS wavemap wavemap_io wavemap_msgs wavemap_ros_conversions minkindr)

# Compiler definitions and options
add_wavemap_compile_definitions_and_options()
add_compile_options(-Wno-suggest-attribute=const)

# For all targets
catkin_package(
    INCLUDE_DIRS
    include
    LIBRARIES
    ${PROJECT_NAME}
    CATKIN_DEPENDS
    wavemap
    wavemap_msgs
    wavemap_io
    wavemap_ros_conversions
)

# Add minkindr as header-only library
add_library(minkindr INTERFACE)
target_include_directories(minkindr INTERFACE ${minkindr_SOURCE_DIR}/include)

include_directories(include ${catkin_INCLUDE_DIRS} wavemap wavemap_io minkindr)

# Binaries
add_executable(save_map_to_file
        src/io/save_map_to_file.cc)
target_link_libraries(save_map_to_file PUBLIC ${catkin_LIBRARIES} minkindr)
add_executable(load_map_from_file
        src/io/load_map_from_file.cc)
target_link_libraries(load_map_from_file PUBLIC ${catkin_LIBRARIES} minkindr)
add_executable(receive_map_over_ros
        src/io/receive_map_over_ros.cc)
target_link_libraries(receive_map_over_ros PUBLIC ${catkin_LIBRARIES} minkindr)
add_executable(send_map_over_ros
        src/io/send_map_over_ros.cc)
target_link_libraries(send_map_over_ros PUBLIC ${catkin_LIBRARIES} minkindr)

add_executable(fixed_resolution
        src/queries/fixed_resolution.cc)
target_link_libraries(fixed_resolution PUBLIC ${catkin_LIBRARIES} minkindr)
add_executable(multi_resolution
        src/queries/multi_resolution.cc)
target_link_libraries(multi_resolution PUBLIC ${catkin_LIBRARIES} minkindr)
add_executable(accelerated_queries
        src/queries/accelerated_queries.cc)
target_link_libraries(accelerated_queries PUBLIC ${catkin_LIBRARIES} minkindr)
add_executable(nearest_neighbor_interpolation
        src/queries/nearest_neighbor_interpolation.cc)
target_link_libraries(nearest_neighbor_interpolation PUBLIC ${catkin_LIBRARIES} minkindr)
add_executable(trilinear_interpolation
        src/queries/trilinear_interpolation.cc)
target_link_libraries(trilinear_interpolation PUBLIC ${catkin_LIBRARIES} minkindr)
add_executable(classification
        src/queries/classification.cc)
target_link_libraries(classification PUBLIC ${catkin_LIBRARIES} minkindr)