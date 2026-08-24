cmake_minimum_required(VERSION 3.0.2)
project(wavemap_ros)

# Find dependencies
# NOTE: OpenCV is normally included through cv_bridge, but we still explicitly
#       include it to avoid issues on Jetson.
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
    wavemap wavemap_msgs wavemap_ros_conversions
    roscpp rosbag message_filters cv_bridge image_transport tf2_ros
    std_srvs sensor_msgs visualization_msgs)

# Register catkin package
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS
    wavemap wavemap_msgs wavemap_ros_conversions
    roscpp rosbag message_filters cv_bridge image_transport tf2_ros
    std_srvs sensor_msgs visualization_msgs)


# Optional dependencies
find_package(livox_ros_driver2 QUIET)
if (livox_ros_driver2_FOUND)
  include_directories(${livox_ros_driver2_INCLUDE_DIRS})
  add_compile_definitions(LIVOX_AVAILABLE)
endif ()

# Libraries
add_library(${PROJECT_NAME}
    src/inputs/depth_image_topic_input.cc
    src/inputs/pointcloud_topic_input.cc
    src/inputs/ros_input_base.cc
    src/inputs/ros_input_factory.cc
    src/map_operations/crop_map_operation.cc
    src/map_operations/map_ros_operation_factory.cc
    src/map_operations/publish_map_operation.cc
    src/map_operations/publish_pointcloud_operation.cc
    src/utils/pointcloud_undistorter.cc
    src/utils/ros_logging_level.cc
    src/utils/rosbag_processor.cc
    src/utils/tf_transformer.cc
    src/ros_server.cc)
set_wavemap_target_properties(${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}
    PUBLIC include ${catkin_INCLUDE_DIRS}
    PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../examples/cpp/digital_twin/common
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../examples/cpp/digital_twin/ouster_mapping)
target_link_libraries(${PROJECT_NAME}
    PUBLIC ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

# Binaries
add_executable(ros_server app/ros_server.cc)
set_wavemap_target_properties(ros_server)
target_link_libraries(ros_server ${PROJECT_NAME})

add_executable(occupancy_layered_server
    ../../../examples/cpp/digital_twin/ouster_mapping/occupancy_sensor_inputs.cc)
set_wavemap_target_properties(occupancy_layered_server)
target_include_directories(occupancy_layered_server PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../examples/cpp/digital_twin/common
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../examples/cpp/digital_twin/ouster_mapping)
target_link_libraries(occupancy_layered_server ${PROJECT_NAME})

add_executable(reflectivity_layered_server
    ../../../examples/cpp/digital_twin/ouster_mapping/reflectivity_only_sensor_inputs.cc)
set_wavemap_target_properties(reflectivity_layered_server)
target_include_directories(reflectivity_layered_server PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../examples/cpp/digital_twin/common
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../examples/cpp/digital_twin/ouster_mapping)
target_link_libraries(reflectivity_layered_server ${PROJECT_NAME})

add_executable(reflectivity_class_layered_server
    ../../../examples/cpp/digital_twin/ouster_mapping/ouster_sensor_inputs.cc)
set_wavemap_target_properties(reflectivity_class_layered_server)
target_include_directories(reflectivity_class_layered_server PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../examples/cpp/digital_twin/common
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../examples/cpp/digital_twin/ouster_mapping)
target_link_libraries(reflectivity_class_layered_server ${PROJECT_NAME})

add_executable(rosbag_processor app/rosbag_processor.cc)
set_wavemap_target_properties(rosbag_processor)
target_link_libraries(rosbag_processor ${PROJECT_NAME})

add_executable(publish_field_layer_updates_experiment
    app/publish_field_layer_updates_experiment.cc)
set_wavemap_target_properties(publish_field_layer_updates_experiment)
target_link_libraries(publish_field_layer_updates_experiment ${PROJECT_NAME})

add_executable(field_layer_sensor_bridge
    app/field_layer_sensor_bridge.cc)
set_wavemap_target_properties(field_layer_sensor_bridge)
target_link_libraries(field_layer_sensor_bridge ${PROJECT_NAME})

add_executable(layered_incremental_publish_experiment
    app/layered_incremental_publish_experiment.cc)
set_wavemap_target_properties(layered_incremental_publish_experiment)
target_include_directories(layered_incremental_publish_experiment PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../../../examples/cpp/digital_twin/common)
target_link_libraries(layered_incremental_publish_experiment ${PROJECT_NAME})

add_executable(computed_point_field_binding_experiment
    app/computed_point_field_binding_experiment.cc)
set_wavemap_target_properties(computed_point_field_binding_experiment)
target_link_libraries(computed_point_field_binding_experiment ${PROJECT_NAME})

# Support installs
# Mark targets for installation
install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
# Mark headers for installation
install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    FILES_MATCHING PATTERN "*.h")
# Mark config files for installation
install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
