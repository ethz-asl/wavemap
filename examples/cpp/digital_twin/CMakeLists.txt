# Binaries
add_executable(new_voxel voxel_data/new_voxel.cc)
set_wavemap_target_properties(new_voxel)
target_link_libraries(new_voxel PUBLIC wavemap::wavemap_core)

add_executable(voxel_haar_transform voxel_data/voxel_haar_transform.cc)
set_wavemap_target_properties(voxel_haar_transform)
target_link_libraries(voxel_haar_transform PUBLIC wavemap::wavemap_core)

add_executable(voxel_block_experiment map_api/voxel_block_experiment.cc)
set_wavemap_target_properties(voxel_block_experiment)
target_link_libraries(voxel_block_experiment PUBLIC wavemap::wavemap_core)

add_executable(voxel_map_experiment map_api/voxel_map_experiment.cc)
set_wavemap_target_properties(voxel_map_experiment)
target_link_libraries(voxel_map_experiment PUBLIC wavemap::wavemap_core)

add_executable(legacy_hashed_wavelet_octree_experiment
    io/legacy_hashed_wavelet_octree_experiment.cc)
set_wavemap_target_properties(legacy_hashed_wavelet_octree_experiment)
target_link_libraries(legacy_hashed_wavelet_octree_experiment
    PUBLIC wavemap::wavemap_core)

add_executable(legacy_file_io_experiment io/legacy_file_io_experiment.cc)
set_wavemap_target_properties(legacy_file_io_experiment)
target_link_libraries(legacy_file_io_experiment
    PUBLIC wavemap::wavemap_core wavemap::wavemap_io)

add_executable(layered_map_io_experiment io/layered_map_io_experiment.cc)
set_wavemap_target_properties(layered_map_io_experiment)
target_link_libraries(layered_map_io_experiment PUBLIC wavemap::wavemap_core)

if ("$ENV{ROS_VERSION}" STREQUAL "1")
  find_package(catkin REQUIRED
      COMPONENTS roscpp wavemap wavemap_msgs wavemap_ros_conversions)

  add_executable(experiment old_experiments/experiment.cc)
  set_wavemap_target_properties(experiment)
  target_include_directories(experiment PUBLIC ${catkin_INCLUDE_DIRS})
  target_link_libraries(experiment PUBLIC
      wavemap::wavemap_core wavemap::wavemap_io ${catkin_LIBRARIES})

  add_executable(publish_experiment old_experiments/publish_experiment.cc)
  set_wavemap_target_properties(publish_experiment)
  target_include_directories(publish_experiment PUBLIC ${catkin_INCLUDE_DIRS})
  target_link_libraries(publish_experiment PUBLIC
      wavemap::wavemap_core wavemap::wavemap_io ${catkin_LIBRARIES})

  add_executable(blocks_experiment old_experiments/blocks_experiment.cc)
  set_wavemap_target_properties(blocks_experiment)
  target_include_directories(blocks_experiment PUBLIC ${catkin_INCLUDE_DIRS})
  target_link_libraries(blocks_experiment PUBLIC
      wavemap::wavemap_core wavemap::wavemap_io ${catkin_LIBRARIES})

  add_executable(partial_save_experiment old_experiments/partial_save_experiment.cc)
  set_wavemap_target_properties(partial_save_experiment)
  target_include_directories(partial_save_experiment PUBLIC ${catkin_INCLUDE_DIRS})
  target_link_libraries(partial_save_experiment PUBLIC
      wavemap::wavemap_core wavemap::wavemap_io ${catkin_LIBRARIES})

  add_executable(layered_ros_msg_experiment ros/layered_ros_msg_experiment.cc)
  set_wavemap_target_properties(layered_ros_msg_experiment)
  target_include_directories(layered_ros_msg_experiment PUBLIC ${catkin_INCLUDE_DIRS})
  target_link_libraries(layered_ros_msg_experiment PUBLIC
      wavemap::wavemap_core ${catkin_LIBRARIES})

  add_executable(publish_layered_map_experiment ros/publish_layered_map_experiment.cc)
  set_wavemap_target_properties(publish_layered_map_experiment)
  target_include_directories(publish_layered_map_experiment PUBLIC ${catkin_INCLUDE_DIRS})
  target_link_libraries(publish_layered_map_experiment PUBLIC
      wavemap::wavemap_core ${catkin_LIBRARIES})

  add_executable(subscribe_layered_map_experiment ros/subscribe_layered_map_experiment.cc)
  set_wavemap_target_properties(subscribe_layered_map_experiment)
  target_include_directories(subscribe_layered_map_experiment PUBLIC ${catkin_INCLUDE_DIRS})
  target_link_libraries(subscribe_layered_map_experiment PUBLIC
      wavemap::wavemap_core ${catkin_LIBRARIES})
else ()
  message(STATUS "Skipping ROS digital_twin examples because ROS1 is not active.")
endif ()
