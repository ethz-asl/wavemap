# Binaries
add_executable(experiment experiment.cc)
set_wavemap_target_properties(experiment)
target_include_directories(experiment PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(experiment PUBLIC
    wavemap::wavemap_core wavemap::wavemap_io ${catkin_LIBRARIES})

add_executable(publish_experiment publish_experiment.cc)
set_wavemap_target_properties(publish_experiment)
target_include_directories(publish_experiment PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(publish_experiment PUBLIC
    wavemap::wavemap_core wavemap::wavemap_io ${catkin_LIBRARIES})

add_executable(blocks_experiment blocks_experiment.cc)
set_wavemap_target_properties(blocks_experiment)
target_include_directories(blocks_experiment PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(blocks_experiment PUBLIC
    wavemap::wavemap_core wavemap::wavemap_io ${catkin_LIBRARIES})

add_executable(partial_save_experiment partial_save_experiment.cc)
set_wavemap_target_properties(partial_save_experiment)
target_include_directories(partial_save_experiment PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(partial_save_experiment PUBLIC
    wavemap::wavemap_core wavemap::wavemap_io ${catkin_LIBRARIES})
