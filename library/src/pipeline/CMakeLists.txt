add_library(wavemap_pipeline)
add_library(wavemap::wavemap_pipeline ALIAS wavemap_pipeline)

add_wavemap_include_directories(wavemap_pipeline)
target_sources(wavemap_pipeline PRIVATE
  map_operations/map_operation_factory.cc
  map_operations/prune_map_operation.cc
  map_operations/threshold_map_operation.cc
  pipeline.cc)

set_wavemap_target_properties(wavemap_pipeline)
target_link_libraries(wavemap_pipeline PUBLIC Eigen3::Eigen glog)
