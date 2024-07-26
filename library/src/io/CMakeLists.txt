add_library(wavemap_io)
add_library(wavemap::wavemap_io ALIAS wavemap_io)

add_wavemap_include_directories(wavemap_io)
target_sources(wavemap_io PRIVATE file_conversions.cc stream_conversions.cc)

set_wavemap_target_properties(wavemap_io)
target_link_libraries(wavemap_io PUBLIC Eigen3::Eigen glog wavemap_core)
