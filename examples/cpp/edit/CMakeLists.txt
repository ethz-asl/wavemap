# Binaries
add_executable(transform_map transform_map.cc)
set_wavemap_target_properties(transform_map)
target_link_libraries(transform_map PUBLIC
    wavemap::wavemap_core wavemap::wavemap_io)
