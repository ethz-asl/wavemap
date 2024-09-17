# Binaries
add_executable(save_map_to_file save_map_to_file.cc)
set_wavemap_target_properties(save_map_to_file)
target_link_libraries(save_map_to_file PUBLIC
    wavemap::wavemap_core wavemap::wavemap_io)

add_executable(load_map_from_file load_map_from_file.cc)
set_wavemap_target_properties(load_map_from_file)
target_link_libraries(load_map_from_file PUBLIC
    wavemap::wavemap_core wavemap::wavemap_io)

add_executable(save_map_to_stream save_map_to_stream.cc)
set_wavemap_target_properties(save_map_to_stream)
target_link_libraries(save_map_to_stream PUBLIC
    wavemap::wavemap_core wavemap::wavemap_io)

add_executable(load_map_from_stream load_map_from_stream.cc)
set_wavemap_target_properties(load_map_from_stream)
target_link_libraries(load_map_from_stream PUBLIC
    wavemap::wavemap_core wavemap::wavemap_io)
