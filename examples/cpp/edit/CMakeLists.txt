# Binaries
add_executable(crop_map crop_map.cc)
set_wavemap_target_properties(crop_map)
target_link_libraries(crop_map PUBLIC
    wavemap::wavemap_core wavemap::wavemap_io)

add_executable(multiply_map multiply_map.cc)
set_wavemap_target_properties(multiply_map)
target_link_libraries(multiply_map PUBLIC
    wavemap::wavemap_core wavemap::wavemap_io)

add_executable(sum_map sum_map.cc)
set_wavemap_target_properties(sum_map)
target_link_libraries(sum_map PUBLIC
    wavemap::wavemap_core wavemap::wavemap_io)

add_executable(transform_map transform_map.cc)
set_wavemap_target_properties(transform_map)
target_link_libraries(transform_map PUBLIC
    wavemap::wavemap_core wavemap::wavemap_io)
