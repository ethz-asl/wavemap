# Binaries
add_executable(occupancy_to_esdf occupancy_to_esdf.cc)
set_wavemap_target_properties(occupancy_to_esdf)
target_link_libraries(occupancy_to_esdf PUBLIC
    wavemap::wavemap_core wavemap::wavemap_io)
