# Binaries
add_executable(example_pipeline example_pipeline.cc)
set_wavemap_target_properties(example_pipeline)
target_link_libraries(example_pipeline PUBLIC
    wavemap::wavemap_core
    wavemap::wavemap_io
    wavemap::wavemap_pipeline)
