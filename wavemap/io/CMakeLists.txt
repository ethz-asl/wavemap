add_library(io STATIC)
add_library(wavemap::io ALIAS io)
target_sources(io PRIVATE
    src/file_conversions.cc
    src/stream_conversions.cc
)
target_include_directories(io PRIVATE include ../core/include)
target_link_libraries(io PUBLIC wavemap::core)
set_global_target_properties(io)
