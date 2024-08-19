# Declare the wavemap pipeline library
add_library(wavemap_pipeline)
add_library(wavemap::wavemap_pipeline ALIAS wavemap_pipeline)

# Configure target properties, include directories and linking
set_wavemap_target_properties(wavemap_pipeline)
add_wavemap_include_directories(wavemap_pipeline)
target_link_libraries(wavemap_pipeline PUBLIC Eigen3::Eigen glog wavemap_core)

# Set sources
target_sources(wavemap_pipeline PRIVATE
    map_operations/map_operation_factory.cc
    map_operations/prune_map_operation.cc
    map_operations/threshold_map_operation.cc
    pipeline.cc)

# Support installs
if (GENERATE_WAVEMAP_INSTALL_RULES)
  # Mark headers for installation
  install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/wavemap/pipeline/
      DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/wavemap/pipeline
      FILES_MATCHING PATTERN "*.h")

  # Mark target for installation and add it to the CMake exports file
  install(TARGETS wavemap_pipeline
      EXPORT wavemap-targets
      LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
      ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
      RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif ()
