# Declare the wavemap io library
add_library(wavemap_io)
add_library(wavemap::wavemap_io ALIAS wavemap_io)

# Configure target properties, include directories and linking
set_wavemap_target_properties(wavemap_io)
add_wavemap_include_directories(wavemap_io)
target_link_libraries(wavemap_io PUBLIC Eigen3::Eigen glog wavemap_core)

# Set sources
target_sources(wavemap_io PRIVATE file_conversions.cc stream_conversions.cc)

# Support installs
if (GENERATE_WAVEMAP_INSTALL_RULES)
  # Mark headers for installation
  install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/wavemap/io/
      DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/wavemap/io
      FILES_MATCHING PATTERN "*.h")

  # Mark target for installation and add it to the CMake exports file
  install(TARGETS wavemap_io
      EXPORT wavemap-targets
      LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
      ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
      RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif ()
