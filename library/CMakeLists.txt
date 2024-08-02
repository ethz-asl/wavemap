cmake_minimum_required(VERSION 3.10)
project(wavemap VERSION 2.0.0 LANGUAGES CXX)

# CMake helpers and general wavemap tooling (e.g. to run clang-tidy CI)
include(GNUInstallDirs)
include(cmake/wavemap-extras.cmake)
enable_wavemap_general_tooling()

# Dependencies
option(USE_SYSTEM_EIGEN "Use system pre-installed Eigen" ON)
option(USE_SYSTEM_GLOG "Use system pre-installed glog" ON)
option(USE_SYSTEM_BOOST "Use system pre-installed Boost" ON)
include(cmake/find-wavemap-deps.cmake)

# Libraries
add_subdirectory(src/core)
add_subdirectory(src/io)
add_subdirectory(src/pipeline)

# Tests
if (ENABLE_TESTING OR CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif ()

# Benchmarks
if (ENABLE_BENCHMARKING)
  add_subdirectory(benchmark)
endif ()

# Install
# Mark embedded 3rd party header-only libraries for installation
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/wavemap/3rd_party/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/wavemap/3rd_party
    FILES_MATCHING PATTERN "*.h")
# Install the CMake exports file
install(EXPORT wavemap-targets
    FILE wavemap-targets.cmake
    NAMESPACE wavemap::
    DESTINATION ${CMAKE_INSTALL_DATADIR}/wavemap/cmake)
# Generate and install a version file
include(CMakePackageConfigHelpers)
write_basic_package_version_file(wavemap-config-version.cmake
    VERSION ${PACKAGE_VERSION}
    COMPATIBILITY AnyNewerVersion)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/wavemap-config-version.cmake"
    DESTINATION ${CMAKE_INSTALL_DATADIR}/wavemap/cmake)
# Install our CMake helper functions
install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/cmake/wavemap-extras.cmake"
    DESTINATION ${CMAKE_INSTALL_DATADIR}/wavemap/cmake)
# Generate and install a package config file
configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/wavemap-config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/wavemap-config.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_DATADIR}/wavemap/cmake
    NO_CHECK_REQUIRED_COMPONENTS_MACRO)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/wavemap-config.cmake"
    DESTINATION ${CMAKE_INSTALL_DATADIR}/wavemap/cmake)
