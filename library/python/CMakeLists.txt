cmake_minimum_required(VERSION 3.15...3.27)
project(pywavemap LANGUAGES CXX)

# Warn if the user invokes CMake directly
if (NOT SKBUILD)
  message(WARNING "\
  This CMake file is meant to be executed using 'scikit-build-core'.
  Running it directly will almost certainly not produce the desired
  result. If you are a user trying to install this package, use the
  command below, which will install all necessary build dependencies,
  compile the package in an isolated environment, and then install it.
  =====================================================================
   $ pip install .
  =====================================================================
  If you are a software developer, and this is your own package, then
  it is usually much more efficient to install the build dependencies
  in your environment once and use the following command that avoids
  a costly creation of a new virtual environment at every compilation:
  =====================================================================
   $ pip install nanobind scikit-build-core[pyproject]
   $ pip install --no-build-isolation -ve .
  =====================================================================
  You may optionally add -Ceditable.rebuild=true to auto-rebuild when
  the package is imported. Otherwise, you need to rerun the above
  after editing C++ files.")
endif ()

# Load the wavemap library (if not already loaded)
if (NOT TARGET wavemap::wavemap_core)
  if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../../library/cpp)
    message(STATUS "Loading wavemap library installed on system")
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../library/cpp
        ${CMAKE_CURRENT_BINARY_DIR}/wavemap)
  else ()
    set(WAVEMAP_TAG feature/pywavemap)
    message(STATUS "Loading wavemap library from GitHub (tag ${WAVEMAP_TAG})")
    cmake_minimum_required(VERSION 3.18)
    include(FetchContent)
    FetchContent_Declare(
        ext_wavemap PREFIX wavemap
        GIT_REPOSITORY https://github.com/ethz-asl/wavemap.git
        GIT_TAG ${WAVEMAP_TAG}
        GIT_SHALLOW 1
        SOURCE_SUBDIR library/cpp)
    FetchContent_MakeAvailable(ext_wavemap)
  endif ()
endif ()

# Try to import all Python components potentially needed by nanobind
find_package(Python 3.8
    REQUIRED COMPONENTS Interpreter Development.Module
    OPTIONAL_COMPONENTS Development.SABIModule)

# Import nanobind through CMake's find_package mechanism
find_package(nanobind CONFIG REQUIRED)

# Compile our extension
nanobind_add_module(pywavemap STABLE_ABI
    src/pywavemap.cc
    src/logging.cc
    src/map.cc
    src/param.cc
    src/pipeline.cc)
set_wavemap_target_properties(pywavemap)
target_include_directories(pywavemap PUBLIC include)
target_link_libraries(pywavemap PUBLIC
    wavemap::wavemap_core wavemap::wavemap_io wavemap::wavemap_pipeline)

# Install directive for scikit-build-core
install(TARGETS pywavemap LIBRARY DESTINATION pywavemap)
