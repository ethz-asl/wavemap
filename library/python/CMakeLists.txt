cmake_minimum_required(VERSION 3.18)
project(pywavemap LANGUAGES CXX)

# Warn if the user invokes CMake directly
if (NOT SKBUILD AND NOT $ENV{CLION_IDE})
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
if (NOT SKBUILD)
  execute_process(COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
      OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE NB_DIR)
  list(APPEND CMAKE_PREFIX_PATH "${NB_DIR}")
endif ()
find_package(nanobind CONFIG REQUIRED)

# Compile our extension
nanobind_add_module(_cpp_bindings STABLE_ABI
    src/pywavemap.cc
    src/logging.cc
    src/map.cc
    src/measurements.cc
    src/param.cc
    src/pipeline.cc)
set_wavemap_target_properties(_cpp_bindings)
target_include_directories(_cpp_bindings PUBLIC include)
target_link_libraries(_cpp_bindings PUBLIC
    wavemap::wavemap_core wavemap::wavemap_io wavemap::wavemap_pipeline)
# Disable some default wavemap warnings that trigger on nanobind
target_compile_options(_cpp_bindings PRIVATE
    -Wno-pedantic -Wno-unused-result -Wno-suggest-attribute=const)

# Install directive for scikit-build-core
install(TARGETS _cpp_bindings LIBRARY DESTINATION pywavemap)

# Generate stubs
nanobind_add_stub(pywavemap_stub INSTALL_TIME
    MODULE _cpp_bindings
    OUTPUT "pywavemap/__init__.pyi"
    PYTHON_PATH "pywavemap")
nanobind_add_stub(pywavemap_logging_stub INSTALL_TIME
    MODULE _cpp_bindings.logging
    OUTPUT "pywavemap/logging.pyi"
    PYTHON_PATH "pywavemap")
nanobind_add_stub(pywavemap_param_stub INSTALL_TIME
    MODULE _cpp_bindings.param
    OUTPUT "pywavemap/param.pyi"
    PYTHON_PATH "pywavemap")
