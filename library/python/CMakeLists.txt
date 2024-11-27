cmake_minimum_required(VERSION 3.18)
project(pywavemap VERSION 2.2.0 LANGUAGES CXX)

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
  if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../cpp/)
    message(STATUS "Loading wavemap C++ library sources")
    set(GENERATE_WAVEMAP_INSTALL_RULES OFF)
    set(BUILD_SHARED_LIBS OFF)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../cpp
        ${CMAKE_CURRENT_BINARY_DIR}/wavemap)
  else ()
    message(ERROR
        "Can not load wavemap C++ library sources. Are you using an old "
        "version of pip? If so, retry after upgrading with "
        "\"python3 -m pip install --upgrade pip\".")
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
nanobind_add_module(_pywavemap_bindings STABLE_ABI
    src/pywavemap.cc
    src/convert.cc
    src/indices.cc
    src/logging.cc
    src/maps.cc
    src/measurements.cc
    src/param.cc
    src/pipeline.cc)
set_wavemap_target_properties(_pywavemap_bindings)
target_include_directories(_pywavemap_bindings PRIVATE include)
target_link_libraries(_pywavemap_bindings PRIVATE
    wavemap::wavemap_core wavemap::wavemap_io wavemap::wavemap_pipeline)
# Disable some default wavemap warnings that trigger on nanobind
target_compile_options(_pywavemap_bindings PRIVATE
    -Wno-pedantic -Wno-unused-result -Wno-suggest-attribute=const)

# Install directive for scikit-build-core
install(TARGETS _pywavemap_bindings LIBRARY DESTINATION pywavemap)

# Generate stubs
nanobind_add_stub(pywavemap_stub INSTALL_TIME
    MODULE _pywavemap_bindings
    OUTPUT "pywavemap/__init__.pyi"
    MARKER_FILE "pywavemap/py.typed"
    PYTHON_PATH "pywavemap")
nanobind_add_stub(pywavemap_convert_stub INSTALL_TIME
    MODULE _pywavemap_bindings.convert
    OUTPUT "pywavemap/convert.pyi"
    PYTHON_PATH "pywavemap")
nanobind_add_stub(pywavemap_logging_stub INSTALL_TIME
    MODULE _pywavemap_bindings.logging
    OUTPUT "pywavemap/logging.pyi"
    PYTHON_PATH "pywavemap")
nanobind_add_stub(pywavemap_param_stub INSTALL_TIME
    MODULE _pywavemap_bindings.param
    OUTPUT "pywavemap/param.pyi"
    PYTHON_PATH "pywavemap")
