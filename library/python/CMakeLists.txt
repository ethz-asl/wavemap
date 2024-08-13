cmake_minimum_required(VERSION 3.15...3.27)
project(pywavemap)

# Build in Release mode by default
if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE
      PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif ()

# Load the wavemap library (if not already loaded)
if (NOT TARGET wavemap::wavemap_core)
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../library/cpp
      ${CMAKE_CURRENT_BINARY_DIR}/wavemap)
endif ()

# Load Python
if (CMAKE_VERSION VERSION_LESS 3.18)
  set(DEV_MODULE Development)
else ()
  set(DEV_MODULE Development.Module)
endif ()
find_package(Python 3.8 COMPONENTS Interpreter ${DEV_MODULE} REQUIRED)

# Detect the installed nanobind package and import it into CMake
execute_process(COMMAND "${Python_EXECUTABLE}" -m nanobind --cmake_dir
    OUTPUT_STRIP_TRAILING_WHITESPACE OUTPUT_VARIABLE NB_DIR)
list(APPEND CMAKE_PREFIX_PATH "${NB_DIR}")
find_package(nanobind CONFIG REQUIRED)

# Generate the bindings
nanobind_add_module(pywavemap src/pywavemap.cc src/param_conversions.cc)
set_wavemap_target_properties(pywavemap)
target_include_directories(pywavemap PUBLIC include)
target_link_libraries(pywavemap
    PUBLIC wavemap::wavemap_core wavemap::wavemap_io)
