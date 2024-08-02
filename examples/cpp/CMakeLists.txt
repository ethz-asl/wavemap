cmake_minimum_required(VERSION 3.0.2)
project(wavemap_examples_cpp VERSION 2.0.0 LANGUAGES CXX)

# Load the wavemap library
# First, try to load it from sources
if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../../library)
  message(STATUS "Using wavemap library from local sources")
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../library
      ${CMAKE_CURRENT_BINARY_DIR}/wavemap)
else ()
  # If sources are not available, see if wavemap is installed as a system lib
  find_package(wavemap QUIET)
  if (wavemap_FOUND)
    message(STATUS "Using wavemap library installed on system")
  else () # Otherwise, fetch wavemap's code from GitHub
    set(WAVEMAP_TAG develop/v2.0)
    message(STATUS "Fetching wavemap library from GitHub (tag ${WAVEMAP_TAG})")
    cmake_minimum_required(VERSION 3.18)
    include(FetchContent)
    FetchContent_Declare(
        ext_wavemap PREFIX wavemap
        GIT_REPOSITORY https://github.com/ethz-asl/wavemap.git
        GIT_TAG ${WAVEMAP_TAG}
        GIT_SHALLOW 1
        SOURCE_SUBDIR library)
    FetchContent_MakeAvailable(ext_wavemap)
  endif ()
endif ()

# Add each set of examples
add_subdirectory(io)
add_subdirectory(queries)
add_subdirectory(planning)
