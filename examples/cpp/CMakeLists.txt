cmake_minimum_required(VERSION 3.10)
project(wavemap_examples_cpp VERSION 2.1.0 LANGUAGES CXX)

# Load the wavemap library
# First, try to load it from sources
if (TARGET wavemap::wavemap_core)
  message(STATUS "Wavemap library already loaded. No need to load it again.")
elseif (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../../library/cpp)
  message(STATUS "Loading wavemap library from local sources")
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../library/cpp
      ${CMAKE_CURRENT_BINARY_DIR}/wavemap)
else ()
  # If sources are not available, see if wavemap is installed as a system lib
  find_package(wavemap QUIET)
  if (wavemap_FOUND)
    message(STATUS "Loading wavemap library installed on system")
  else () # Otherwise, fetch wavemap's code from GitHub
    set(WAVEMAP_VERSION main)  # Select a git branch, tag or commit
    message(STATUS
        "Loading wavemap library from GitHub (ref ${WAVEMAP_VERSION})")
    cmake_minimum_required(VERSION 3.18)
    include(FetchContent)
    FetchContent_Declare(
        ext_wavemap PREFIX wavemap
        GIT_REPOSITORY https://github.com/ethz-asl/wavemap.git
        GIT_TAG ${WAVEMAP_VERSION}
        GIT_SHALLOW 1
        SOURCE_SUBDIR library/cpp)
    FetchContent_MakeAvailable(ext_wavemap)
  endif ()
endif ()

# Add each set of examples
add_subdirectory(io)
add_subdirectory(queries)
add_subdirectory(planning)
