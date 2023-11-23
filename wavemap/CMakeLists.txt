cmake_minimum_required(VERSION 3.16...3.26)
project(wavemap_cpp VERSION 0.1.0 LANGUAGES CXX)

option(USE_CCACHE "Build using Ccache if found on the path" ON)
option(USE_SYSTEM_EIGEN3 "Use system pre-installed Eigen" OFF)
option(USE_SYSTEM_GLOG "Use system pre-installed glog" OFF) # requires cmake >= 3.21
option(USE_SYSTEM_TRACY "Use system pre-installed tracy" OFF)
option(USE_SYSTEM_BOOST "Use system pre-installed boost" ON)

# ccache setup
if(USE_CCACHE)
  find_program(CCACHE_PATH ccache)
  if(CCACHE_PATH)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
    message(STATUS "Using ccache: ${CCACHE_PATH}")
  endif()
endif()

# Set build type (repeat here for C++ only consumers)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

include(3rdparty/find_dependencies.cmake)
include(cmake/CompilerOptions.cmake)

add_subdirectory(core)
add_subdirectory(io)