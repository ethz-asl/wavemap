# Set glog options
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build shared libraries")
set(WITH_GFLAGS OFF CACHE BOOL "Build with gflags support")
set(BUILD_TESTING OFF CACHE BOOL "Don't build glog tests")
set(INSTALL_GLOG_HEADERS OFF CACHE BOOL "Install glog headers")

# Include glog using FetchContent
include(FetchContent)
FetchContent_Declare(
  glog
  GIT_REPOSITORY https://github.com/google/glog.git
  GIT_TAG v0.4.0
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(glog)
