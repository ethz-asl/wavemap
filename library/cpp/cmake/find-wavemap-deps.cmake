if (CMAKE_VERSION VERSION_GREATER 3.24)
  cmake_policy(SET CMP0135 OLD)
endif ()

# Eigen
if (USE_SYSTEM_EIGEN)
  find_package(Eigen3 QUIET NO_MODULE)
endif ()
if (USE_SYSTEM_EIGEN AND TARGET Eigen3::Eigen)
  message(STATUS "Using system Eigen (version ${Eigen3_VERSION})")
else ()
  message(STATUS "Fetching external Eigen")
  set(USE_SYSTEM_EIGEN OFF)
  include("${CMAKE_CURRENT_LIST_DIR}/eigen/eigen.cmake")
endif ()

# GLOG
if (USE_SYSTEM_GLOG)
  find_package(glog QUIET)
  if (NOT glog_FOUND)
    find_package(PkgConfig QUIET)
    if (PkgConfig_FOUND)
      pkg_check_modules(glog QUIET libglog)
    endif ()
  endif ()
endif ()
if (USE_SYSTEM_GLOG AND glog_FOUND)
  message(STATUS "Using system Glog (version ${glog_VERSION})")
else ()
  message(STATUS "Fetching external Glog")
  set(USE_SYSTEM_GLOG OFF)
  include("${CMAKE_CURRENT_LIST_DIR}/glog/glog.cmake")
endif ()

# Boost's Preprocessor
if (USE_SYSTEM_BOOST)
  find_package(Boost 1.71 QUIET COMPONENTS headers)
  # Boost's system package exposes all headers through a single Boost::headers
  # target. However, wavemap only needs Boost's Preprocessor component which we
  # selectively pull in as Boost::preprocessor when using FetchContent. To allow
  # targets to link against Boost::preprocessor regardless of which install mode
  # is used, we define an alias from Boost::preprocessor to Boost::headers.
  if (Boost_FOUND)
    set_target_properties(Boost::headers PROPERTIES IMPORTED_GLOBAL TRUE)
    add_library(Boost::preprocessor ALIAS Boost::headers)
  endif ()
endif ()
if (USE_SYSTEM_BOOST AND TARGET Boost::preprocessor)
  message(STATUS "Using system Boost (version ${Boost_VERSION})")
else ()
  message(STATUS "Fetching external Boost")
  set(USE_SYSTEM_BOOST OFF)
  include(
      "${CMAKE_CURRENT_LIST_DIR}/boost_preprocessor/boost_preprocessor.cmake")
endif ()

# Optional dependencies
if (TRACY_ENABLE)
  find_package(Tracy QUIET)
  if (NOT Tracy_FOUND)
    message(FATAL_ERROR
        "Profiling is enabled (TRACY_ENABLE=ON), but Tracy is not available. "
        "Please switch it to OFF or install Tracy as a system package with: "
        "cmake -S . -B build -DTRACY_ENABLE=ON -DBUILD_SHARED_LIBS=true "
        "-DCMAKE_BUILD_TYPE=Release && cmake --build build --parallel && "
        "sudo make install")
  endif ()
endif ()
