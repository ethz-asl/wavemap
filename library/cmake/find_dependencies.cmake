# Eigen
if (USE_SYSTEM_EIGEN)
  find_package(Eigen3 QUIET NO_MODULE)
endif ()
if (USE_SYSTEM_EIGEN AND TARGET Eigen3::Eigen)
  message(STATUS "Using system Eigen")
else ()
  message(STATUS "Fetching external Eigen")
  set(USE_SYSTEM_EIGEN OFF)
  include("${CMAKE_CURRENT_LIST_DIR}/eigen/eigen.cmake")
endif ()

# GLOG
if (USE_SYSTEM_GLOG)
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(glog REQUIRED libglog)
endif ()
if (USE_SYSTEM_GLOG AND glog_FOUND)
  message(STATUS "Using system Glog")
else ()
  message(STATUS "Fetching external Glog")
  set(USE_SYSTEM_GLOG OFF)
  include("${CMAKE_CURRENT_LIST_DIR}/glog/glog.cmake")
endif ()

# Boost's Preprocessor
if (USE_SYSTEM_BOOST)
  find_package(Boost 1.71 COMPONENTS headers)
  # Boost's system package exposes all headers through a single Boost::headers
  # target. However, wavemap only needs Boost's Preprocessor component which we
  # selectively pull in as Boost::preprocessor when using FetchContent. To allow
  # targets to link against Boost::preprocessor regardless of which install mode
  # is used, we define an alias from Boost::preprocessor to Boost::headers.
  set_target_properties(Boost::headers PROPERTIES IMPORTED_GLOBAL TRUE)
  add_library(Boost::preprocessor ALIAS Boost::headers)
endif ()
if (USE_SYSTEM_BOOST AND TARGET Boost::preprocessor)
  message(STATUS "Using system Boost")
else ()
  message(STATUS "Fetching external Boost")
  set(USE_SYSTEM_BOOST OFF)
  include(
      "${CMAKE_CURRENT_LIST_DIR}/boost_preprocessor/boost_preprocessor.cmake")
endif ()

# Optional dependencies
find_package(tracy QUIET)
