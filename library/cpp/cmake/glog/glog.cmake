include(FetchContent)
FetchContent_Declare(glog
    GIT_REPOSITORY https://github.com/google/glog.git
    GIT_TAG v0.6.0)
FetchContent_GetProperties(glog)
if (NOT glog_POPULATED)
  FetchContent_Populate(glog)
  if(${CMAKE_VERSION} GREATER_EQUAL 3.25)
    add_subdirectory(${glog_SOURCE_DIR} ${glog_BINARY_DIR}
        SYSTEM EXCLUDE_FROM_ALL)
  else()
    # Emulate the SYSTEM flag introduced in CMake 3.25. Withouth this flag the
    # compiler will consider this 3rdparty headers as source code and fail due
    # the -Werror flag.
    add_subdirectory(${glog_SOURCE_DIR} ${glog_BINARY_DIR} EXCLUDE_FROM_ALL)
    get_target_property(glog_include_dirs glog INTERFACE_INCLUDE_DIRECTORIES)
    set_target_properties(glog
        PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${glog_include_dirs}")
  endif()
endif ()
