set(BOOST_BUILD_DOC OFF CACHE BOOL "Don't build Boost docs")
set(BOOST_BUILD_TESTS OFF CACHE BOOL "Don't build Boost tests")
set(BOOST_INCLUDE_LIBRARIES preprocessor)
set(BOOST_ENABLE_CMAKE ON)

include(FetchContent)
FetchContent_Declare(
  boost
  GIT_REPOSITORY https://github.com/boostorg/boost.git
  GIT_TAG boost-1.74.0
  GIT_SHALLOW TRUE
  GIT_PROGRESS TRUE
  EXCLUDE_FROM_ALL TRUE
)
FetchContent_MakeAvailable(boost)

# if(NOT boost_POPULATED)
#   FetchContent_Populate(boost)
#   if(${CMAKE_VERSION} GREATER_EQUAL 3.25)
#     add_subdirectory(${boost_SOURCE_DIR} ${boost_BINARY_DIR} SYSTEM EXCLUDE_FROM_ALL)
#   else()
#     # Emulate the SYSTEM flag introduced in CMake 3.25. Withouth this flag the compiler will
#     # consider this 3rdparty headers as source code and fail due the -Werror flag.
#     add_subdirectory(${boost_SOURCE_DIR} ${boost_BINARY_DIR} EXCLUDE_FROM_ALL)
#     get_target_property(boost_include_dirs boost INTERFACE_INCLUDE_DIRECTORIES)
#     set_target_properties(boost PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${boost_include_dirs}")
#   endif()
# endif()