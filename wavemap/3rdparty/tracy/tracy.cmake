include(FetchContent)
FetchContent_Declare(Tracy
    GIT_REPOSITORY https://github.com/wolfpld/tracy.git
    GIT_TAG v0.10
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(Tracy)
# if(NOT tracy_POPULATED)
#   FetchContent_Populate(tracy)
#   if(${CMAKE_VERSION} GREATER_EQUAL 3.25)
#     add_subdirectory(${tracy_SOURCE_DIR} ${tracy_BINARY_DIR} SYSTEM EXCLUDE_FROM_ALL)
#   else()
#     # Emulate the SYSTEM flag introduced in CMake 3.25. Withouth this flag the compiler will
#     # consider this 3rdparty headers as source code and fail due the -Werror flag.
#     add_subdirectory(${tracy_SOURCE_DIR} ${tracy_BINARY_DIR} EXCLUDE_FROM_ALL)
#     get_target_property(tracy_include_dirs tracy INTERFACE_INCLUDE_DIRECTORIES)
#     set_target_properties(tracy PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${tracy_include_dirs}")
#   endif()
# endif()

# if(NOT tracy_POPULATED)
#     FetchContent_Populate(tracy)
#     add_subdirectory(${tracy_SOURCE_DIR} ${tracy_BINARY_DIR} SYSTEM EXCLUDE_FROM_ALL)
# endif()

