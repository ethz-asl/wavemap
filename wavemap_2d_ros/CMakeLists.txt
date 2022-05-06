cmake_minimum_required(VERSION 3.0.2)
project(wavemap_2d_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# Configure
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
add_compile_definitions(EIGEN_INITIALIZE_MATRICES_BY_NAN)
add_compile_options(
  -march=native -Wall -Wextra -Wpedantic -Wsuggest-attribute=const
  -Wno-deprecated-copy -Wno-class-memaccess)

if (USE_UBSAN)
  add_compile_options(
    -fsanitize=undefined
    -fsanitize=shift
    -fsanitize=integer-divide-by-zero
    -fsanitize=null
    -fsanitize=return
    -fsanitize=signed-integer-overflow
    -fsanitize=bounds-strict
    -fsanitize=alignment
    -fsanitize=float-divide-by-zero
    -fsanitize=float-cast-overflow
    -fsanitize=enum
    -fsanitize=vptr
    -fsanitize=pointer-overflow
    -fsanitize=builtin
    -fno-omit-frame-pointer
    -g)
  add_link_options(-fsanitize=undefined)
endif ()
if (USE_ASAN)
  add_compile_options(-fsanitize=address -fsanitize-address-use-after-scope
                      -fno-omit-frame-pointer -g)
  add_link_options(-fsanitize=address)
endif ()
if (USE_TSAN)
  add_compile_options(-fsanitize=thread -fno-omit-frame-pointer -g)
  add_link_options(-fsanitize=thread)
endif ()

if (USE_CLANG_TIDY)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif ()

# Libraries
cs_add_library(${PROJECT_NAME} src/tf_transformer.cc src/wavemap_2d_server.cc)

# Binaries
add_executable(wavemap_2d_server_node app/wavemap_2d_server_node.cc)
target_link_libraries(wavemap_2d_server_node ${PROJECT_NAME})

cs_add_executable(rosbag_processor app/rosbag_processor.cc)
target_link_libraries(rosbag_processor ${PROJECT_NAME})

# Export
cs_install()
cs_export()
