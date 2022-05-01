cmake_minimum_required(VERSION 3.0.2)
project(wavemap_2d_ground_truth)

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
    -fsanitize=float-divide-by-zero
    -fsanitize=float-cast-overflow
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
cs_add_library(${PROJECT_NAME} src/occupancy_grid_creator.cc src/geometry.cc)

# Binaries
cs_add_executable(ply_converter src/user_interfaces/ply_converter.cc)
target_link_libraries(ply_converter ${PROJECT_NAME})

# Gazebo plugin
set(BUILD_GAZEBO_PLUGINS TRUE)
if (BUILD_GAZEBO_PLUGINS)
  include_directories(${GAZEBO_INCLUDE_DIRS})
  cs_add_library(${PROJECT_NAME}_plugin src/user_interfaces/gazebo_plugin.cc
                 src/geometry.cc src/occupancy_grid_creator.cc)
endif ()

# Export
cs_install()
cs_export()
