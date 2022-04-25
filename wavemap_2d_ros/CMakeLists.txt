cmake_minimum_required(VERSION 3.0.2)
project(wavemap_2d_ros)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# Configure
set(CMAKE_CXX_FLAGS
    "${CMAKE_CXX_FLAGS} -std=c++17 \
    -Wall -Wextra -Wsuggest-attribute=const \
    -Wno-deprecated-copy -Wno-class-memaccess \
    -fPIC -DEIGEN_INITIALIZE_MATRICES_BY_NAN")

if (USE_UBSAN)
  set(CMAKE_CXX_FLAGS
      "${CMAKE_CXX_FLAGS} -fsanitize=undefined -fno-omit-frame-pointer -g \
      -fsanitize=float-divide-by-zero -fsanitize=float-cast-overflow")
endif ()
if (USE_ASAN)
  set(CMAKE_CXX_FLAGS
      "${CMAKE_CXX_FLAGS} -fsanitize=address -fno-omit-frame-pointer -g \
      -fsanitize-address-use-after-scope")
endif ()
if (USE_TSAN)
  set(CMAKE_CXX_FLAGS
      "${CMAKE_CXX_FLAGS} -fsanitize=thread -fno-omit-frame-pointer -g")
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
