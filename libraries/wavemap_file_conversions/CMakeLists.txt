cmake_minimum_required(VERSION 3.0.2)
project(wavemap_file_conversions)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

# Compiler definitions and options
add_wavemap_compile_definitions_and_options()

# Protobuf
add_compile_options(-Wno-suggest-attribute=const)
set(PROTO_DEFNS proto/wavemap_file_conversions/Map.proto)
set(BASE_PATH "proto")
PROTOBUF_CATKIN_GENERATE_CPP2(${BASE_PATH} PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})

# Libraries
# cmake-lint: disable=C0301
add_library(${PROJECT_NAME}
    src/map_conversions.cc src/proto_utils.cc ${PROTO_SRCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Export
cs_install()
cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include)
