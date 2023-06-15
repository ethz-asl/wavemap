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
cs_add_library(${PROJECT_NAME}
    src/proto_conversions.cc
    src/file_conversions.cc
    ${PROTO_SRCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

# Tests
if (CATKIN_ENABLE_TESTING)
  catkin_add_gtest(
      test_${PROJECT_NAME}
      test/src/test_proto_conversions.cc
      test/src/test_file_conversions.cc)
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} gtest_main)
endif ()

# Export
cs_install()
cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include)
