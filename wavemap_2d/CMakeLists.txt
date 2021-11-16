cmake_minimum_required(VERSION 3.0.2)
project(wavemap_2d)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++17 -Wall -Wextra -Wno-deprecated-copy -fPIC -DEIGEN_INITIALIZE_MATRICES_BY_NAN)

############
# PROTOBUF #
############
#set(PROTO_DEFNS proto/wavemap/OccupancyMap.proto)
#set(BASE_PATH "proto")
##set(PROTOBUF_COMPILE_PYTHON true)

#PROTOBUF_CATKIN_GENERATE_CPP2(${BASE_PATH} PROTO_SRCS PROTO_HDRS ${PROTO_DEFNS})

#############
# LIBRARIES #
#############
#cs_add_library(${PROJECT_NAME}_proto
#    ${PROTO_SRCS})

cs_add_library(${PROJECT_NAME}
    src/pointcloud_integrator.cc)
target_link_libraries(${PROJECT_NAME}) # ${PROJECT_NAME}_proto)

############
# BINARIES #
############
add_executable(wavemap_carmen_processor
    app/wavemap_carmen_processor.cc)
target_link_libraries(wavemap_carmen_processor ${PROJECT_NAME})

#########
# TESTS #
#########
#add_definitions(-DVISUALIZE_UNIT_TEST_RESULTS)

#catkin_add_gtest(test_something
#    test/test_something.cc)
#target_link_libraries(test_something ${PROJECT_NAME})

##########
# EXPORT #
##########
cs_install()
cs_export(INCLUDE_DIRS ${CATKIN_DEVEL_PREFIX}/include)
