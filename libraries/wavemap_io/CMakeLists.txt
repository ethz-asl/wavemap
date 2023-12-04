cmake_minimum_required(VERSION 3.5)
project(wavemap_io)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(wavemap REQUIRED)
find_package(wavemap_msgs REQUIRED)
find_package(wavemap_ros_conversions REQUIRED)
message(${wavemap_ros_conversions_INCLUDE_DIRS})
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PkgConfig REQUIRED)
pkg_check_modules(glog REQUIRED libglog)
# find_package(catkin REQUIRED COMPONENTS wavemap wavemap_msgs wavemap_ros_conversions minkindr)
#catkin_simple(ALL_DEPS_REQUIRED)
# TODO(victorr): Switch to regular catkin

# Compiler definitions and options
add_wavemap_compile_definitions_and_options()

include(FetchContent)
FetchContent_Declare(
    tracy
    GIT_REPOSITORY https://github.com/wolfpld/tracy.git
    GIT_TAG v0.10
    GIT_SHALLOW TRUE
    GIT_PROGRESS TRUE
)
FetchContent_MakeAvailable(tracy)

# Add minkindr as header-only library
add_library(minkindr INTERFACE)
target_include_directories(minkindr INTERFACE ${minkindr_SOURCE_DIR}/include)

include_directories(include ${rclcpp_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})
# Libraries
# cmake-lint: disable=C0301
add_library(${PROJECT_NAME}
    src/file_conversions.cc
    src/stream_conversions.cc)
ament_target_dependencies(${PROJECT_NAME} wavemap wavemap_msgs wavemap_ros_conversions)
target_link_libraries(${PROJECT_NAME} TracyClient minkindr ${EIGEN3_INCLUDE_DIRS} ${glog_LIBRARIES})

# Tests
find_package(ament_cmake_gtest REQUIRED)
if (BUILD_TESTING)
  ament_add_gtest(
      test_${PROJECT_NAME}
      test/src/test_file_conversions.cc)
  ament_target_dependencies(test_${PROJECT_NAME} wavemap wavemap_msgs wavemap_ros_conversions)
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} TracyClient gtest_main minkindr ${EIGEN3_INCLUDE_DIRS} ${glog_LIBRARIES})
endif ()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
  )

# Export
install(DIRECTORY include/
DESTINATION include)

ament_export_include_directories(include)

ament_package()
