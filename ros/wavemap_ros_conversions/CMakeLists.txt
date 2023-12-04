cmake_minimum_required(VERSION 3.5)
project(wavemap_ros_conversions)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(xmlrpcpp REQUIRED)
find_package(wavemap REQUIRED)
find_package(wavemap_msgs REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PkgConfig REQUIRED)
pkg_check_modules(glog REQUIRED libglog)

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

message(${wavemap_INCLUDE_DIRS})

include_directories(include ${rclcpp_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS} ${glog_INCLUDE_DIRS} ${wavemap_INCLUDE_DIRS})

# Add minkindr as header-only library
add_library(minkindr INTERFACE)
target_include_directories(minkindr INTERFACE ${minkindr_SOURCE_DIR}/include)

# Libraries
add_library(${PROJECT_NAME}
    src/config_conversions.cc
    src/map_msg_conversions.cc
    src/time_conversions.cc
    )
ament_target_dependencies(${PROJECT_NAME} rclcpp wavemap wavemap_msgs xmlrpcpp)
# target_include_directories(${PROJECT_NAME} ${glog_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} TracyClient minkindr ${glog_LIBRARIES})

# Tests
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_lint_auto REQUIRED)
if (BUILD_TESTING)
  ament_add_gtest(
      test_${PROJECT_NAME}
      test/src/test_map_msg_conversions.cc)
  ament_target_dependencies(test_${PROJECT_NAME} rclcpp wavemap wavemap_msgs)
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} gtest_main minkindr)
#   target_include_directories(test_${PROJECT_NAME} PUBLIC )
  ament_lint_auto_find_test_dependencies()
endif ()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
# Export
install(DIRECTORY include/
DESTINATION include)
# export()
ament_export_include_directories(include)
ament_export_dependencies(rclcpp wavemap wavemap_msgs)
ament_export_libraries(${PROJECT_NAME})

ament_package()