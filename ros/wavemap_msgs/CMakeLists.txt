cmake_minimum_required(VERSION 3.5)
project(wavemap_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    "msg/HashedWaveletOctree.msg"
    "msg/HashedWaveletOctreeBlock.msg"
    "msg/Index3D.msg"
    "msg/Map.msg"
    "msg/OctreeNode.msg"
    "msg/WaveletOctree.msg"
    "msg/WaveletOctreeNode.msg"
    "srv/FilePath.srv"
    DEPENDENCIES std_msgs builtin_interfaces
)

ament_package()