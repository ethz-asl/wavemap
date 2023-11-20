cmake_minimum_required(VERSION 3.0.2)
project(wavemap_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation message_runtime std_msgs)

add_message_files(
    FILES
    HashedWaveletOctree.msg
    HashedWaveletOctreeBlock.msg
    Index3D.msg
    Map.msg
    OctreeNode.msg
    WaveletOctree.msg
    WaveletOctreeNode.msg
)

add_service_files(
  FILES
  FilePath.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS 
    message_generation
    message_runtime
    std_msgs
)

# Export
install(DIRECTORY include/${PROJECT_NAME}/
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
# export()
