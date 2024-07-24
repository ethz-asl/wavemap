cmake_minimum_required(VERSION 3.0.2)
project(wavemap_msgs)

# Load catkin and required dependencies to generate msgs
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

# Declare message files to be built
add_message_files(FILES
  HashedBlock.msg
  HashedBlocks.msg
  HashedWaveletOctree.msg
  HashedWaveletOctreeBlock.msg
  Index3D.msg
  Map.msg
  OctreeNode.msg
  WaveletOctree.msg
  WaveletOctreeNode.msg)

# Declare service files to be built
add_service_files(FILES
  FilePath.srv)

# Generate the language-specific message and service files
generate_messages(DEPENDENCIES std_msgs)

# Finish registering wavemap_msgs as a catkin pacakge
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
