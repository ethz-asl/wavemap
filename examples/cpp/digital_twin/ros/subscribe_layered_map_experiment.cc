#include <iostream>

#include <ros/ros.h>
#include <wavemap_ros_conversions/map_msg_conversions.h>

#include "../common/layered_ros_converter.h"

namespace {
void layeredMapCallback(const wavemap_msgs::Map::ConstPtr& msg) {
  // Convert the standard wavemap ROS wrapper back into the custom layered C++ map type.
  LayeredMap::Ptr map;
  if (!wavemap::convert::rosMsgToMap<LayeredVoxel, LayeredVoxelRosConverter>(*msg, map)) {
    std::cerr << "Failed to convert wavemap_msgs::Map back to a layered map.\n";
    return;
  }

  if (!map) {
    std::cerr << "Converted layered map pointer is null.\n";
    return;
  }

  // This index is one of the voxels populated by publish_layered_map_experiment.
  const wavemap::Index3D sample_voxel_index = wavemap::Index3D(8, 2, 2) + wavemap::Index3D(1, 1, 2);
  const LayeredVoxel sample_voxel = map->getVoxelValue(sample_voxel_index);

  std::cout << "Received layered map frame: " << msg->header.frame_id << "\n";
  std::cout << "Received layered map stamp: " << msg->header.stamp << "\n";
  std::cout << "Block count: " << map->getHashMap().size() << "\n";
  std::cout << "Layer count: " << msg->layered_hashed_wavelet_octree.front().layer_names.size() << "\n";
  printVoxel("Received sample voxel", sample_voxel);

  // The example only needs one message to prove the subscriber path works.
  ros::shutdown();
}
}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "wavemap_subscribe_layered_map_experiment");
  ros::NodeHandle nh;

  const std::string map_topic = "/wavemap/map";
  const int queue_size = 1;

  // Subscribe to the same topic used by the existing wavemap map publisher path.
  ros::Subscriber map_sub = nh.subscribe(map_topic, queue_size, layeredMapCallback);

  std::cout << "Waiting for layered map on: " << map_topic << "\n";
  ros::spin();
  return 0;
}
