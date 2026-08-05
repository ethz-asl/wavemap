#include <iostream>
#include <vector>

#include <ros/ros.h>
#include <wavemap_ros_conversions/map_msg_conversions.h>

#include "../common/layered_ros_converter.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "wavemap_publish_layered_map_experiment");
  ros::NodeHandle nh;

  wavemap::HashedWaveletOctreeConfig config;
  config.min_cell_width = 0.1f;
  config.min_log_odds = -2.f;
  config.max_log_odds = 4.f;
  config.tree_height = 3;

  ContinuousWaveletMap map(config);
  std::vector<wavemap::Index3D> populated_indices;
  populateLayeredCube(map, populated_indices);

  const std::string map_topic = "/wavemap/map";
  const std::string map_frame = "map";
  const int queue_size = 1;
  const bool latch = true;

  // Publish through the same wrapper type used by the existing wavemap ROS path.
  ros::Publisher map_pub = nh.advertise<wavemap_msgs::Map>(map_topic, queue_size, latch);
  ros::Duration(0.5).sleep();

  wavemap_msgs::Map map_msg;
  if (!wavemap::convert::mapToRosMsg<LayeredVoxel, LayeredVoxelRosConverter>(map, map_frame, ros::Time::now(), map_msg)) {
    std::cerr << "Failed to convert layered map to wavemap_msgs::Map.\n";
    return 1;
  }

  // The message carries the normal ROS header plus the layered map payload in layered_hashed_wavelet_octree.
  map_pub.publish(map_msg);
  ros::spinOnce();

  std::cout << "Published layered map on: " << map_topic << "\n";
  std::cout << "Frame: " << map_frame << "\n";
  std::cout << "Populated voxels: " << populated_indices.size() << "\n";
  std::cout << "Layer count: " << map_msg.layered_hashed_wavelet_octree.front().layer_names.size() << "\n";
  std::cout << "Block count: " << map_msg.layered_hashed_wavelet_octree.front().blocks.size() << "\n";

  ros::spin();
  return 0;
}
