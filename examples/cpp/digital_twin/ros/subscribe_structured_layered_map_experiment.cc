#include <cmath>
#include <iostream>
#include <optional>
#include <vector>

#include <ros/ros.h>

#include "../common/example_layered_map_ros_config.h"
#include "../common/layered_ros_converter.h"

namespace {
ExampleLayeredMapConfig makeConfig() {
  ExampleLayeredMapConfig config;
  config.continuous_map.min_cell_width = 0.1f;
  config.continuous_map.min_log_odds = -2.f;
  config.continuous_map.max_log_odds = 4.f;
  config.continuous_map.tree_height = 3;
  config.discrete_compression.block_height = 1;
  return config;
}

bool almostEqual(float lhs, float rhs) { return std::abs(lhs - rhs) < 1e-5f; }

bool almostEqual(const LayeredVoxel& lhs, const LayeredVoxel& rhs) {
  return almostEqual(lhs.occupancy, rhs.occupancy) &&
         almostEqual(lhs.data.rgb.r, rhs.data.rgb.r) &&
         almostEqual(lhs.data.rgb.g, rhs.data.rgb.g) &&
         almostEqual(lhs.data.rgb.b, rhs.data.rgb.b) &&
         almostEqual(lhs.data.traversability, rhs.data.traversability);
}

template <typename ValueT>
bool sameValue(const std::optional<ValueT>& value, const ValueT& expected) {
  return value && *value == expected;
}

void structuredLayeredMapCallback(const wavemap_msgs::LayeredMap::ConstPtr& msg) {
  ExampleLayeredMapConfig config = makeConfig();

  ExampleLayeredMap received_map(config);
  if (!wavemap::convert::rosMsgToLayeredMap<ExampleLayeredMap,
                                           LayeredVoxelRosConverter>(
          *msg, received_map)) {
    std::cerr << "Failed to convert ROS message to LayeredMap.\n";
    ros::shutdown();
    return;
  }

  ExampleLayeredMap expected_map(config);
  std::vector<wavemap::Index3D> populated_indices;
  populateLayeredCube(expected_map.continuousMap(), populated_indices);

  const wavemap::Index3D first_index = populated_indices.front();
  const wavemap::Index3D middle_index = populated_indices[populated_indices.size() / 2u];
  const wavemap::Index3D last_index = populated_indices.back();

  size_t verified_continuous = 0u;
  for (const wavemap::Index3D& voxel_index : populated_indices) {
    if (almostEqual(expected_map.continuousMap().getVoxelValue(voxel_index),
                    received_map.continuousMap().getVoxelValue(voxel_index))) {
      ++verified_continuous;
    }
  }

  const bool semantic_ok =
      sameValue(received_map.discreteLayers().semantic.getValue(first_index), 1) &&
      sameValue(received_map.discreteLayers().semantic.getValue(middle_index), 2) &&
      sameValue(received_map.discreteLayers().semantic.getValue(last_index), 1);
  const bool changed_ok =
      sameValue(received_map.discreteLayers().changed.getValue(first_index), false) &&
      sameValue(received_map.discreteLayers().changed.getValue(middle_index), true) &&
      sameValue(received_map.discreteLayers().changed.getValue(last_index), false);
  const bool unknown_ok =
      !received_map.discreteLayers().semantic.getValue(wavemap::Index3D(0, 0, 0));

  std::cout << "Received LayeredMap frame: " << msg->header.frame_id
            << "\n";
  std::cout << "Received LayeredMap stamp: " << msg->header.stamp
            << "\n";
  std::cout << "Continuous ROS map blocks: "
            << msg->continuous_map.layered_hashed_wavelet_octree.front().blocks.size()
            << "\n";
  std::cout << "Discrete ROS layers: " << msg->discrete_layers.size() << "\n";
  std::cout << "continuous voxels verified: " << verified_continuous << "/"
            << populated_indices.size() << "\n";
  std::cout << "semantic labels preserved: " << (semantic_ok ? "yes" : "no")
            << "\n";
  std::cout << "changed flags preserved: " << (changed_ok ? "yes" : "no")
            << "\n";
  std::cout << "unknown discrete voxel remains unknown: "
            << (unknown_ok ? "yes" : "no") << "\n";

  ros::shutdown();
}
}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "wavemap_subscribe_structured_layered_map_experiment");
  ros::NodeHandle nh;

  const std::string topic = "/wavemap/layered_map";
  const int queue_size = 1;
  ros::Subscriber subscriber =
      nh.subscribe(topic, queue_size, structuredLayeredMapCallback);

  std::cout << "Waiting for LayeredMap on: " << topic << "\n";
  ros::spin();
  return 0;
}
