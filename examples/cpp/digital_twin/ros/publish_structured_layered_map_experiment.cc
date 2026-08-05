#include <iostream>
#include <map>
#include <vector>

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include "../common/example_layered_map_ros_config.h"
#include "../common/layered_ros_converter.h"

namespace {
void populateDiscreteLayers(ExampleLayeredMap& map,
                            const std::vector<wavemap::Index3D>& indices) {
  const wavemap::Index3D first_index = indices.front();
  const wavemap::Index3D middle_index = indices[indices.size() / 2u];
  const wavemap::Index3D last_index = indices.back();

  map.discreteLayers().semantic.setValue(first_index, 1);
  map.discreteLayers().semantic.setValue(middle_index, 2);
  map.discreteLayers().semantic.setValue(last_index, 1);
  map.discreteLayers().changed.setValue(first_index, false);
  map.discreteLayers().changed.setValue(middle_index, true);
  map.discreteLayers().changed.setValue(last_index, false);
}
}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "wavemap_publish_structured_layered_map_experiment");
  ros::NodeHandle nh;

  ExampleLayeredMapConfig config;
  config.continuous_map.min_cell_width = 0.1f;
  config.continuous_map.min_log_odds = -2.f;
  config.continuous_map.max_log_odds = 4.f;
  config.continuous_map.tree_height = 3;
  config.discrete_compression.block_height = 1;

  ExampleLayeredMap map(config);
  std::vector<wavemap::Index3D> populated_indices;
  populateLayeredCube(map.continuousMap(), populated_indices);
  populateDiscreteLayers(map, populated_indices);

  wavemap_msgs::LayeredMap msg;
  if (!wavemap::convert::layeredMapToRosMsg<ExampleLayeredMap,
                                           LayeredVoxelRosConverter>(
          map, "map", ros::Time::now(), msg)) {
    std::cerr << "Failed to convert LayeredMap to ROS message.\n";
    return 1;
  }

  const std::string layered_map_topic = "/wavemap/layered_map";
  const std::string continuous_map_topic = "/wavemap/map";
  const int queue_size = 1;
  const bool latch = true;
  ros::Publisher layered_map_publisher =
      nh.advertise<wavemap_msgs::LayeredMap>(layered_map_topic, queue_size, latch);
  ros::Publisher continuous_map_publisher =
      nh.advertise<wavemap_msgs::Map>(continuous_map_topic, queue_size, latch);
  std::map<std::string, ros::Publisher> marker_publishers;
  auto marker_publisher_factory = [&](const std::string& topic) -> ros::Publisher& {
    auto [publisher_it, inserted] = marker_publishers.emplace(topic, ros::Publisher{});
    if (inserted) {
      publisher_it->second =
          nh.advertise<visualization_msgs::MarkerArray>(topic, queue_size, latch);
    }
    return publisher_it->second;
  };
  ros::Duration(0.5).sleep();

  layered_map_publisher.publish(msg);
  continuous_map_publisher.publish(msg.continuous_map);
  if (!layered_map_viz::publishDiscreteLayerMarkers(
          map.discreteLayers(), msg.header.frame_id, msg.header.stamp,
          marker_publisher_factory)) {
    std::cerr << "Failed to publish discrete layer markers.\n";
    return 1;
  }
  ros::spinOnce();

  std::cout << "Published LayeredMap on: " << layered_map_topic << "\n";
  std::cout << "Published continuous map on: " << continuous_map_topic << "\n";
  for (const auto& [topic, publisher] : marker_publishers) {
    (void)publisher;
    std::cout << "Published discrete markers on: " << topic << "\n";
  }
  std::cout << "Frame: " << msg.header.frame_id << "\n";
  std::cout << "Continuous ROS map blocks: "
            << msg.continuous_map.layered_hashed_wavelet_octree.front().blocks.size()
            << "\n";
  std::cout << "Discrete ROS layers: " << msg.discrete_layers.size() << "\n";
  std::cout << "Populated continuous voxels: " << populated_indices.size() << "\n";
  std::cout << "Semantic parent cells: "
            << map.discreteLayers().semantic.parentCount() << "\n";
  std::cout << "Changed parent cells: "
            << map.discreteLayers().changed.parentCount() << "\n";

  ros::spin();
  return 0;
}
