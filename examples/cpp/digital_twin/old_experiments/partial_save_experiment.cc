#include <filesystem>
#include <iostream>
#include <string>

#include <ros/ros.h>
#include <wavemap/core/map/hashed_chunked_wavelet_octree.h>
#include <wavemap/core/map/hashed_wavelet_octree.h>
#include <wavemap/core/map/map_base.h>
#include <wavemap/io/file_conversions.h>
#include <wavemap_msgs/Map.h>
#include <wavemap_ros_conversions/map_msg_conversions.h>

namespace {
bool keepBlock(const wavemap::Index3D& block_index) {
  return block_index.x() <= 1 && block_index.x() >= -1 && block_index.y() <= 1 && block_index.y() >= -1 && block_index.z() <= 1 && block_index.z() >= -1;
}

template <typename MapT>
bool publishMap(const MapT& map, ros::Publisher& map_pub, const std::string& map_frame, const std::string& label) {
  wavemap_msgs::Map map_msg;
  if (!wavemap::convert::mapToRosMsg(map, map_frame, ros::Time::now(), map_msg)) {
    std::cerr << "Failed to convert " << label << " to ROS msg.\n";
    return false;
  }

  map_pub.publish(map_msg);
  ros::spinOnce();
  std::cout << "Published " << label << " on " << map_pub.getTopic() << "\n";
  return true;
}

template <typename HashedMapT>
bool savePartialMap(HashedMapT& map, const std::filesystem::path& output_path) {
  const size_t initial_block_count = map.getHashMap().size();

  map.eraseBlockIf([](const wavemap::Index3D& block_index, const auto& /*block*/) { return !keepBlock(block_index); });

  const size_t final_block_count = map.getHashMap().size();
  std::cout << "Kept " << final_block_count << " of " << initial_block_count << " blocks.\n";

  if (!wavemap::io::mapToFile(map, output_path)) {
    std::cerr << "Failed to save partial map to: " << output_path << "\n";
    return false;
  }

  std::cout << "Saved partial map to: " << output_path << "\n";
  return true;
}

template <typename HashedMapT>
int runExperiment(HashedMapT& map, ros::Publisher& map_pub, const std::string& map_frame, const std::filesystem::path& output_path, const std::string& map_type_name) {
  std::cout << "Loaded map type: " << map_type_name << "\n";
  std::cout << "Original block count: " << map.getHashMap().size() << "\n";

  if (!publishMap(map, map_pub, map_frame, "original map")) {
    return 1;
  }

  std::cout << "Showing original map...\n";
  ros::Duration(5.0).sleep();

  if (!savePartialMap(map, output_path)) {
    return 1;
  }

  if (!publishMap(map, map_pub, map_frame, "partial map")) {
    return 1;
  }

  ros::spin();
  return 0;
}
}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "wavemap_partial_save_experiment");
  ros::NodeHandle nh;

  const std::filesystem::path input_path = "/home/ci/data/maps/wavemap1.wvmp";
  const std::filesystem::path output_path = "/home/ci/data/maps/wavemap_blocks.wvmp";
  const std::string map_topic = "/wavemap/map";
  const std::string map_frame = "map";

  const int queue_size = 1;
  const bool latch = true;
  ros::Publisher map_pub = nh.advertise<wavemap_msgs::Map>(map_topic, queue_size, latch);
  ros::Duration(0.5).sleep();

  wavemap::MapBase::Ptr map;
  if (!wavemap::io::fileToMap(input_path, map)) {
    std::cerr << "Failed to load map from: " << input_path << "\n";
    return 1;
  }

  if (!map) {
    std::cerr << "Loaded map pointer is null.\n";
    return 1;
  }

  if (auto hashed_chunked_map = std::dynamic_pointer_cast<wavemap::HashedChunkedWaveletOctree>(map)) {
    return runExperiment(*hashed_chunked_map, map_pub, map_frame, output_path, "hashed_chunked_wavelet_octree");
  }

  if (auto hashed_map = std::dynamic_pointer_cast<wavemap::HashedWaveletOctree>(map)) {
    return runExperiment(*hashed_map, map_pub, map_frame, output_path, "hashed_wavelet_octree");
  }

  std::cerr << "Loaded map does not expose hashed block APIs.\n";
  std::cerr << "Use a hashed_wavelet_octree or hashed_chunked_wavelet_octree map.\n";
  return 1;
}
