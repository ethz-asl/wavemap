#include <filesystem>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <ros/ros.h>
#include <wavemap/core/map/hashed_chunked_wavelet_octree.h>
#include <wavemap/core/map/hashed_wavelet_octree.h>
#include <wavemap/core/map/map_base.h>
#include <wavemap/io/file_conversions.h>
#include <wavemap_msgs/Map.h>
#include <wavemap_ros_conversions/map_msg_conversions.h>

namespace {
template <typename HashedMapT>
bool publishMap(const HashedMapT& map, ros::Publisher& map_pub,
                const std::string& map_frame, const std::string& label) {
  wavemap_msgs::Map map_msg;
  if (!wavemap::convert::mapToRosMsg(map, map_frame, ros::Time::now(),
                                     map_msg)) {
    std::cerr << "Failed to convert map to ROS msg for: " << label << "\n";
    return false;
  }

  map_pub.publish(map_msg);
  ros::spinOnce();
  std::cout << "Published " << label << " on " << map_pub.getTopic() << "\n";
  return true;
}

template <typename HashedMapT>
bool eraseBlocksAndPublish(HashedMapT& map, const std::string& map_type_name,
                           ros::Publisher& map_pub,
                           const std::string& map_frame) {
  std::cout << "Loaded map type: " << map_type_name << "\n";
  std::cout << "Initial block count: " << map.getHashMap().size() << "\n";

  if (!publishMap(map, map_pub, map_frame, "original map")) {
    return false;
  }

  const std::vector<wavemap::Index3D> blocks_to_erase{
      {0, 0, 0},    {1, 0, 0},   {1, 1, 0},   {0, 1, 0},
      {-1, 1, 0},   {-1, 0, 0},  {-1, -1, 0}, {0, -1, 0},
      {1, -1, 0},   {2, -1, 0},  {2, 0, 0},   {2, 1, 0},
      {2, 2, 0},    {1, 2, 0},   {0, 2, 0},   {-1, 2, 0},
      {-2, 2, 0},   {-2, 1, 0},  {-2, 0, 0},  {-2, -1, 0},
      {-2, -2, 0},  {-1, -2, 0}, {0, -2, 0},  {1, -2, 0},
      {2, -2, 0},
  };

  for (const wavemap::Index3D& block_to_erase : blocks_to_erase) {
    ros::Duration(1.0).sleep();

    std::cout << "Erasing block " << block_to_erase.transpose()
              << " | existed before erase: " << std::boolalpha
              << map.hasBlock(block_to_erase) << "\n";
    const bool erased = map.eraseBlock(block_to_erase);
    std::cout << "  erased: " << erased
              << " | remaining block count: " << map.getHashMap().size()
              << "\n";

    const std::string label =
        "map after erasing block " + std::to_string(block_to_erase.x()) + " " +
        std::to_string(block_to_erase.y()) + " " +
        std::to_string(block_to_erase.z());
    if (!publishMap(map, map_pub, map_frame, label)) {
      return false;
    }
  }

  return true;
}
}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "wavemap_blocks_experiment");
  ros::NodeHandle nh;

  const std::filesystem::path input_path = "/home/ci/data/maps/wavemap1.wvmp";
  const std::string map_topic = "/wavemap/map";
  const std::string map_frame = "map";

  const int queue_size = 1;
  const bool latch = true;
  ros::Publisher map_pub =
      nh.advertise<wavemap_msgs::Map>(map_topic, queue_size, latch);
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

  if (auto hashed_chunked_map =
          std::dynamic_pointer_cast<wavemap::HashedChunkedWaveletOctree>(
              map)) {
    return eraseBlocksAndPublish(*hashed_chunked_map,
                                 "hashed_chunked_wavelet_octree", map_pub,
                                 map_frame)
               ? 0
               : 1;
  }

  if (auto hashed_map =
          std::dynamic_pointer_cast<wavemap::HashedWaveletOctree>(map)) {
    return eraseBlocksAndPublish(*hashed_map, "hashed_wavelet_octree", map_pub,
                                 map_frame)
               ? 0
               : 1;
  }

  std::cerr << "Loaded map does not expose hashed block APIs.\n";
  std::cerr << "Use a hashed_blocks, hashed_wavelet_octree, or "
               "hashed_chunked_wavelet_octree map.\n";
  return 1;
}
