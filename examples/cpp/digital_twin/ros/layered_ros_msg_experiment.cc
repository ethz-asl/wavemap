#include <cmath>
#include <iostream>
#include <vector>

#include <wavemap_ros_conversions/map_msg_conversions.h>

#include "../common/layered_ros_converter.h"

namespace {
bool almostEqual(float lhs, float rhs) {
  return std::abs(lhs - rhs) < 1e-5f;
}

bool almostEqual(const LayeredVoxel& lhs, const LayeredVoxel& rhs) {
  return almostEqual(lhs.occupancy, rhs.occupancy) && almostEqual(lhs.data.r, rhs.data.r) && almostEqual(lhs.data.g, rhs.data.g) && almostEqual(lhs.data.b, rhs.data.b) && almostEqual(lhs.data.traversability, rhs.data.traversability);
}
}  // namespace

int main() {
  wavemap::HashedWaveletOctreeConfig config;
  config.min_cell_width = 0.1f;
  config.min_log_odds = -2.f;
  config.max_log_odds = 4.f;
  config.tree_height = 3;

  LayeredMap map(config);
  std::vector<wavemap::Index3D> populated_indices;
  populateLayeredCube(map, populated_indices);

  // Convert the layered C++ map into the standard wavemap_msgs::Map wrapper, matching the normal wavemap publishing path.
  wavemap_msgs::Map msg;
  if (!wavemap::convert::mapToRosMsg<LayeredVoxel, LayeredVoxelRosConverter>(map, "map", ros::Time(1, 0), msg)) {
    std::cerr << "Failed to convert layered map to wavemap_msgs::Map.\n";
    return 1;
  }

  // Convert the wrapped ROS message back into a fresh C++ map to validate the full wrapper round-trip.
  LayeredMap::Ptr loaded_map;
  if (!wavemap::convert::rosMsgToMap<LayeredVoxel, LayeredVoxelRosConverter>(msg, loaded_map)) {
    std::cerr << "Failed to convert wavemap_msgs::Map back to a layered map.\n";
    return 1;
  }

  if (!loaded_map) {
    std::cerr << "Loaded map pointer is null.\n";
    return 1;
  }

  size_t verified_voxels = 0u;
  for (const wavemap::Index3D& voxel_index : populated_indices) {
    const LayeredVoxel original_voxel = map.getVoxelValue(voxel_index);
    const LayeredVoxel loaded_voxel = loaded_map->getVoxelValue(voxel_index);
    if (almostEqual(original_voxel, loaded_voxel)) {
      ++verified_voxels;
    } else {
      std::cerr << "Layered ROS voxel mismatch at index " << voxel_index.transpose() << "\n";
      printVoxel("Original", original_voxel);
      printVoxel("Loaded", loaded_voxel);
    }
  }

  const wavemap::Index3D sample_voxel_index = wavemap::Index3D(8, 2, 2) + wavemap::Index3D(1, 1, 2);
  printVoxel("Original sample voxel", map.getVoxelValue(sample_voxel_index));
  printVoxel("Loaded sample voxel", loaded_map->getVoxelValue(sample_voxel_index));
  std::cout << "ROS wrapper frame: " << msg.header.frame_id << "\n";
  std::cout << "ROS message layers: " << msg.layered_hashed_wavelet_octree.front().layer_names.size() << "\n";
  std::cout << "ROS message blocks: " << msg.layered_hashed_wavelet_octree.front().blocks.size() << "\n";
  std::cout << "Verified layered ROS voxels: " << verified_voxels << " / " << populated_indices.size() << "\n";

  return verified_voxels == populated_indices.size() ? 0 : 1;
}
