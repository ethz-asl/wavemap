#include <cmath>
#include <iostream>
#include <optional>
#include <vector>

#include "../common/example_layered_map_ros_config.h"
#include "../common/layered_ros_converter.h"

namespace {
bool almostEqual(float lhs, float rhs) {
  return std::abs(lhs - rhs) < 1e-5f;
}

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
}  // namespace

int main() {
  ExampleLayeredMapConfig config;
  config.continuous_map.min_cell_width = 0.1f;
  config.continuous_map.min_log_odds = -2.f;
  config.continuous_map.max_log_odds = 4.f;
  config.continuous_map.tree_height = 3;
  config.discrete_compression.block_height = 1;

  ExampleLayeredMap map(config);
  std::vector<wavemap::Index3D> populated_indices;
  populateLayeredCube(map.continuousMap(), populated_indices);

  const wavemap::Index3D first_index = populated_indices.front();
  const wavemap::Index3D middle_index = populated_indices[populated_indices.size() / 2u];
  const wavemap::Index3D last_index = populated_indices.back();

  map.discreteLayers().semantic.setValue(first_index, 1);
  map.discreteLayers().semantic.setValue(middle_index, 2);
  map.discreteLayers().semantic.setValue(last_index, 1);
  map.discreteLayers().changed.setValue(first_index, false);
  map.discreteLayers().changed.setValue(middle_index, true);
  map.discreteLayers().changed.setValue(last_index, false);

  wavemap_msgs::LayeredMap msg;
  if (!wavemap::convert::layeredMapToRosMsg<ExampleLayeredMap,
                                           LayeredVoxelRosConverter>(
          map, "map", ros::Time(1, 0), msg)) {
    std::cerr << "Failed to convert LayeredMap to ROS message.\n";
    return 1;
  }

  ExampleLayeredMap loaded_map(config);
  if (!wavemap::convert::rosMsgToLayeredMap<ExampleLayeredMap,
                                           LayeredVoxelRosConverter>(
          msg, loaded_map)) {
    std::cerr << "Failed to convert ROS message back to LayeredMap.\n";
    return 1;
  }

  size_t verified_continuous = 0u;
  for (const wavemap::Index3D& voxel_index : populated_indices) {
    if (almostEqual(map.continuousMap().getVoxelValue(voxel_index),
                    loaded_map.continuousMap().getVoxelValue(voxel_index))) {
      ++verified_continuous;
    }
  }

  const bool semantic_ok =
      sameValue(loaded_map.discreteLayers().semantic.getValue(first_index), 1) &&
      sameValue(loaded_map.discreteLayers().semantic.getValue(middle_index), 2) &&
      sameValue(loaded_map.discreteLayers().semantic.getValue(last_index), 1);
  const bool changed_ok =
      sameValue(loaded_map.discreteLayers().changed.getValue(first_index), false) &&
      sameValue(loaded_map.discreteLayers().changed.getValue(middle_index), true) &&
      sameValue(loaded_map.discreteLayers().changed.getValue(last_index), false);
  const bool unknown_ok =
      !loaded_map.discreteLayers().semantic.getValue(wavemap::Index3D(0, 0, 0));

  std::cout << "LayeredMap ROS message experiment\n";
  std::cout << "frame: " << msg.header.frame_id << "\n";
  std::cout << "continuous ROS map blocks: "
            << msg.continuous_map.layered_hashed_wavelet_octree.front().blocks.size()
            << "\n";
  std::cout << "discrete ROS layers: " << msg.discrete_layers.size() << "\n";
  std::cout << "continuous voxels verified: " << verified_continuous << "/"
            << populated_indices.size() << "\n";
  std::cout << "semantic labels preserved: " << (semantic_ok ? "yes" : "no")
            << "\n";
  std::cout << "changed flags preserved: " << (changed_ok ? "yes" : "no")
            << "\n";
  std::cout << "unknown discrete voxel remains unknown: "
            << (unknown_ok ? "yes" : "no") << "\n";

  return verified_continuous == populated_indices.size() && semantic_ok &&
                 changed_ok && unknown_ok
             ? 0
             : 1;
}
