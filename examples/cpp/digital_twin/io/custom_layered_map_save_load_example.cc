#include <filesystem>
#include <iostream>
#include <optional>
#include <vector>

#include "../common/custom_layered_map_config.h"

namespace {
template <typename ValueT>
bool sameValue(const std::optional<ValueT>& value, const ValueT& expected) {
  return value && *value == expected;
}

bool sameVoxel(const MyVoxel& lhs, const MyVoxel& rhs) {
  return lhs.occupancy == rhs.occupancy && lhs.data == rhs.data;
}
}  // namespace

int main() {
  MyLayeredMapConfig config = makeMyLayeredMapConfig();
  MyLayeredMap map(config);
  const std::vector<wavemap::Index3D> indices{
      wavemap::Index3D(1, 2, 3), wavemap::Index3D(2, 2, 3)};

  map.continuousMap().setVoxelValue(
      indices[0], MyVoxel(12.f, MyContinuousLayers{22.5f, 0.75f}));
  map.continuousMap().setVoxelValue(
      indices[1], MyVoxel(14.f, MyContinuousLayers{24.0f, 0.50f}));
  map.discreteLayers().semantic_label.setValue(indices[0], 4);
  map.discreteLayers().semantic_label.setValue(indices[1], 7);
  map.discreteLayers().manually_changed.setValue(indices[0], true);
  map.discreteLayers().manually_changed.setValue(indices[1], false);

  const std::filesystem::path output_dir = "/home/ci/data/maps";
  std::filesystem::create_directories(output_dir);
  const std::filesystem::path output_path =
      output_dir / "custom_layered_map_full.lwvmp";

  if (!MyLayeredMapIo::save(output_path, map)) {
    std::cerr << "Failed to save custom LayeredMap.\n";
    return 1;
  }

  MyLayeredMap loaded_map(config);
  if (!MyLayeredMapIo::load(output_path, loaded_map)) {
    std::cerr << "Failed to load custom LayeredMap.\n";
    return 1;
  }

  size_t matching_continuous = 0u;
  for (const wavemap::Index3D& index : indices) {
    if (sameVoxel(map.continuousMap().getVoxelValue(index),
                  loaded_map.continuousMap().getVoxelValue(index))) {
      ++matching_continuous;
    }
  }

  const bool semantic_ok =
      sameValue(loaded_map.discreteLayers().semantic_label.getValue(indices[0]),
                4) &&
      sameValue(loaded_map.discreteLayers().semantic_label.getValue(indices[1]),
                7);
  const bool changed_ok =
      sameValue(loaded_map.discreteLayers().manually_changed.getValue(indices[0]),
                true) &&
      sameValue(loaded_map.discreteLayers().manually_changed.getValue(indices[1]),
                false);
  const bool unknown_ok =
      !loaded_map.discreteLayers().semantic_label.getValue(
          wavemap::Index3D(9, 9, 9));

  std::cout << "Custom LayeredMap save/load example\n";
  std::cout << "saved file: " << output_path << "\n";
  std::cout << "continuous voxels matched: " << matching_continuous << "/"
            << indices.size() << "\n";
  std::cout << "semantic labels preserved: " << (semantic_ok ? "yes" : "no")
            << "\n";
  std::cout << "manual changed flags preserved: "
            << (changed_ok ? "yes" : "no") << "\n";
  std::cout << "unknown custom discrete voxel remains unknown: "
            << (unknown_ok ? "yes" : "no") << "\n";

  return matching_continuous == indices.size() && semantic_ok && changed_ok &&
                 unknown_ok
             ? 0
             : 1;
}
