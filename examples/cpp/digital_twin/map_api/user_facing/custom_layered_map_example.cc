#include <cmath>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>

#include "../../common/custom_layered_map_config.h"

namespace {
bool almostEqual(float lhs, float rhs) { return std::abs(lhs - rhs) < 1e-5f; }

template <typename ValueT>
bool sameValue(const std::optional<ValueT>& value, const ValueT& expected) {
  return value && *value == expected;
}

void printOptionalBool(const std::optional<bool>& value) {
  if (value) {
    std::cout << (*value ? "true" : "false");
  } else {
    std::cout << "none";
  }
}
}  // namespace

int main() {
  // 1. User code starts from its own config type.
  MyLayeredMapConfig config = makeMyLayeredMapConfig();
  MyLayeredMap map(config);

  // 2. Continuous data lives inside the wavelet map together with occupancy.
  const wavemap::Index3D tree_index(1, 2, 3);
  const wavemap::Index3D road_index(2, 2, 3);
  map.continuousMap().setVoxelValue(
      tree_index, MyVoxel(/*occupancy=*/12.f,
                          MyContinuousLayers{/*temperature=*/22.5f,
                                             /*intensity=*/0.75f}));
  map.continuousMap().setVoxelValue(
      road_index, MyVoxel(/*occupancy=*/8.f,
                          MyContinuousLayers{/*temperature=*/19.0f,
                                             /*intensity=*/0.25f}));

  // 3. Discrete data is updated per voxel in the side discrete layer bundle.
  map.discreteLayers().semantic_label.setValue(tree_index, 4);
  map.discreteLayers().semantic_label.setValue(road_index, 7);
  map.discreteLayers().manually_changed.setValue(tree_index, true);
  map.discreteLayers().manually_changed.setValue(road_index, false);

  // 4. Continuous data still uses the Wavemap wavelet threshold/prune path.
  map.continuousMap().threshold();
  map.continuousMap().prune();

  const MyVoxel tree_voxel = map.continuousMap().getVoxelValue(tree_index);
  const std::optional<int> tree_label =
      map.discreteLayers().semantic_label.getValue(tree_index);
  const std::optional<bool> tree_changed =
      map.discreteLayers().manually_changed.getValue(tree_index);

  std::cout << "User-facing LayeredMap example\n";
  std::cout << "continuous tree voxel: occupancy=" << tree_voxel.occupancy
            << " temperature=" << tree_voxel.data.temperature
            << " intensity=" << tree_voxel.data.intensity << "\n";
  std::cout << "discrete tree voxel: semantic_label="
            << (tree_label ? std::to_string(*tree_label) : "none")
            << " manually_changed=";
  printOptionalBool(tree_changed);
  std::cout << "\n";

  // 5. Full LayeredMap persistence stores continuous + discrete layers.
  const std::filesystem::path output_dir = "/home/ci/data/maps";
  std::filesystem::create_directories(output_dir);
  const std::filesystem::path output_file =
      output_dir / "user_facing_layered_map_example.lwvmp";
  if (!MyLayeredMapIo::save(output_file, map)) {
    std::cerr << "Failed to save LayeredMap to " << output_file << "\n";
    return 1;
  }

  MyLayeredMap loaded_map(config);
  if (!MyLayeredMapIo::load(output_file, loaded_map)) {
    std::cerr << "Failed to load LayeredMap from " << output_file << "\n";
    return 1;
  }

  const MyVoxel loaded_tree_voxel =
      loaded_map.continuousMap().getVoxelValue(tree_index);
  const bool continuous_ok =
      almostEqual(tree_voxel.occupancy, loaded_tree_voxel.occupancy) &&
      almostEqual(tree_voxel.data.temperature,
                  loaded_tree_voxel.data.temperature) &&
      almostEqual(tree_voxel.data.intensity, loaded_tree_voxel.data.intensity);
  const bool discrete_ok =
      sameValue(loaded_map.discreteLayers().semantic_label.getValue(tree_index),
                4) &&
      sameValue(loaded_map.discreteLayers().semantic_label.getValue(road_index),
                7) &&
      sameValue(
          loaded_map.discreteLayers().manually_changed.getValue(tree_index),
          true) &&
      sameValue(
          loaded_map.discreteLayers().manually_changed.getValue(road_index),
          false);
  const bool unknown_ok =
      !loaded_map.discreteLayers().semantic_label.getValue(
          wavemap::Index3D(100, 100, 100));

  std::cout << "saved file: " << output_file << "\n";
  std::cout << "continuous values preserved after load: "
            << (continuous_ok ? "yes" : "no") << "\n";
  std::cout << "discrete values preserved after load: "
            << (discrete_ok ? "yes" : "no") << "\n";
  std::cout << "unknown discrete voxel remains unknown: "
            << (unknown_ok ? "yes" : "no") << "\n";
  std::cout << "semantic parent cells: "
            << loaded_map.discreteLayers().semantic_label.parentCount()
            << "\n";

  return continuous_ok && discrete_ok && unknown_ok ? 0 : 1;
}
