#include <filesystem>
#include <iostream>
#include <optional>
#include <vector>

#include "../common/example_layered_map_config.h"
#include <wavemap/layered/layered_map_io.h>

namespace {
template <typename ValueT>
bool sameValue(const std::optional<ValueT>& value, const ValueT& expected) {
  return value && *value == expected;
}

bool sameVoxel(const LayeredVoxel& lhs, const LayeredVoxel& rhs) {
  return lhs == rhs;
}
}  // namespace

int main() {
  ExampleLayeredMapConfig config;
  config.continuous_map.min_cell_width = 0.1f;
  config.continuous_map.min_log_odds = -100.f;
  config.continuous_map.max_log_odds = 100.f;
  config.continuous_map.tree_height = 3;
  config.discrete_compression.block_height = 1;

  ExampleLayeredMap map(config);

  const std::vector<wavemap::Index3D> indices{
      wavemap::Index3D(0, 0, 0), wavemap::Index3D(1, 0, 0),
      wavemap::Index3D(1, 1, 1), wavemap::Index3D(4, 0, 0)};

  map.continuousMap().setVoxelValue(
      indices[0], makeLayeredVoxel(10.f, rgb(1.f, 0.f, 0.f), 0.1f));
  map.continuousMap().setVoxelValue(
      indices[1], makeLayeredVoxel(11.f, rgb(0.f, 1.f, 0.f), 0.2f));
  map.continuousMap().setVoxelValue(
      indices[2], makeLayeredVoxel(12.f, rgb(0.f, 0.f, 1.f), 0.3f));
  map.continuousMap().setVoxelValue(
      indices[3], makeLayeredVoxel(13.f, rgb(0.5f, 0.5f, 0.5f), 0.4f));

  map.discreteLayers().semantic.setValue(indices[0], 1);
  map.discreteLayers().semantic.setValue(indices[1], 1);
  map.discreteLayers().semantic.setValue(indices[2], 2);
  map.discreteLayers().semantic.setValue(indices[3], 4);

  map.discreteLayers().changed.setValue(indices[0], false);
  map.discreteLayers().changed.setValue(indices[1], false);
  map.discreteLayers().changed.setValue(indices[2], true);
  map.discreteLayers().changed.setValue(indices[3], true);

  const std::filesystem::path output_dir = "/home/ci/data/maps";
  std::filesystem::create_directories(output_dir);
  const std::filesystem::path output_path = output_dir / "layered_map_full.lwvmp";

  if (!ExampleLayeredMapIo::save(output_path, map)) {
    std::cerr << "Failed to save LayeredMap to " << output_path << "\n";
    return 1;
  }

  ExampleLayeredMap loaded_map(config);
  if (!ExampleLayeredMapIo::load(output_path, loaded_map)) {
    std::cerr << "Failed to load LayeredMap from " << output_path << "\n";
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
      sameValue(loaded_map.discreteLayers().semantic.getValue(indices[0]), 1) &&
      sameValue(loaded_map.discreteLayers().semantic.getValue(indices[1]), 1) &&
      sameValue(loaded_map.discreteLayers().semantic.getValue(indices[2]), 2) &&
      sameValue(loaded_map.discreteLayers().semantic.getValue(indices[3]), 4);
  const bool changed_ok =
      sameValue(loaded_map.discreteLayers().changed.getValue(indices[0]), false) &&
      sameValue(loaded_map.discreteLayers().changed.getValue(indices[1]), false) &&
      sameValue(loaded_map.discreteLayers().changed.getValue(indices[2]), true) &&
      sameValue(loaded_map.discreteLayers().changed.getValue(indices[3]), true);
  const bool unknown_stays_unknown =
      !loaded_map.discreteLayers().semantic.getValue(wavemap::Index3D(2, 2, 2));
  const bool stats_ok =
      loaded_map.discreteLayers().semantic.parentCount() ==
          map.discreteLayers().semantic.parentCount() &&
      loaded_map.discreteLayers().semantic.exceptionCount() ==
          map.discreteLayers().semantic.exceptionCount() &&
      loaded_map.discreteLayers().changed.parentCount() ==
          map.discreteLayers().changed.parentCount() &&
      loaded_map.discreteLayers().changed.exceptionCount() ==
          map.discreteLayers().changed.exceptionCount();

  std::cout << "LayeredMap save/load experiment\n";
  std::cout << "saved file: " << output_path << "\n";
  std::cout << "continuous voxels matched: " << matching_continuous << "/"
            << indices.size() << "\n";
  std::cout << "semantic values preserved: " << (semantic_ok ? "yes" : "no")
            << "\n";
  std::cout << "changed values preserved: " << (changed_ok ? "yes" : "no")
            << "\n";
  std::cout << "unknown discrete voxel remains unknown: "
            << (unknown_stays_unknown ? "yes" : "no") << "\n";
  std::cout << "discrete compression stats preserved: "
            << (stats_ok ? "yes" : "no") << "\n";

  return matching_continuous == indices.size() && semantic_ok && changed_ok &&
                 unknown_stays_unknown && stats_ok
             ? 0
             : 1;
}
