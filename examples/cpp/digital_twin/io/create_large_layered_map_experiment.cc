#include <cmath>
#include <filesystem>
#include <iostream>

#include "../common/example_layered_map_config.h"

namespace {
float normalized(float value, float max_value) {
  return max_value <= 0.f ? 0.f : value / max_value;
}

LayeredVoxel makeVoxel(int x, int y, int z, int size_x, int size_y, int size_z) {
  const float nx = normalized(static_cast<float>(x), static_cast<float>(size_x - 1));
  const float ny = normalized(static_cast<float>(y), static_cast<float>(size_y - 1));
  const float nz = normalized(static_cast<float>(z), static_cast<float>(size_z - 1));

  const float distance_from_center =
      std::sqrt((nx - 0.5f) * (nx - 0.5f) + (ny - 0.5f) * (ny - 0.5f) +
                (nz - 0.5f) * (nz - 0.5f));
  const float occupancy = 3.f - 12.f * distance_from_center;
  const Rgb color = rgb(nx, ny, 1.f - nz);
  const float traversability = std::clamp(1.f - 1.6f * distance_from_center, 0.f, 1.f);

  return makeLayeredVoxel(occupancy, color, traversability);
}

int semanticLabelFor(int x, int y, int z, int size_x, int size_y) {
  if (z > 20 && x > size_x / 4 && x < 3 * size_x / 4 &&
      y > size_y / 4 && y < 3 * size_y / 4) {
    return 2;
  }
  if ((x / 16 + y / 16) % 2 == 0) {
    return 1;
  }
  return 3;
}
}  // namespace

int main(int argc, char** argv) {
  const std::filesystem::path output_path =
      argc > 1 ? std::filesystem::path(argv[1])
               : std::filesystem::path("/home/ci/data/maps/layered_map_large.lwvmp");

  ExampleLayeredMapConfig config;
  config.continuous_map.min_cell_width = 0.25f;
  config.continuous_map.min_log_odds = -2.f;
  config.continuous_map.max_log_odds = 4.f;
  config.continuous_map.tree_height = 6;
  config.continuous_map.only_prune_blocks_if_unused_for = 5.f;
  config.continuous_pruning = makeLayeredPruningConfigWithScales(
      4.f, 1.f, 1.f, 1e-3f, 1e-3f, 1e-3f, 10.f);
  config.continuous_threshold.data.rgb_min = rgb(0.f, 0.f, 0.f);
  config.continuous_threshold.data.rgb_max = rgb(1.f, 1.f, 1.f);
  config.continuous_threshold.data.traversability_min = 0.f;
  config.continuous_threshold.data.traversability_max = 1.f;
  config.discrete_compression.block_height = 2;

  ExampleLayeredMap map(config);

  constexpr int kSizeX = 96;
  constexpr int kSizeY = 80;
  constexpr int kSizeZ = 40;
  size_t populated_continuous_voxels = 0u;
  size_t populated_discrete_voxels = 0u;

  for (int x = 0; x < kSizeX; ++x) {
    for (int y = 0; y < kSizeY; ++y) {
      for (int z = 0; z < kSizeZ; ++z) {
        const wavemap::Index3D index{x, y, z};
        const LayeredVoxel voxel = makeVoxel(x, y, z, kSizeX, kSizeY, kSizeZ);
        map.continuousMap().setVoxelValue(index, voxel);
        ++populated_continuous_voxels;

        // Keep discrete attributes spatially aligned with the occupied part of the
        // continuous map. The continuous map still stores free-space values around
        // it, but labels only exist where a semantic value would make sense.
        if (1e-3f <= voxel.occupancy) {
          map.discreteLayers().semantic.setValue(
              index, semanticLabelFor(x, y, z, kSizeX, kSizeY));
          map.discreteLayers().changed.setValue(
              index, x == y || (z % 11 == 0 && x % 9 == 0));
          ++populated_discrete_voxels;
        }
      }
    }
  }

  const size_t continuous_nodes_before = map.continuousMap().size();
  const size_t continuous_blocks_before = map.continuousMap().getHashMap().size();
  map.continuousMap().threshold();
  map.continuousMap().prune();
  const size_t continuous_nodes_after = map.continuousMap().size();
  const size_t continuous_blocks_after = map.continuousMap().getHashMap().size();

  std::filesystem::create_directories(output_path.parent_path());
  if (!ExampleLayeredMapIo::save(output_path, map)) {
    std::cerr << "Failed to save LayeredMap to " << output_path << "\n";
    return 1;
  }

  std::cout << "Created large layered map\n";
  std::cout << "output: " << output_path << "\n";
  std::cout << "populated continuous voxels: " << populated_continuous_voxels << "\n";
  std::cout << "populated discrete voxels: " << populated_discrete_voxels << "\n";
  std::cout << "continuous blocks before prune: " << continuous_blocks_before << "\n";
  std::cout << "continuous blocks after prune: " << continuous_blocks_after << "\n";
  std::cout << "continuous nodes before prune: " << continuous_nodes_before << "\n";
  std::cout << "continuous nodes after prune: " << continuous_nodes_after << "\n";
  std::cout << "semantic parents: " << map.discreteLayers().semantic.parentCount()
            << " exceptions: " << map.discreteLayers().semantic.exceptionCount()
            << " observed: " << map.discreteLayers().semantic.observedValueCount()
            << "\n";
  std::cout << "changed parents: " << map.discreteLayers().changed.parentCount()
            << " exceptions: " << map.discreteLayers().changed.exceptionCount()
            << " observed: " << map.discreteLayers().changed.observedValueCount()
            << "\n";
  std::cout << "file size bytes: " << std::filesystem::file_size(output_path) << "\n";

  return 0;
}
