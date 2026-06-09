#include <filesystem>
#include <iostream>
#include <vector>

#include <wavemap/io/file_conversions.h>

#include "layered_voxel_config.h"

int main() {
  wavemap::HashedWaveletOctreeConfig config;
  config.min_cell_width = 0.1f;
  config.min_log_odds = -2.f;
  config.max_log_odds = 4.f;
  config.tree_height = 3;

  LayeredMap map(config);

  const wavemap::Index3D cube_origin(8, 2, 2);
  constexpr int cube_size_x = 2;
  constexpr int cube_size_y = 2;
  constexpr int cube_size_z = 4;
  size_t populated_voxels = 0u;
  std::vector<wavemap::Index3D> populated_indices;
  populated_indices.reserve(cube_size_x * cube_size_y * cube_size_z);

  // Populate a small cube of voxels with varying occupancy, color, and traversability values.
  for (int dx = 0; dx < cube_size_x; ++dx) {
    for (int dy = 0; dy < cube_size_y; ++dy) {
      for (int dz = 0; dz < cube_size_z; ++dz) {
        const wavemap::Index3D voxel_index = cube_origin + wavemap::Index3D(dx, dy, dz);
        const wavemap::FloatingPoint layer_value = static_cast<wavemap::FloatingPoint>(populated_voxels);
        const LayeredVoxel original_voxel(0.5f + 0.02f * layer_value, LayeredData{0.1f * dx, 0.1f * dy, 0.05f * dz, 0.9f - 0.03f * layer_value});
        const LayeredVoxel voxel_update(0.1f, LayeredData{0.01f, 0.02f, 0.03f, -0.01f});

        map.setVoxelValue(voxel_index, original_voxel);
        map.addToVoxelValue(voxel_index, voxel_update);
        populated_indices.emplace_back(voxel_index);
        ++populated_voxels;
      }
    }
  }

  const wavemap::Index3D sample_voxel_index = cube_origin + wavemap::Index3D(1, 1, 2);

  const std::filesystem::path output_path = "/home/ci/data/maps/layered_map.wvmp";
  if (!wavemap::io::mapToFile<LayeredVoxel, LayeredVoxelSerializer>(map, output_path)) {
    std::cerr << "Failed to save layered map to: " << output_path << "\n";
    return 1;
  }

  LayeredMap::Ptr loaded_map;
  if (!wavemap::io::fileToMap<LayeredVoxel, LayeredVoxelSerializer>(output_path, loaded_map)) {
    std::cerr << "Failed to load layered map from: " << output_path << "\n";
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
    if (original_voxel == loaded_voxel) {
      ++verified_voxels;
    } else {
      std::cerr << "Layered voxel mismatch at index " << voxel_index.transpose() << "\n";
      printVoxel("Original", original_voxel);
      printVoxel("Loaded", loaded_voxel);
    }
  }

  printVoxel("Original sample voxel", map.getVoxelValue(sample_voxel_index));
  printVoxel("Loaded sample voxel", loaded_map->getVoxelValue(sample_voxel_index));
  std::cout << "Verified layered voxels: " << verified_voxels << " / " << populated_indices.size() << "\n";
  std::cout << "Saved layered map to: " << output_path << "\n";

  return verified_voxels == populated_indices.size() ? 0 : 1;
}
