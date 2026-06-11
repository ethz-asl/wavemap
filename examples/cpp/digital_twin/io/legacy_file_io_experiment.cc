#include <filesystem>
#include <iostream>
#include <memory>

#include <wavemap/core/map/hashed_wavelet_octree.h>
#include <wavemap/core/map/map_base.h>
#include <wavemap/io/file_conversions.h>

int main() {
  wavemap::HashedWaveletOctreeConfig config;
  config.min_cell_width = 0.1f;
  config.min_log_odds = -2.f;
  config.max_log_odds = 4.f;
  config.tree_height = 3;

  wavemap::HashedWaveletOctree map(config);

  const wavemap::Index3D voxel_index(9, 2, 3);
  map.setCellValue(voxel_index, 0.8f);
  map.addToCellValue(voxel_index, 0.2f);

  const std::filesystem::path output_path =
      "/home/ci/data/maps/legacy_hashed_wavelet_octree.wvmp";
  if (!wavemap::io::mapToFile(map, output_path)) {
    std::cerr << "Failed to save legacy map to: " << output_path << "\n";
    return 1;
  }

  wavemap::MapBase::Ptr loaded_map;
  if (!wavemap::io::fileToMap(output_path, loaded_map)) {
    std::cerr << "Failed to load legacy map from: " << output_path << "\n";
    return 1;
  }

  auto loaded_hashed_map =
      std::dynamic_pointer_cast<wavemap::HashedWaveletOctree>(loaded_map);
  if (!loaded_hashed_map) {
    std::cerr << "Loaded map is not a HashedWaveletOctree.\n";
    return 1;
  }

  const wavemap::FloatingPoint original_value = map.getCellValue(voxel_index);
  const wavemap::FloatingPoint loaded_value =
      loaded_hashed_map->getCellValue(voxel_index);

  std::cout << "Original occupancy: " << original_value << "\n";
  std::cout << "Loaded occupancy: " << loaded_value << "\n";
  std::cout << "Saved legacy map to: " << output_path << "\n";

  return original_value == loaded_value ? 0 : 1;
}
