#include <iostream>

#include <wavemap/core/map/hashed_wavelet_octree.h>

int main() {
  wavemap::HashedWaveletOctreeConfig config;
  config.min_cell_width = 0.1f;
  config.min_log_odds = -2.f;
  config.max_log_odds = 4.f;
  config.tree_height = 3;

  wavemap::HashedWaveletOctree map(config);

  const wavemap::Index3D voxel_index(9, 2, 3);

  map.setCellValue(voxel_index, 0.8f);
  std::cout << "After setCellValue: " << map.getCellValue(voxel_index)
            << "\n";

  map.addToCellValue(voxel_index, 0.2f);
  std::cout << "After addToCellValue: " << map.getCellValue(voxel_index)
            << "\n";

  size_t visited_leaves = 0u;
  map.forEachLeaf(
      [&visited_leaves, &voxel_index](const wavemap::OctreeIndex& leaf_index,
                                      wavemap::FloatingPoint occupancy) {
        ++visited_leaves;
        if (leaf_index.height == 0 && leaf_index.position == voxel_index) {
          std::cout << "Visited target leaf occupancy: " << occupancy << "\n";
        }
      });

  std::cout << "Visited leaves: " << visited_leaves << "\n";
  std::cout << "Block count: " << map.getHashMap().size() << "\n";

  return 0;
}
