#include <iostream>

#include "layered_voxel_config.h"

int main() {
  wavemap::HashedWaveletOctreeConfig config;
  config.min_cell_width = 0.1f;
  config.min_log_odds = -2.f;
  config.max_log_odds = 4.f;
  config.tree_height = 3;

  LayeredMap map(config);

  const wavemap::Index3D voxel_index(9, 2, 3);
  const LayeredVoxel initial_voxel(
      0.8f, LayeredData{1.f, 0.f, 0.f, 0.75f});

  map.setVoxelValue(voxel_index, initial_voxel);
  printVoxel("Stored voxel", map.getVoxelValue(voxel_index));

  map.addToCellValue(voxel_index, 0.2f);
  printVoxel("After occupancy update", map.getVoxelValue(voxel_index));

  const LayeredVoxel voxel_update(
      0.1f, LayeredData{0.f, 0.5f, 0.f, -0.25f});
  map.addToVoxelValue(voxel_index, voxel_update);
  printVoxel("After full voxel update", map.getVoxelValue(voxel_index));

  size_t visited_leaves = 0u;
  map.forEachVoxelLeaf(
      [&visited_leaves, &voxel_index](const wavemap::OctreeIndex& leaf_index,
                                      const LayeredVoxel& voxel) {
        ++visited_leaves;
        if (leaf_index.height == 0 && leaf_index.position == voxel_index) {
          printVoxel("Visited target leaf", voxel);
        }
      });

  std::cout << "Visited leaves: " << visited_leaves << "\n";
  std::cout << "Occupancy view: " << map.getCellValue(voxel_index) << "\n";
  std::cout << "Block count: " << map.getHashMap().size() << "\n";

  return 0;
}
