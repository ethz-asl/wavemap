#include <iostream>

#include "../../common/layered_voxel_config.h"

int main() {
  constexpr wavemap::IndexElement tree_height = 3;
  constexpr wavemap::FloatingPoint min_log_odds = -2.f;
  constexpr wavemap::FloatingPoint max_log_odds = 4.f;

  LayeredBlock block(tree_height, min_log_odds, max_log_odds);

  const wavemap::OctreeIndex voxel_index{0, wavemap::Index3D{1, 2, 3}};
  const LayeredVoxel initial_voxel(
      0.8f, ContinuousLayers{rgb(1.f, 0.f, 0.f), 0.75f});

  block.setVoxelValue(voxel_index, initial_voxel);
  printVoxel("Stored voxel", block.getVoxelValue(voxel_index));

  block.addToCellValue(voxel_index, 0.2f);
  printVoxel("After occupancy update", block.getVoxelValue(voxel_index));

  const LayeredVoxel voxel_update(
      0.1f, ContinuousLayers{rgb(0.f, 0.5f, 0.f), -0.25f});
  block.addToVoxelValue(voxel_index, voxel_update);
  printVoxel("After full voxel update", block.getVoxelValue(voxel_index));

  size_t visited_leaves = 0u;
  block.forEachVoxelLeaf(
      wavemap::Index3D::Zero(),
      [&visited_leaves](const wavemap::OctreeIndex& leaf_index,
                        const LayeredVoxel& voxel) {
        ++visited_leaves;
        if (leaf_index.height == 0 &&
            leaf_index.position == wavemap::Index3D{1, 2, 3}) {
          printVoxel("Visited target leaf", voxel);
        }
      });

  std::cout << "Visited leaves: " << visited_leaves << "\n";
  std::cout << "Occupancy view: " << block.getCellValue(voxel_index) << "\n";

  return 0;
}
