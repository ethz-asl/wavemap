#include <iostream>

#include "wavemap/core/indexing/ndtree_index.h"
#include "wavemap/core/map/hashed_wavelet_octree.h"

int main() {
  wavemap::HashedWaveletOctreeConfig config;
  config.min_cell_width = 0.1f;
  config.min_log_odds = -100.f;
  config.max_log_odds = 100.f;
  config.tree_height = 3;

  wavemap::HashedWaveletOctreeT<int> map(config);

  const wavemap::Index3D a(1, 2, 3);
  const wavemap::Index3D b(2, 2, 3);
  const wavemap::Index3D c(3, 2, 3);

  map.setVoxelValue(a, 10);
  map.setVoxelValue(b, 20);
  map.setVoxelValue(c, 30);

  std::cout << "Before threshold/prune\n";
  std::cout << "a getVoxelValue: " << map.getVoxelValue(a) << "\n";
  std::cout << "b getVoxelValue: " << map.getVoxelValue(b) << "\n";
  std::cout << "c getVoxelValue: " << map.getVoxelValue(c) << "\n";
  std::cout << "a getCellValue: " << map.getCellValue(a) << "\n";
  std::cout << "block count: " << map.getHashMap().size() << "\n";
  std::cout << "node count: " << map.size() << "\n";

  size_t visited_leaves = 0u;
  map.forEachVoxelLeaf(
      [&visited_leaves](const wavemap::OctreeIndex& index, int value) {
        ++visited_leaves;
        if (value != 0) {
          std::cout << "leaf h=" << index.height << " pos="
                    << index.position.transpose() << " value=" << value
                    << "\n";
        }
      });
  std::cout << "visited leaves: " << visited_leaves << "\n";

  map.threshold();
  map.prune();

  std::cout << "After threshold/prune\n";
  std::cout << "a getVoxelValue: " << map.getVoxelValue(a) << "\n";
  std::cout << "b getVoxelValue: " << map.getVoxelValue(b) << "\n";
  std::cout << "c getVoxelValue: " << map.getVoxelValue(c) << "\n";
  std::cout << "block count: " << map.getHashMap().size() << "\n";
  std::cout << "node count: " << map.size() << "\n";

  return 0;
}
