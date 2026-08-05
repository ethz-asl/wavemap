#include <iostream>

#include "wavemap/core/indexing/ndtree_index.h"
#include "wavemap/core/map/hashed_wavelet_octree.h"

int main() {
  wavemap::HashedWaveletOctreeConfig config;
  config.min_cell_width = 0.1f;
  config.min_log_odds = -100.f;
  config.max_log_odds = 100.f;
  config.tree_height = 3;

  wavemap::HashedWaveletOctree map(config);

  const wavemap::Index3D a(1, 2, 3);
  const wavemap::Index3D b(2, 2, 3);
  const wavemap::Index3D c(3, 2, 3);

  map.setCellValue(a, 10.f);
  map.setCellValue(b, 20.f);
  map.setCellValue(c, 30.f);

  std::cout << "Before threshold/prune\n";
  std::cout << "a getCellValue: " << map.getCellValue(a) << "\n";
  std::cout << "b getCellValue: " << map.getCellValue(b) << "\n";
  std::cout << "c getCellValue: " << map.getCellValue(c) << "\n";
  std::cout << "block count: " << map.getHashMap().size() << "\n";
  std::cout << "node count: " << map.size() << "\n";

  size_t visited_leaves = 0u;
  map.forEachVoxelLeaf(
      [&visited_leaves](const wavemap::OctreeIndex& index,
                                      wavemap::FloatingPoint value) {
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
  std::cout << "a getCellValue: " << map.getCellValue(a) << "\n";
  std::cout << "b getCellValue: " << map.getCellValue(b) << "\n";
  std::cout << "c getCellValue: " << map.getCellValue(c) << "\n";
  std::cout << "block count: " << map.getHashMap().size() << "\n";
  std::cout << "node count: " << map.size() << "\n";

  return 0;
}
