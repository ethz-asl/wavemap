#include <cmath>
#include <iostream>

#include "../../common/layered_voxel_config.h"

int main() {
  wavemap::HashedWaveletOctreeConfig config;
  config.min_cell_width = 0.1f;
  config.min_log_odds = -100.f;
  config.max_log_odds = 100.f;
  config.tree_height = 3;

  ContinuousWaveletMap map(config);

  const wavemap::Index3D a(1, 2, 3);
  const wavemap::Index3D b(2, 2, 3);
  const wavemap::Index3D c(3, 2, 3);

  map.setVoxelValue(a,
                    makeLayeredVoxel(10.f, rgb(1.f, 0.f, 0.f),
                                      0.1f));
  map.setVoxelValue(b,
                    makeLayeredVoxel(20.f, rgb(0.f, 1.f, 0.f),
                                      0.5f));
  map.setVoxelValue(c,
                    makeLayeredVoxel(30.f, rgb(0.f, 1.f, 1.f),
                                      0.9f));

  std::cout << "Before threshold/prune\n";
  printVoxel("a", map.getVoxelValue(a));
  printVoxel("b", map.getVoxelValue(b));
  printVoxel("c", map.getVoxelValue(c));
  std::cout << "block count: " << map.getHashMap().size() << "\n";
  std::cout << "node count: " << map.size() << "\n";

  size_t non_zero_leaves = 0u;
  map.forEachVoxelLeaf(
      [&non_zero_leaves](const wavemap::OctreeIndex& index,
                         const LayeredVoxel& voxel) {
        const bool has_data = std::abs(voxel.occupancy) > 1e-5f ||
                              std::abs(voxel.data.traversability) > 1e-5f ||
                              std::abs(voxel.data.rgb.r) > 1e-5f ||
                              std::abs(voxel.data.rgb.g) > 1e-5f ||
                              std::abs(voxel.data.rgb.b) > 1e-5f;
        if (has_data) {
          ++non_zero_leaves;
          std::cout << "leaf h=" << index.height << " pos="
                    << index.position.transpose();
          printVoxel(" value", voxel);
        }
      });
  std::cout << "non-zero leaves: " << non_zero_leaves << "\n";

  map.threshold();
  map.prune();

  std::cout << "After threshold/prune\n";
  printVoxel("a", map.getVoxelValue(a));
  printVoxel("b", map.getVoxelValue(b));
  printVoxel("c", map.getVoxelValue(c));
  std::cout << "block count: " << map.getHashMap().size() << "\n";
  std::cout << "node count: " << map.size() << "\n";

  return 0;
}
