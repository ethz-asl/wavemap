#include <cmath>
#include <iostream>
#include <string>

#include "wavemap/core/indexing/ndtree_index.h"
#include "wavemap/core/map/cell_types/voxel_data.h"
#include "wavemap/core/map/hashed_wavelet_octree.h"

struct MixedData {
  float traversability = 0.f;
  float r = 0.f;
  float g = 0.f;
  float b = 0.f;
  int label = 0;

  bool operator==(const MixedData& other) const {
    return traversability == other.traversability && r == other.r &&
           g == other.g && b == other.b && label == other.label;
  }
};

struct MixedDataPolicy {
  static MixedData add(const MixedData& lhs, const MixedData& rhs) {
    return {lhs.traversability + rhs.traversability, lhs.r + rhs.r,
            lhs.g + rhs.g, lhs.b + rhs.b, lhs.label + rhs.label};
  }

  static MixedData subtract(const MixedData& lhs, const MixedData& rhs) {
    return {lhs.traversability - rhs.traversability, lhs.r - rhs.r,
            lhs.g - rhs.g, lhs.b - rhs.b, lhs.label - rhs.label};
  }

  static MixedData scale(const MixedData& data, wavemap::FloatingPoint factor) {
    return {factor * data.traversability, factor * data.r, factor * data.g,
            factor * data.b, static_cast<int>(factor * data.label)};
  }
};

using MixedVoxel = wavemap::VoxelData<MixedData, MixedDataPolicy>;
using MixedMap = wavemap::HashedWaveletOctreeT<MixedVoxel>;

void printVoxel(const std::string& name, const MixedVoxel& voxel) {
  std::cout << name << ": occ=" << voxel.occupancy
            << " trav=" << voxel.data.traversability << " rgb=("
            << voxel.data.r << ", " << voxel.data.g << ", " << voxel.data.b
            << ") label=" << voxel.data.label << "\n";
}

int main() {
  wavemap::HashedWaveletOctreeConfig config;
  config.min_cell_width = 0.1f;
  config.min_log_odds = -100.f;
  config.max_log_odds = 100.f;
  config.tree_height = 3;

  MixedMap map(config);

  const wavemap::Index3D a(1, 2, 3);
  const wavemap::Index3D b(2, 2, 3);
  const wavemap::Index3D c(3, 2, 3);

  map.setVoxelValue(a, MixedVoxel(10.f, {0.1f, 1.f, 0.f, 0.f, 1}));
  map.setVoxelValue(b, MixedVoxel(20.f, {0.5f, 0.f, 1.f, 0.f, 5}));
  map.setVoxelValue(c, MixedVoxel(30.f, {0.9f, 0.f, 0.f, 1.f, 9}));

  std::cout << "Before threshold/prune\n";
  printVoxel("a", map.getVoxelValue(a));
  printVoxel("b", map.getVoxelValue(b));
  printVoxel("c", map.getVoxelValue(c));
  std::cout << "block count: " << map.getHashMap().size() << "\n";
  std::cout << "node count: " << map.size() << "\n";

  size_t labeled_leaves = 0u;
  map.forEachVoxelLeaf(
      [&labeled_leaves](const wavemap::OctreeIndex& index,
                        const MixedVoxel& voxel) {
        if (voxel.data.label != 0) {
          ++labeled_leaves;
          std::cout << "leaf h=" << index.height << " pos="
                    << index.position.transpose();
          printVoxel(" value", voxel);
        }
      });
  std::cout << "labeled leaves: " << labeled_leaves << "\n";

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
