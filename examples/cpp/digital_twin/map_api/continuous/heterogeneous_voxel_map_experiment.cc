#include <cmath>
#include <iostream>
#include <string>

#include "wavemap/core/indexing/ndtree_index.h"
#include "wavemap/core/map/cell_types/voxel_data.h"
#include "wavemap/core/map/hashed_wavelet_octree.h"

struct HeterogeneousData {
  float temperature = 0.f;
  int label = 0;
  bool dynamic = false;
  std::string name;

  bool operator==(const HeterogeneousData& other) const {
    return temperature == other.temperature && label == other.label &&
           dynamic == other.dynamic && name == other.name;
  }
};

struct HeterogeneousDataPolicy {
  static HeterogeneousData add(const HeterogeneousData& lhs,
                               const HeterogeneousData& rhs) {
    return {lhs.temperature + rhs.temperature, lhs.label + rhs.label,
            lhs.dynamic || rhs.dynamic, mergeName(lhs.name, rhs.name)};
  }

  static HeterogeneousData subtract(const HeterogeneousData& lhs,
                                    const HeterogeneousData& rhs) {
    return {lhs.temperature - rhs.temperature, lhs.label - rhs.label,
            lhs.dynamic != rhs.dynamic, keepName(lhs.name, rhs.name)};
  }

  static HeterogeneousData scale(const HeterogeneousData& data,
                                 wavemap::FloatingPoint factor) {
    return {factor * data.temperature, static_cast<int>(factor * data.label),
            data.dynamic && std::abs(factor) > 1e-6f, data.name};
  }

 private:
  static std::string mergeName(const std::string& lhs,
                               const std::string& rhs) {
    if (!lhs.empty()) {
      return lhs;
    }
    return rhs;
  }

  static std::string keepName(const std::string& lhs,
                              const std::string& rhs) {
    if (!lhs.empty()) {
      return lhs;
    }
    return rhs;
  }
};

using HeterogeneousVoxel =
    wavemap::VoxelData<HeterogeneousData, HeterogeneousDataPolicy>;
using HeterogeneousMap = wavemap::HashedWaveletOctreeT<HeterogeneousVoxel>;

void printVoxel(const std::string& name, const HeterogeneousVoxel& voxel) {
  std::cout << name << ": occ=" << voxel.occupancy
            << " temperature=" << voxel.data.temperature
            << " label=" << voxel.data.label
            << " dynamic=" << std::boolalpha << voxel.data.dynamic
            << " name=\"" << voxel.data.name << "\"\n";
}

int main() {
  wavemap::HashedWaveletOctreeConfig config;
  config.min_cell_width = 0.1f;
  config.min_log_odds = -100.f;
  config.max_log_odds = 100.f;
  config.tree_height = 3;

  HeterogeneousMap map(config);

  const wavemap::Index3D a(1, 2, 3);
  const wavemap::Index3D b(2, 2, 3);
  const wavemap::Index3D c(3, 2, 3);

  const HeterogeneousVoxel voxel_a(
      10.f, HeterogeneousData{22.5f, 1, false, "wall"});
  const HeterogeneousVoxel voxel_b(
      20.f, HeterogeneousData{24.0f, 5, true, "chair"});
  const HeterogeneousVoxel voxel_c(
      30.f, HeterogeneousData{25.5f, 9, false, "table"});

  map.setVoxelValue(a, voxel_a);
  map.setVoxelValue(b, voxel_b);
  map.setVoxelValue(c, voxel_c);

  std::cout << "Expected values\n";
  printVoxel("a expected", voxel_a);
  printVoxel("b expected", voxel_b);
  printVoxel("c expected", voxel_c);

  std::cout << "Read through wavemap/Haar\n";
  printVoxel("a read", map.getVoxelValue(a));
  printVoxel("b read", map.getVoxelValue(b));
  printVoxel("c read", map.getVoxelValue(c));
  std::cout << "block count: " << map.getHashMap().size() << "\n";
  std::cout << "node count: " << map.size() << "\n";

  size_t interesting_leaves = 0u;
  map.forEachVoxelLeaf(
      [&interesting_leaves](const wavemap::OctreeIndex& index,
                            const HeterogeneousVoxel& voxel) {
        const bool interesting =
            std::abs(voxel.occupancy) > 1e-5f ||
            std::abs(voxel.data.temperature) > 1e-5f ||
            voxel.data.label != 0 || voxel.data.dynamic ||
            !voxel.data.name.empty();
        if (interesting) {
          ++interesting_leaves;
          std::cout << "leaf h=" << index.height << " pos="
                    << index.position.transpose();
          printVoxel(" value", voxel);
        }
      });
  std::cout << "interesting leaves: " << interesting_leaves << "\n";

  map.threshold();
  map.prune();

  std::cout << "After threshold/prune\n";
  printVoxel("a read", map.getVoxelValue(a));
  printVoxel("b read", map.getVoxelValue(b));
  printVoxel("c read", map.getVoxelValue(c));
  std::cout << "block count: " << map.getHashMap().size() << "\n";
  std::cout << "node count: " << map.size() << "\n";

  return 0;
}
