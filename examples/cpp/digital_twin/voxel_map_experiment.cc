#include <iostream>

#include <wavemap/core/map/hashed_wavelet_octree.h>
#include <wavemap/core/map/cell_types/voxel_data.h>

namespace {
struct DigitalTwinData {
  float r = 0.f;
  float g = 0.f;
  float b = 0.f;
  float traversability = 0.f;

  bool operator==(const DigitalTwinData& other) const {
    return r == other.r && g == other.g && b == other.b &&
           traversability == other.traversability;
  }
};

struct DigitalTwinDataPolicy {
  static DigitalTwinData add(const DigitalTwinData& lhs, const DigitalTwinData& rhs) {
    return {lhs.r + rhs.r, lhs.g + rhs.g, lhs.b + rhs.b,
            lhs.traversability + rhs.traversability};
  }

  static DigitalTwinData subtract(const DigitalTwinData& lhs,
                                  const DigitalTwinData& rhs) {
    return {lhs.r - rhs.r, lhs.g - rhs.g, lhs.b - rhs.b,
            lhs.traversability - rhs.traversability};
  }

  static DigitalTwinData scale(const DigitalTwinData& data,
                               wavemap::FloatingPoint factor) {
    return {factor * data.r, factor * data.g, factor * data.b,
            factor * data.traversability};
  }
};

using DigitalTwinVoxel = wavemap::VoxelData<DigitalTwinData, DigitalTwinDataPolicy>;
using DigitalTwinMap = wavemap::HashedWaveletOctreeT<DigitalTwinVoxel>;

void printVoxel(const char* label, const DigitalTwinVoxel& voxel) {
  std::cout << label << " occupancy: " << voxel.occupancy << "\n";
  std::cout << label << " color: " << voxel.data.r << " " << voxel.data.g
            << " " << voxel.data.b << "\n";
  std::cout << label << " traversability: "
            << voxel.data.traversability << "\n";
}
}  // namespace

int main() {
  wavemap::HashedWaveletOctreeConfig config;
  config.min_cell_width = 0.1f;
  config.min_log_odds = -2.f;
  config.max_log_odds = 4.f;
  config.tree_height = 3;

  DigitalTwinMap map(config);

  const wavemap::Index3D voxel_index(9, 2, 3);
  const DigitalTwinVoxel initial_voxel(0.8f, DigitalTwinData{1.f, 0.f, 0.f, 0.75f});

  map.setVoxelValue(voxel_index, initial_voxel);
  printVoxel("Stored voxel", map.getVoxelValue(voxel_index));

  map.addToCellValue(voxel_index, 0.2f);
  printVoxel("After occupancy update", map.getVoxelValue(voxel_index));

  const DigitalTwinVoxel voxel_update(0.1f, DigitalTwinData{0.f, 0.5f, 0.f, -0.25f});
  map.addToVoxelValue(voxel_index, voxel_update);
  printVoxel("After full voxel update", map.getVoxelValue(voxel_index));

  size_t visited_leaves = 0u;
  map.forEachVoxelLeaf([&visited_leaves, &voxel_index](const wavemap::OctreeIndex& leaf_index, const DigitalTwinVoxel& voxel) {
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
