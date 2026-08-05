#include <cmath>
#include <iostream>
#include <string>
#include <tuple>
#include <vector>

#include "wavemap/core/indexing/ndtree_index.h"
#include "wavemap/core/map/cell_types/voxel_data.h"
#include "wavemap/core/map/hashed_wavelet_octree.h"

struct CastDiscreteData {
  float label = 0.f;
  float dynamic = 0.f;
  float code = 0.f;

  bool operator==(const CastDiscreteData& other) const {
    return label == other.label && dynamic == other.dynamic &&
           code == other.code;
  }
};

struct CastDiscreteDataPolicy {
  static CastDiscreteData add(const CastDiscreteData& lhs,
                              const CastDiscreteData& rhs) {
    return {lhs.label + rhs.label, lhs.dynamic + rhs.dynamic,
            lhs.code + rhs.code};
  }

  static CastDiscreteData subtract(const CastDiscreteData& lhs,
                                   const CastDiscreteData& rhs) {
    return {lhs.label - rhs.label, lhs.dynamic - rhs.dynamic,
            lhs.code - rhs.code};
  }

  static CastDiscreteData scale(const CastDiscreteData& data,
                                wavemap::FloatingPoint factor) {
    return {factor * data.label, factor * data.dynamic, factor * data.code};
  }
};

struct RawDiscreteData {
  int label = 0;
  bool dynamic = false;
  char code = '\0';

  bool operator==(const RawDiscreteData& other) const {
    return label == other.label && dynamic == other.dynamic &&
           code == other.code;
  }
};

using CastVoxel = wavemap::VoxelData<CastDiscreteData, CastDiscreteDataPolicy>;
using CastMap = wavemap::HashedWaveletOctreeT<CastVoxel>;

CastVoxel toVoxel(wavemap::FloatingPoint occupancy,
                  const RawDiscreteData& data) {
  return CastVoxel(
      occupancy,
      {static_cast<float>(data.label), data.dynamic ? 1.f : 0.f,
       static_cast<float>(static_cast<unsigned char>(data.code))});
}

RawDiscreteData fromVoxelRounded(const CastVoxel& voxel) {
  return {static_cast<int>(std::lround(voxel.data.label)),
          std::lround(voxel.data.dynamic) != 0,
          static_cast<char>(std::lround(voxel.data.code))};
}

void printRaw(const std::string& name, const RawDiscreteData& data) {
  std::cout << name << ": label=" << data.label
            << " dynamic=" << std::boolalpha << data.dynamic
            << " code=" << data.code << "\n";
}

void printVoxel(const std::string& name, const CastVoxel& voxel) {
  std::cout << name << ": occ=" << voxel.occupancy
            << " label_f=" << voxel.data.label
            << " dynamic_f=" << voxel.data.dynamic
            << " code_f=" << voxel.data.code << " rounded=("
            << fromVoxelRounded(voxel).label << ", " << std::boolalpha
            << fromVoxelRounded(voxel).dynamic << ", "
            << fromVoxelRounded(voxel).code << ")\n";
}

int main() {
  wavemap::HashedWaveletOctreeConfig config;
  config.min_cell_width = 0.1f;
  config.min_log_odds = -100.f;
  config.max_log_odds = 100.f;
  config.tree_height = 3;

  CastMap map(config);

  const std::vector<std::tuple<wavemap::Index3D, wavemap::FloatingPoint,
                               RawDiscreteData>> samples = {
      {wavemap::Index3D(1, 2, 3), 10.f, {1, false, 'A'}},
      {wavemap::Index3D(2, 2, 3), 20.f, {5, true, 'B'}},
      {wavemap::Index3D(3, 2, 3), 30.f, {9, false, 'C'}},
  };

  for (const auto& [index, occupancy, data] : samples) {
    map.setVoxelValue(index, toVoxel(occupancy, data));
  }

  std::cout << "Expected raw values\n";
  for (const auto& [index, occupancy, data] : samples) {
    (void)index;
    (void)occupancy;
    printRaw("expected", data);
  }

  size_t preserved_after_rounding = 0u;
  std::cout << "Read through float cast + Haar\n";
  for (const auto& [index, occupancy, expected] : samples) {
    (void)occupancy;
    const CastVoxel read_voxel = map.getVoxelValue(index);
    const RawDiscreteData rounded = fromVoxelRounded(read_voxel);
    printVoxel("read", read_voxel);
    if (rounded == expected) {
      ++preserved_after_rounding;
    }
  }

  size_t interesting_leaves = 0u;
  map.forEachVoxelLeaf(
      [&interesting_leaves](const wavemap::OctreeIndex& index,
                            const CastVoxel& voxel) {
        const bool interesting = std::abs(voxel.occupancy) > 1e-5f ||
                                 std::abs(voxel.data.label) > 1e-5f ||
                                 std::abs(voxel.data.dynamic) > 1e-5f ||
                                 std::abs(voxel.data.code) > 1e-5f;
        if (interesting) {
          ++interesting_leaves;
          std::cout << "leaf h=" << index.height << " pos="
                    << index.position.transpose();
          printVoxel(" value", voxel);
        }
      });

  std::cout << "preserved after rounding: " << preserved_after_rounding << "/"
            << samples.size() << "\n";
  std::cout << "interesting leaves: " << interesting_leaves << "\n";
  std::cout << "block count: " << map.getHashMap().size() << "\n";
  std::cout << "node count: " << map.size() << "\n";

  return preserved_after_rounding == samples.size() ? 0 : 1;
}
