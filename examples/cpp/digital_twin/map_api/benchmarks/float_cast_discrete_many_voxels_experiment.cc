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

RawDiscreteData makeData(int x, int y, int z) {
  return {1 + ((3 * x + 5 * y + 7 * z) % 11),
          ((x + y + z) % 2) == 0,
          static_cast<char>('A' + ((x + 2 * y + 3 * z) % 26))};
}

void printMismatch(const wavemap::Index3D& index,
                   const RawDiscreteData& expected,
                   const CastVoxel& read_voxel,
                   const RawDiscreteData& rounded) {
  std::cout << "mismatch at " << index.transpose() << " expected=(label="
            << expected.label << ", dynamic=" << std::boolalpha
            << expected.dynamic << ", code=" << expected.code
            << ") read_f=(label=" << read_voxel.data.label
            << ", dynamic=" << read_voxel.data.dynamic
            << ", code=" << read_voxel.data.code << ") rounded=(label="
            << rounded.label << ", dynamic=" << rounded.dynamic
            << ", code=" << rounded.code << ")\n";
}

int main() {
  wavemap::HashedWaveletOctreeConfig config;
  config.min_cell_width = 0.1f;
  config.min_log_odds = -100.f;
  config.max_log_odds = 100.f;
  config.tree_height = 4;

  CastMap map(config);

  std::vector<std::pair<wavemap::Index3D, RawDiscreteData>> samples;
  samples.reserve(8 * 8 * 4);
  for (int x = 0; x < 8; ++x) {
    for (int y = 0; y < 8; ++y) {
      for (int z = 0; z < 4; ++z) {
        samples.emplace_back(wavemap::Index3D(x, y, z), makeData(x, y, z));
      }
    }
  }

  for (const auto& [index, data] : samples) {
    const wavemap::FloatingPoint occupancy =
        static_cast<wavemap::FloatingPoint>(data.label);
    map.setVoxelValue(index, toVoxel(occupancy, data));
  }

  size_t preserved = 0u;
  size_t label_errors = 0u;
  size_t dynamic_errors = 0u;
  size_t code_errors = 0u;
  constexpr size_t kMaxPrintedMismatches = 12u;
  size_t printed_mismatches = 0u;

  for (const auto& [index, expected] : samples) {
    const CastVoxel read_voxel = map.getVoxelValue(index);
    const RawDiscreteData rounded = fromVoxelRounded(read_voxel);
    if (rounded == expected) {
      ++preserved;
      continue;
    }

    if (rounded.label != expected.label) {
      ++label_errors;
    }
    if (rounded.dynamic != expected.dynamic) {
      ++dynamic_errors;
    }
    if (rounded.code != expected.code) {
      ++code_errors;
    }
    if (printed_mismatches < kMaxPrintedMismatches) {
      printMismatch(index, expected, read_voxel, rounded);
      ++printed_mismatches;
    }
  }

  size_t interesting_leaves = 0u;
  map.forEachVoxelLeaf(
      [&interesting_leaves](const wavemap::OctreeIndex& /*index*/,
                            const CastVoxel& voxel) {
        const bool interesting = std::abs(voxel.occupancy) > 1e-5f ||
                                 std::abs(voxel.data.label) > 1e-5f ||
                                 std::abs(voxel.data.dynamic) > 1e-5f ||
                                 std::abs(voxel.data.code) > 1e-5f;
        if (interesting) {
          ++interesting_leaves;
        }
      });

  std::cout << "samples: " << samples.size() << "\n";
  std::cout << "preserved after rounding: " << preserved << "/"
            << samples.size() << "\n";
  std::cout << "label errors: " << label_errors << "\n";
  std::cout << "dynamic errors: " << dynamic_errors << "\n";
  std::cout << "code errors: " << code_errors << "\n";
  std::cout << "interesting leaves: " << interesting_leaves << "\n";
  std::cout << "block count: " << map.getHashMap().size() << "\n";
  std::cout << "node count: " << map.size() << "\n";

  return preserved == samples.size() ? 0 : 1;
}
