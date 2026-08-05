#include <chrono>
#include <iostream>
#include <map>
#include <optional>
#include <tuple>
#include <vector>

#include "wavemap/core/indexing/ndtree_index.h"
#include "wavemap/core/map/hashed_wavelet_octree.h"

struct RawDiscreteData {
  int label = 0;
  bool dynamic = false;
  char code = '\0';

  bool operator==(const RawDiscreteData& other) const {
    return label == other.label && dynamic == other.dynamic &&
           code == other.code;
  }
};

struct IndexKey {
  int x = 0;
  int y = 0;
  int z = 0;

  explicit IndexKey(const wavemap::Index3D& index)
      : x(index.x()), y(index.y()), z(index.z()) {}

  bool operator<(const IndexKey& other) const {
    return std::tie(x, y, z) < std::tie(other.x, other.y, other.z);
  }
};

class RawDiscreteLayeredMap {
 public:
  explicit RawDiscreteLayeredMap(
      const wavemap::HashedWaveletOctreeConfig& config)
      : occupancy_map_(config) {}

  void setOccupancy(const wavemap::Index3D& index,
                    wavemap::FloatingPoint occupancy) {
    occupancy_map_.setCellValue(index, occupancy);
  }

  wavemap::FloatingPoint getOccupancy(const wavemap::Index3D& index) const {
    return occupancy_map_.getCellValue(index);
  }

  void setRawData(const wavemap::Index3D& index, const RawDiscreteData& data) {
    raw_data_[IndexKey(index)] = data;
  }

  std::optional<RawDiscreteData> getRawData(
      const wavemap::Index3D& index) const {
    const auto it = raw_data_.find(IndexKey(index));
    if (it == raw_data_.end()) {
      return std::nullopt;
    }
    return it->second;
  }

  size_t rawDataSize() const { return raw_data_.size(); }
  size_t occupancyBlockCount() const { return occupancy_map_.getHashMap().size(); }
  size_t occupancyNodeCount() const { return occupancy_map_.size(); }

  size_t approximateRawBytesWithoutMapOverhead() const {
    return raw_data_.size() * (sizeof(IndexKey) + sizeof(RawDiscreteData));
  }

 private:
  wavemap::HashedWaveletOctree occupancy_map_;
  std::map<IndexKey, RawDiscreteData> raw_data_;
};

RawDiscreteData makeData(int x, int y, int z) {
  return {1 + ((3 * x + 5 * y + 7 * z) % 11),
          ((x + y + z) % 2) == 0,
          static_cast<char>('A' + ((x + 2 * y + 3 * z) % 26))};
}

void printMismatch(const wavemap::Index3D& index,
                   const RawDiscreteData& expected,
                   const RawDiscreteData& read,
                   wavemap::FloatingPoint occupancy) {
  std::cout << "mismatch at " << index.transpose() << " occ=" << occupancy
            << " expected=(label=" << expected.label << ", dynamic="
            << std::boolalpha << expected.dynamic << ", code="
            << expected.code << ") read=(label=" << read.label
            << ", dynamic=" << read.dynamic << ", code=" << read.code
            << ")\n";
}

int main() {
  wavemap::HashedWaveletOctreeConfig config;
  config.min_cell_width = 0.1f;
  config.min_log_odds = -100.f;
  config.max_log_odds = 100.f;
  config.tree_height = 4;

  std::vector<std::pair<wavemap::Index3D, RawDiscreteData>> samples;
  samples.reserve(8 * 8 * 4);
  for (int x = 0; x < 8; ++x) {
    for (int y = 0; y < 8; ++y) {
      for (int z = 0; z < 4; ++z) {
        samples.emplace_back(wavemap::Index3D(x, y, z), makeData(x, y, z));
      }
    }
  }

  RawDiscreteLayeredMap map(config);

  const auto write_start = std::chrono::steady_clock::now();
  for (const auto& [index, data] : samples) {
    map.setOccupancy(index, static_cast<wavemap::FloatingPoint>(data.label));
    map.setRawData(index, data);
  }
  const auto write_end = std::chrono::steady_clock::now();

  size_t preserved_raw = 0u;
  size_t printed_mismatches = 0u;
  constexpr size_t kMaxPrintedMismatches = 12u;

  const auto read_start = std::chrono::steady_clock::now();
  for (const auto& [index, expected] : samples) {
    const wavemap::FloatingPoint occupancy = map.getOccupancy(index);
    const auto read = map.getRawData(index);
    if (read && *read == expected) {
      ++preserved_raw;
    } else if (read && printed_mismatches < kMaxPrintedMismatches) {
      printMismatch(index, expected, *read, occupancy);
      ++printed_mismatches;
    }
  }
  const auto read_end = std::chrono::steady_clock::now();

  const auto write_us =
      std::chrono::duration_cast<std::chrono::microseconds>(write_end -
                                                            write_start)
          .count();
  const auto read_us =
      std::chrono::duration_cast<std::chrono::microseconds>(read_end -
                                                            read_start)
          .count();

  std::cout << "samples: " << samples.size() << "\n";
  std::cout << "raw entries: " << map.rawDataSize() << "\n";
  std::cout << "preserved raw values: " << preserved_raw << "/"
            << samples.size() << "\n";
  std::cout << "occupancy blocks: " << map.occupancyBlockCount() << "\n";
  std::cout << "occupancy nodes: " << map.occupancyNodeCount() << "\n";
  std::cout << "approx raw bytes without map overhead: "
            << map.approximateRawBytesWithoutMapOverhead() << "\n";
  std::cout << "write time [us]: " << write_us << "\n";
  std::cout << "read time [us]: " << read_us << "\n";

  return preserved_raw == samples.size() ? 0 : 1;
}
