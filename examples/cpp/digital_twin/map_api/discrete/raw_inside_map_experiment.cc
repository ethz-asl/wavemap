#include <chrono>
#include <iostream>
#include <map>
#include <optional>
#include <string>
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

void printData(const std::string& name, wavemap::FloatingPoint occupancy,
               const RawDiscreteData& data) {
  std::cout << name << ": occ=" << occupancy << " label=" << data.label
            << " dynamic=" << std::boolalpha << data.dynamic
            << " code=" << data.code << "\n";
}

int main() {
  wavemap::HashedWaveletOctreeConfig config;
  config.min_cell_width = 0.1f;
  config.min_log_odds = -100.f;
  config.max_log_odds = 100.f;
  config.tree_height = 3;

  RawDiscreteLayeredMap map(config);

  const std::vector<std::tuple<wavemap::Index3D, wavemap::FloatingPoint,
                               RawDiscreteData>> samples = {
      {wavemap::Index3D(1, 2, 3), 10.f, {1, false, 'A'}},
      {wavemap::Index3D(2, 2, 3), 20.f, {5, true, 'B'}},
      {wavemap::Index3D(3, 2, 3), 30.f, {9, false, 'C'}},
  };

  const auto write_start = std::chrono::steady_clock::now();
  for (const auto& [index, occupancy, data] : samples) {
    map.setOccupancy(index, occupancy);
    map.setRawData(index, data);
  }
  const auto write_end = std::chrono::steady_clock::now();

  size_t preserved_raw_values = 0u;
  const auto read_start = std::chrono::steady_clock::now();
  for (const auto& [index, expected_occupancy, expected_data] : samples) {
    const wavemap::FloatingPoint occupancy = map.getOccupancy(index);
    const auto data = map.getRawData(index);
    if (!data) {
      std::cout << "missing raw data\n";
      continue;
    }
    printData("read", occupancy, *data);
    if (*data == expected_data) {
      ++preserved_raw_values;
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

  std::cout << "raw entries: " << map.rawDataSize() << "\n";
  std::cout << "preserved raw values: " << preserved_raw_values << "/"
            << samples.size() << "\n";
  std::cout << "occupancy blocks: " << map.occupancyBlockCount() << "\n";
  std::cout << "occupancy nodes: " << map.occupancyNodeCount() << "\n";
  std::cout << "approx raw bytes without map overhead: "
            << map.approximateRawBytesWithoutMapOverhead() << "\n";
  std::cout << "write time [us]: " << write_us << "\n";
  std::cout << "read time [us]: " << read_us << "\n";

  return preserved_raw_values == samples.size() ? 0 : 1;
}
