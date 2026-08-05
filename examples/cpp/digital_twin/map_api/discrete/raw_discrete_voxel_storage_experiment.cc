#include <chrono>
#include <iostream>
#include <map>
#include <string>
#include <tuple>
#include <vector>

#include "wavemap/core/indexing/index_hashes.h"
#include "wavemap/core/indexing/ndtree_index.h"

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

using RawStorage = std::map<IndexKey, RawDiscreteData>;

void printData(const std::string& name, const RawDiscreteData& data) {
  std::cout << name << ": label=" << data.label
            << " dynamic=" << std::boolalpha << data.dynamic
            << " code=" << data.code << "\n";
}

int main() {
  const std::vector<std::pair<wavemap::Index3D, RawDiscreteData>> samples = {
      {wavemap::Index3D(1, 2, 3), {1, false, 'A'}},
      {wavemap::Index3D(2, 2, 3), {5, true, 'B'}},
      {wavemap::Index3D(3, 2, 3), {9, false, 'C'}},
  };

  RawStorage storage;

  const auto write_start = std::chrono::steady_clock::now();
  for (const auto& [index, data] : samples) {
    storage[IndexKey(index)] = data;
  }
  const auto write_end = std::chrono::steady_clock::now();

  size_t preserved_values = 0u;
  const auto read_start = std::chrono::steady_clock::now();
  for (const auto& [index, expected] : samples) {
    const RawDiscreteData read = storage.at(IndexKey(index));
    printData("read", read);
    if (read == expected) {
      ++preserved_values;
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

  const size_t approximate_payload_bytes = storage.size() *
      (sizeof(IndexKey) + sizeof(RawDiscreteData));

  std::cout << "entries: " << storage.size() << "\n";
  std::cout << "preserved values: " << preserved_values << "/"
            << samples.size() << "\n";
  std::cout << "approx payload bytes without map overhead: "
            << approximate_payload_bytes << "\n";
  std::cout << "write time [us]: " << write_us << "\n";
  std::cout << "read time [us]: " << read_us << "\n";

  return preserved_values == samples.size() ? 0 : 1;
}
