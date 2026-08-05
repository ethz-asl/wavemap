#include <chrono>
#include <iostream>
#include <map>
#include <tuple>
#include <vector>

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

RawDiscreteData makeData(int x, int y, int z) {
  return {1 + ((3 * x + 5 * y + 7 * z) % 11),
          ((x + y + z) % 2) == 0,
          static_cast<char>('A' + ((x + 2 * y + 3 * z) % 26))};
}

void printMismatch(const wavemap::Index3D& index,
                   const RawDiscreteData& expected,
                   const RawDiscreteData& read) {
  std::cout << "mismatch at " << index.transpose() << " expected=(label="
            << expected.label << ", dynamic=" << std::boolalpha
            << expected.dynamic << ", code=" << expected.code
            << ") read=(label=" << read.label << ", dynamic=" << read.dynamic
            << ", code=" << read.code << ")\n";
}

int main() {
  std::vector<std::pair<wavemap::Index3D, RawDiscreteData>> samples;
  samples.reserve(8 * 8 * 4);
  for (int x = 0; x < 8; ++x) {
    for (int y = 0; y < 8; ++y) {
      for (int z = 0; z < 4; ++z) {
        samples.emplace_back(wavemap::Index3D(x, y, z), makeData(x, y, z));
      }
    }
  }

  RawStorage storage;

  const auto write_start = std::chrono::steady_clock::now();
  for (const auto& [index, data] : samples) {
    storage[IndexKey(index)] = data;
  }
  const auto write_end = std::chrono::steady_clock::now();

  size_t preserved = 0u;
  size_t printed_mismatches = 0u;
  constexpr size_t kMaxPrintedMismatches = 12u;

  const auto read_start = std::chrono::steady_clock::now();
  for (const auto& [index, expected] : samples) {
    const RawDiscreteData read = storage.at(IndexKey(index));
    if (read == expected) {
      ++preserved;
    } else if (printed_mismatches < kMaxPrintedMismatches) {
      printMismatch(index, expected, read);
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

  const size_t approximate_payload_bytes =
      storage.size() * (sizeof(IndexKey) + sizeof(RawDiscreteData));

  std::cout << "samples: " << samples.size() << "\n";
  std::cout << "entries: " << storage.size() << "\n";
  std::cout << "preserved values: " << preserved << "/" << samples.size()
            << "\n";
  std::cout << "approx payload bytes without map overhead: "
            << approximate_payload_bytes << "\n";
  std::cout << "write time [us]: " << write_us << "\n";
  std::cout << "read time [us]: " << read_us << "\n";

  return preserved == samples.size() ? 0 : 1;
}
