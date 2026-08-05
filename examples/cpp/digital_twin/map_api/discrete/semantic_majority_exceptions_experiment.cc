#include <algorithm>
#include <chrono>
#include <iostream>
#include <map>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "wavemap/core/indexing/ndtree_index.h"

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

struct BlockKey {
  int x = 0;
  int y = 0;
  int z = 0;

  bool operator<(const BlockKey& other) const {
    return std::tie(x, y, z) < std::tie(other.x, other.y, other.z);
  }
};

struct LocalKey {
  int x = 0;
  int y = 0;
  int z = 0;

  bool operator<(const LocalKey& other) const {
    return std::tie(x, y, z) < std::tie(other.x, other.y, other.z);
  }
};

struct ExceptionEntry {
  LocalKey local_index;
  int label = 0;
};

struct CompressedSemanticBlock {
  int majority_label = 0;
  std::vector<ExceptionEntry> exceptions;
};

class MajorityExceptionSemanticLayer {
 public:
  explicit MajorityExceptionSemanticLayer(int block_side)
      : block_side_(block_side) {}

  void build(const std::map<IndexKey, int>& raw_labels) {
    std::map<BlockKey, std::vector<std::pair<LocalKey, int>>> pending_blocks;
    for (const auto& [index_key, label] : raw_labels) {
      const wavemap::Index3D index(index_key.x, index_key.y, index_key.z);
      pending_blocks[toBlockKey(index)].push_back({toLocalKey(index), label});
    }

    blocks_.clear();
    for (const auto& [block_key, entries] : pending_blocks) {
      std::unordered_map<int, int> counts;
      for (const auto& [local_key, label] : entries) {
        (void)local_key;
        ++counts[label];
      }

      const auto majority_it = std::max_element(
          counts.begin(), counts.end(), [](const auto& lhs, const auto& rhs) {
            return lhs.second < rhs.second;
          });

      CompressedSemanticBlock block;
      block.majority_label = majority_it->first;
      for (const auto& [local_key, label] : entries) {
        if (label != block.majority_label) {
          block.exceptions.push_back({local_key, label});
        }
      }
      blocks_[block_key] = std::move(block);
    }
  }

  int getLabel(const wavemap::Index3D& index) const {
    const BlockKey block_key = toBlockKey(index);
    const auto block_it = blocks_.find(block_key);
    if (block_it == blocks_.end()) {
      return 0;
    }

    const LocalKey local_key = toLocalKey(index);
    const auto& block = block_it->second;
    for (const ExceptionEntry& exception : block.exceptions) {
      if (exception.local_index.x == local_key.x &&
          exception.local_index.y == local_key.y &&
          exception.local_index.z == local_key.z) {
        return exception.label;
      }
    }
    return block.majority_label;
  }

  size_t numBlocks() const { return blocks_.size(); }

  size_t numExceptions() const {
    size_t total = 0u;
    for (const auto& [block_key, block] : blocks_) {
      (void)block_key;
      total += block.exceptions.size();
    }
    return total;
  }

  size_t approximateBytes() const {
    return blocks_.size() * (sizeof(BlockKey) + sizeof(int)) +
           numExceptions() * sizeof(ExceptionEntry);
  }

 private:
  const int block_side_;
  std::map<BlockKey, CompressedSemanticBlock> blocks_;

  BlockKey toBlockKey(const wavemap::Index3D& index) const {
    return {floorDiv(index.x(), block_side_), floorDiv(index.y(), block_side_),
            floorDiv(index.z(), block_side_)};
  }

  LocalKey toLocalKey(const wavemap::Index3D& index) const {
    return {floorMod(index.x(), block_side_), floorMod(index.y(), block_side_),
            floorMod(index.z(), block_side_)};
  }

  static int floorDiv(int value, int divisor) {
    int quotient = value / divisor;
    const int remainder = value % divisor;
    if (remainder != 0 && ((remainder < 0) != (divisor < 0))) {
      --quotient;
    }
    return quotient;
  }

  static int floorMod(int value, int divisor) {
    const int remainder = value % divisor;
    return remainder < 0 ? remainder + divisor : remainder;
  }
};

int makeMostlyHomogeneousLabel(int x, int y, int z) {
  const int region_label = 1 + (x / 4) + 2 * (y / 4) + 4 * (z / 4);
  const bool is_exception = ((3 * x + 5 * y + 7 * z) % 17) == 0;
  if (!is_exception) {
    return region_label;
  }
  return 20 + ((x + y + z) % 5);
}

int makeNoisyLabel(int x, int y, int z) {
  return 1 + ((3 * x + 5 * y + 7 * z) % 6);
}

std::map<IndexKey, int> makeRawLabels(bool noisy) {
  std::map<IndexKey, int> labels;
  for (int x = 0; x < 8; ++x) {
    for (int y = 0; y < 8; ++y) {
      for (int z = 0; z < 4; ++z) {
        const wavemap::Index3D index(x, y, z);
        labels[IndexKey(index)] = noisy ? makeNoisyLabel(x, y, z)
                                        : makeMostlyHomogeneousLabel(x, y, z);
      }
    }
  }
  return labels;
}

void runScenario(const char* name, bool noisy) {
  const std::map<IndexKey, int> raw_labels = makeRawLabels(noisy);
  MajorityExceptionSemanticLayer layer(/*block_side=*/4);

  const auto build_start = std::chrono::steady_clock::now();
  layer.build(raw_labels);
  const auto build_end = std::chrono::steady_clock::now();

  size_t preserved = 0u;
  size_t printed_mismatches = 0u;
  constexpr size_t kMaxPrintedMismatches = 5u;

  const auto read_start = std::chrono::steady_clock::now();
  for (const auto& [index_key, expected_label] : raw_labels) {
    const wavemap::Index3D index(index_key.x, index_key.y, index_key.z);
    const int read_label = layer.getLabel(index);
    if (read_label == expected_label) {
      ++preserved;
    } else if (printed_mismatches < kMaxPrintedMismatches) {
      std::cout << "mismatch at " << index.transpose()
                << " expected=" << expected_label << " read=" << read_label
                << "\n";
      ++printed_mismatches;
    }
  }
  const auto read_end = std::chrono::steady_clock::now();

  const auto build_us =
      std::chrono::duration_cast<std::chrono::microseconds>(build_end -
                                                            build_start)
          .count();
  const auto read_us =
      std::chrono::duration_cast<std::chrono::microseconds>(read_end -
                                                            read_start)
          .count();

  const size_t raw_bytes = raw_labels.size() * (sizeof(IndexKey) + sizeof(int));
  const size_t compressed_bytes = layer.approximateBytes();

  std::cout << "Scenario: " << name << "\n";
  std::cout << "samples: " << raw_labels.size() << "\n";
  std::cout << "preserved labels: " << preserved << "/" << raw_labels.size()
            << "\n";
  std::cout << "blocks: " << layer.numBlocks() << "\n";
  std::cout << "exceptions: " << layer.numExceptions() << "\n";
  std::cout << "raw approx bytes: " << raw_bytes << "\n";
  std::cout << "majority+exceptions approx bytes: " << compressed_bytes
            << "\n";
  std::cout << "approx compression ratio vs raw: "
            << static_cast<double>(compressed_bytes) /
                   static_cast<double>(raw_bytes)
            << "\n";
  std::cout << "build time [us]: " << build_us << "\n";
  std::cout << "read time [us]: " << read_us << "\n";
}

int main() {
  runScenario("mostly homogeneous labels", false);
  std::cout << "---\n";
  runScenario("noisy labels", true);
  return 0;
}
