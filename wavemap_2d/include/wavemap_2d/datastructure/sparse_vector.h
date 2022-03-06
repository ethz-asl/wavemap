#ifndef WAVEMAP_2D_DATASTRUCTURE_SPARSE_VECTOR_H_
#define WAVEMAP_2D_DATASTRUCTURE_SPARSE_VECTOR_H_

#include <bitset>
#include <vector>

#include "wavemap_2d/common.h"

namespace wavemap_2d {
template <typename T, T default_value = 0>
class SparseVector {
 public:
  static constexpr uint8_t kMaxSize = 64;
  static constexpr T kDefaultValue = default_value;

  SparseVector() : bitmask_(kZeros) {}

  size_t size() { return bitmask_.count(); }
  size_t capacity() { return compacted_vector_.capacity(); }
  void shrink_to_fit() { compacted_vector_.shrink_to_fit(); }

  bool has(uint8_t idx) const { return bitmask_.test(idx); }

  const T& at(uint8_t idx) const {
    if (has(idx)) {
      const uint8_t sparse_idx = ((kOnes << idx).flip() & bitmask_).count();
      return compacted_vector_[sparse_idx];
    }
    return kDefaultValue;
  }

  T& operator[](uint8_t idx) {
    const uint8_t sparse_idx = ((kOnes << idx).flip() & bitmask_).count();
    if (!has(idx)) {
      bitmask_.set(idx);
      compacted_vector_.insert(compacted_vector_.begin() + sparse_idx,
                               kDefaultValue);
    }
    return compacted_vector_[sparse_idx];
  }
  const T& operator[](uint8_t idx) const { return at(idx); }

  // TODO(victorr): Add inflate (allocate densely) method
  // TODO(victorr): Add deflate (prune) method

 private:
  static constexpr std::bitset<kMaxSize> kZeros{0ull};
  static constexpr std::bitset<kMaxSize> kOnes{0xFFFFFFFFFFFFFFFF};

  std::bitset<kMaxSize> bitmask_;
  std::vector<T> compacted_vector_;
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_DATASTRUCTURE_SPARSE_VECTOR_H_
