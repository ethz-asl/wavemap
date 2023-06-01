#ifndef WAVEMAP_COMMON_DATA_STRUCTURE_SPARSE_VECTOR_H_
#define WAVEMAP_COMMON_DATA_STRUCTURE_SPARSE_VECTOR_H_

#include <bitset>
#include <vector>

namespace wavemap {
template <typename T, size_t max_size, T default_value = 0>
class SparseVector {
 public:
  static constexpr uint8_t kMaxSize = max_size;
  static constexpr T kDefaultValue = default_value;

  SparseVector() = default;

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
      compacted_vector_.insert(compacted_vector_.cbegin() + sparse_idx,
                               kDefaultValue);
    }
    return compacted_vector_[sparse_idx];
  }
  const T& operator[](uint8_t idx) const { return at(idx); }

  // TODO(victorr): Add inflate (allocate densely) method
  // TODO(victorr): Add deflate (prune) method

 private:
  static constexpr std::bitset<max_size> kZeros{0};
  static const std::bitset<max_size> kOnes;
  // NOTE: Unfortunately std::bitset does not (yet) have a constexpr constructor
  //       that makes it possible to set more than the first 64 bits.

  std::bitset<max_size> bitmask_ = kZeros;
  std::vector<T> compacted_vector_;
};

template <typename T, size_t max_size, T default_value>
const std::bitset<max_size> SparseVector<T, max_size, default_value>::kOnes =
    std::bitset<max_size>{}.set();
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_DATA_STRUCTURE_SPARSE_VECTOR_H_
