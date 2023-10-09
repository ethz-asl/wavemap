#ifndef WAVEMAP_UTILS_DATA_COMPARISONS_H_
#define WAVEMAP_UTILS_DATA_COMPARISONS_H_

#include <algorithm>
#include <cmath>
#include <type_traits>

namespace wavemap::data {
// Test strict equality to zero
template <typename T>
bool is_nonzero(const T& data) {
  return data != T{};
}

// Test equality to zero with a tolerance (for POD types)
template <typename DataT, typename ThresholdT,
          std::enable_if_t<std::is_arithmetic_v<DataT>, bool> = true>
bool is_nonzero(const DataT& data, ThresholdT threshold) {
  return threshold < std::abs(data);
}

// Test element-wise equality to zero with a tolerance (for array types)
template <typename DataT, typename ThresholdT,
          decltype(DataT{}.cbegin(), DataT{}.cend(), bool()) = true>
bool is_nonzero(const DataT& data, ThresholdT threshold) {
  return std::any_of(data.cbegin(), data.cend(), [threshold](auto value) {
    return threshold < std::abs(value);
  });
}
}  // namespace wavemap::data

#endif  // WAVEMAP_UTILS_DATA_COMPARISONS_H_
