#ifndef WAVEMAP_CORE_UTILS_DATA_COMPARISONS_H_
#define WAVEMAP_CORE_UTILS_DATA_COMPARISONS_H_

#include <algorithm>
#include <cmath>
#include <type_traits>

#include "wavemap/core/common.h"

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

template <typename ComparisonOp, typename FirstType, typename SecondType>
bool EigenCwise(const FirstType& matrix_a, const SecondType& matrix_b) {
  static_assert(static_cast<Eigen::Index>(FirstType::RowsAtCompileTime) ==
                static_cast<Eigen::Index>(SecondType::RowsAtCompileTime));
  static_assert(static_cast<Eigen::Index>(FirstType::ColsAtCompileTime) ==
                static_cast<Eigen::Index>(SecondType::ColsAtCompileTime));
  if (ComparisonOp{}(matrix_a.array(), matrix_b.array()).all()) {
    return true;
  } else {
    return false;
  }
}

template <typename FirstType, typename SecondType>
bool EigenCwiseNear(const FirstType& matrix_a, const SecondType& matrix_b,
                    FloatingPoint precision = kEpsilon) {
  static_assert(FirstType::RowsAtCompileTime == SecondType::RowsAtCompileTime);
  static_assert(FirstType::ColsAtCompileTime == SecondType::ColsAtCompileTime);
  if (((matrix_a - matrix_b).array().abs() < precision).all()) {
    return true;
  } else {
    return false;
  }
}
}  // namespace wavemap::data

#endif  // WAVEMAP_CORE_UTILS_DATA_COMPARISONS_H_
