#ifndef WAVEMAP_CORE_UTILS_DATA_COMPARISONS_H_
#define WAVEMAP_CORE_UTILS_DATA_COMPARISONS_H_

#include <algorithm>
#include <cmath>
#include <type_traits>

#include "wavemap/core/common.h"
#include "wavemap/core/map/cell_types/cell_data_traits.h"

namespace wavemap::data {
// Test strict equality to zero.
template <typename T>
bool is_nonzero(const T& data) {
  return data != T{};
}

// Test equality to zero with a tolerance for scalar arithmetic types.
template <typename DataT, typename ThresholdT,
          std::enable_if_t<std::is_arithmetic_v<DataT>, bool> = true>
bool is_nonzero(const DataT& data, ThresholdT threshold) {
  return threshold < std::abs(data);
}

// Test element-wise equality to zero with the element type's default pruning
// config. Code paths that need custom thresholds should call CellDataTraits
// directly with an explicit PruningConfig.
template <typename DataT, typename ThresholdT,
          decltype(DataT{}.cbegin(), DataT{}.cend(), bool()) = true>
bool is_nonzero(const DataT& data, ThresholdT /*threshold*/) {
  using ElementT = std::decay_t<decltype(*data.cbegin())>;
  const typename CellDataTraits<ElementT>::PruningConfig pruning_config{};
  return std::any_of(data.cbegin(), data.cend(), [&pruning_config](const auto& value) {
    return CellDataTraits<ElementT>::isNonzero(value, pruning_config);
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
