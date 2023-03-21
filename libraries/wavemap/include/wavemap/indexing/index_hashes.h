#ifndef WAVEMAP_INDEXING_INDEX_HASHES_H_
#define WAVEMAP_INDEXING_INDEX_HASHES_H_

#include <numeric>

#include "wavemap/common.h"
#include "wavemap/indexing/ndtree_index.h"

namespace wavemap {
template <int dim>
struct IndexHash {
  static constexpr auto coefficients =
      int_math::pow_sequence<size_t, 17191u, dim>();

  size_t operator()(const Index<dim>& index) const {
    return std::transform_reduce(coefficients.cbegin(), coefficients.cend(),
                                 index.data(), 0u);
  }
};

template <int dim>
struct NdtreeIndexHash {
  static constexpr auto coefficients =
      int_math::pow_sequence<size_t, 17191u, dim + 1>();

  size_t operator()(const NdtreeIndex<dim>& index) const {
    return std::transform_reduce(coefficients.cbegin(),
                                 std::prev(coefficients.cend()),
                                 index.position.data(), 0u) +
           coefficients.back() * index.height;
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_INDEXING_INDEX_HASHES_H_
