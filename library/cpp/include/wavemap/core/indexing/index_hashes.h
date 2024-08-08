#ifndef WAVEMAP_CORE_INDEXING_INDEX_HASHES_H_
#define WAVEMAP_CORE_INDEXING_INDEX_HASHES_H_

#include <numeric>

#include "wavemap/core/common.h"
#include "wavemap/core/indexing/ndtree_index.h"

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
using Index2DHash = IndexHash<2>;
using Index3DHash = IndexHash<3>;

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
using BinaryTreeIndexHash = NdtreeIndexHash<1>;
using QuadtreeIndexHash = NdtreeIndexHash<2>;
using OctreeIndexHash = NdtreeIndexHash<3>;
}  // namespace wavemap

#endif  // WAVEMAP_CORE_INDEXING_INDEX_HASHES_H_
