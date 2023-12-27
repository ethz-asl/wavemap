#ifndef WAVEMAP_UTILS_MATH_TREE_MATH_H_
#define WAVEMAP_UTILS_MATH_TREE_MATH_H_

#include "wavemap/utils/math/int_math.h"

namespace wavemap::tree_math::perfect_tree {
template <int dim>
constexpr size_t num_total_nodes(size_t tree_height) {
  return (int_math::exp2(dim * tree_height) - 1) / (int_math::exp2(dim) - 1);
}

template <int dim>
constexpr size_t num_total_nodes_fast(size_t tree_height) {
  constexpr size_t kBitWidth = 8 * sizeof(size_t);
  constexpr size_t kMaxHeight = kBitWidth / dim - 1;
  constexpr size_t kMaxNumNodes = num_total_nodes<dim>(kMaxHeight);
  DCHECK(tree_height <= kMaxHeight);
  const size_t height_difference = kMaxHeight - tree_height;
  return kMaxNumNodes >> (height_difference * dim);
}

template <int dim>
constexpr size_t num_inner_nodes(size_t tree_height) {
  DCHECK(0 < tree_height);
  return num_total_nodes<dim>(tree_height - 1);
}

template <int dim>
constexpr size_t num_leaf_nodes(size_t tree_height) {
  DCHECK(0 < tree_height);
  return int_math::exp2(dim * (tree_height - 1));
}
}  // namespace wavemap::tree_math::perfect_tree

#endif  // WAVEMAP_UTILS_MATH_TREE_MATH_H_
