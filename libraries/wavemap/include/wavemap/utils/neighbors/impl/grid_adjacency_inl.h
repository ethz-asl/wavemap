#ifndef WAVEMAP_UTILS_NEIGHBORS_IMPL_GRID_ADJACENCY_INL_H_
#define WAVEMAP_UTILS_NEIGHBORS_IMPL_GRID_ADJACENCY_INL_H_

#include "wavemap/utils/bits/bit_operations.h"

namespace wavemap {
template <int dim>
constexpr std::optional<AdjacencyType> computeAdjacencyType(
    const Index<dim>& index_1, const Index<dim>& index_2) {
  return computeAdjacencyType(index_1 - index_2);
}

template <int dim>
constexpr std::optional<AdjacencyType> computeAdjacencyType(
    const Index<dim>& index_offset) {
  static_assert(dim <= 3);
  const auto offset_abs = index_offset.cwiseAbs();
  if ((1 < offset_abs.array()).any()) {
    return std::nullopt;
  }
  const int num_offset_axes = offset_abs.sum();
  return static_cast<AdjacencyType>(dim - num_offset_axes);
}

template <int dim>
constexpr bool areAdjacent(const Index<dim>& index_1, const Index<dim>& index_2,
                           AdjacencyMask adjacency_mask) {
  return isAdjacent(index_1 - index_2, adjacency_mask);
}

template <int dim>
constexpr bool isAdjacent(const Index<dim>& index_offset,
                          AdjacencyMask adjacency_mask) {
  if (const auto adjacency = computeAdjacencyType(index_offset); adjacency) {
    return bit_ops::is_bit_set(adjacency_mask,
                               static_cast<int>(adjacency.value()));
  }
  return false;
}
}  // namespace wavemap

#endif  // WAVEMAP_UTILS_NEIGHBORS_IMPL_GRID_ADJACENCY_INL_H_
