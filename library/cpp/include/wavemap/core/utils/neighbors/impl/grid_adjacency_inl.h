#ifndef WAVEMAP_CORE_UTILS_NEIGHBORS_IMPL_GRID_ADJACENCY_INL_H_
#define WAVEMAP_CORE_UTILS_NEIGHBORS_IMPL_GRID_ADJACENCY_INL_H_

#include "wavemap/core/utils/bits/bit_operations.h"

namespace wavemap {
template <int dim>
constexpr Adjacency::Id computeAdjacency(const Index<dim>& index_1,
                                         const Index<dim>& index_2) {
  return computeAdjacency<dim>(index_1 - index_2);
}

template <int dim>
constexpr Adjacency::Id computeAdjacency(const Index<dim>& index_offset) {
  static_assert(dim <= 3);
  const auto offset_abs = index_offset.cwiseAbs();
  if ((1 < offset_abs.array()).any()) {
    return Adjacency::kNone;
  }
  const int num_offset_axes = offset_abs.sum();
  return Adjacency::Id{dim - num_offset_axes};
}

template <int dim>
constexpr bool areAdjacent(const Index<dim>& index_1, const Index<dim>& index_2,
                           Adjacency::Id adjacency) {
  return areAdjacent(index_1, index_2, Adjacency::toMask<dim>(adjacency));
}

template <int dim>
constexpr bool isAdjacent(const Index<dim>& index_offset,
                          Adjacency::Id adjacency) {
  return isAdjacent(index_offset, Adjacency::toMask<dim>(adjacency));
}

template <int dim>
constexpr bool areAdjacent(const Index<dim>& index_1, const Index<dim>& index_2,
                           Adjacency::Mask adjacency_mask) {
  return isAdjacent<dim>(index_1 - index_2, adjacency_mask);
}

template <int dim>
constexpr bool isAdjacent(const Index<dim>& index_offset,
                          Adjacency::Mask adjacency_mask) {
  const auto adjacency = computeAdjacency(index_offset);
  return Adjacency::toMask<dim>(adjacency) & adjacency_mask;
}
}  // namespace wavemap

#endif  // WAVEMAP_CORE_UTILS_NEIGHBORS_IMPL_GRID_ADJACENCY_INL_H_
