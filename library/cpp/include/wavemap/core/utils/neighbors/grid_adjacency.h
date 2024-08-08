#ifndef WAVEMAP_CORE_UTILS_NEIGHBORS_GRID_ADJACENCY_H_
#define WAVEMAP_CORE_UTILS_NEIGHBORS_GRID_ADJACENCY_H_

#include "wavemap/core/common.h"
#include "wavemap/core/utils/neighbors/adjacency.h"

namespace wavemap {
template <int dim>
constexpr Adjacency::Id computeAdjacency(const Index<dim>& index_1,
                                         const Index<dim>& index_2);

template <int dim>
constexpr Adjacency::Id computeAdjacency(const Index<dim>& index_offset);

template <int dim>
constexpr bool areAdjacent(const Index<dim>& index_1, const Index<dim>& index_2,
                           Adjacency::Id adjacency);

template <int dim>
constexpr bool isAdjacent(const Index<dim>& index_offset,
                          Adjacency::Id adjacency);

template <int dim>
constexpr bool areAdjacent(const Index<dim>& index_1, const Index<dim>& index_2,
                           Adjacency::Mask adjacency_mask);

template <int dim>
constexpr bool isAdjacent(const Index<dim>& index_offset,
                          Adjacency::Mask adjacency_mask);

// Forbid implicit conversions
template <int dim, typename T>
constexpr bool areAdjacent(const Index<dim>& index_1, const Index<dim>& index_2,
                           T adjacency_mask) = delete;

// Forbid implicit conversions
template <int dim, typename T>
constexpr bool isAdjacent(const Index<dim>& index_offset,
                          T adjacency_mask) = delete;
}  // namespace wavemap

#include "wavemap/core/utils/neighbors/impl/grid_adjacency_inl.h"

#endif  // WAVEMAP_CORE_UTILS_NEIGHBORS_GRID_ADJACENCY_H_
