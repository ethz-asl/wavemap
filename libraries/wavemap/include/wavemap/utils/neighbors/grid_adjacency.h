#ifndef WAVEMAP_UTILS_NEIGHBORS_GRID_ADJACENCY_H_
#define WAVEMAP_UTILS_NEIGHBORS_GRID_ADJACENCY_H_

#include "wavemap/common.h"
#include "wavemap/utils/neighbors/adjacency.h"

namespace wavemap {
template <int dim>
constexpr std::optional<AdjacencyType> computeAdjacencyType(
    const Index<dim>& index_1, const Index<dim>& index_2);

template <int dim>
constexpr std::optional<AdjacencyType> computeAdjacencyType(
    const Index<dim>& index_offset);

template <int dim>
constexpr bool areAdjacent(const Index<dim>& index_1, const Index<dim>& index_2,
                           AdjacencyMask adjacency_mask);

template <int dim>
constexpr bool isAdjacent(const Index<dim>& index_offset,
                          AdjacencyMask adjacency_mask);
}  // namespace wavemap

#include "wavemap/utils/neighbors/impl/grid_adjacency_inl.h"

#endif  // WAVEMAP_UTILS_NEIGHBORS_GRID_ADJACENCY_H_
