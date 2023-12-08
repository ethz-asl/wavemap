#ifndef WAVEMAP_UTILS_NEIGHBORS_NDTREE_ADJACENCY_H_
#define WAVEMAP_UTILS_NEIGHBORS_NDTREE_ADJACENCY_H_

#include "wavemap/common.h"
#include "wavemap/indexing/ndtree_index.h"

namespace wavemap {
// Return true if node_1 and node_2 touch or overlap.
template <int dim>
static bool areAdjacent(const NdtreeIndex<dim>& index_1,
                        const NdtreeIndex<dim>& index_2) {
  const IndexElement height_diff = std::abs(index_1.height - index_2.height);
  const IndexElement width = int_math::exp2(height_diff);

  const NdtreeIndex<dim>& smallest_node =
      index_1.height < index_2.height ? index_1 : index_2;
  const NdtreeIndex<dim>& biggest_node =
      index_1.height < index_2.height ? index_2 : index_1;

  const Index<dim> biggest_node_min_corner =
      int_math::mult_exp2(biggest_node.position, height_diff);
  const auto min_neighbor = biggest_node_min_corner.array() - 1;
  const auto max_neighbor = biggest_node_min_corner.array() + width;

  return (min_neighbor <= smallest_node.position.array() &&
          smallest_node.position.array() <= max_neighbor)
      .all();
}
}  // namespace wavemap

#include "wavemap/utils/neighbors/impl/ndtree_adjacency_inl.h"

#endif  // WAVEMAP_UTILS_NEIGHBORS_NDTREE_ADJACENCY_H_
