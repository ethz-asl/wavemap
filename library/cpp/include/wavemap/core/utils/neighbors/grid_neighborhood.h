#ifndef WAVEMAP_CORE_UTILS_NEIGHBORS_GRID_NEIGHBORHOOD_H_
#define WAVEMAP_CORE_UTILS_NEIGHBORS_GRID_NEIGHBORHOOD_H_

#include "wavemap/core/common.h"
#include "wavemap/core/utils/iterate/grid_iterator.h"
#include "wavemap/core/utils/math/int_math.h"
#include "wavemap/core/utils/neighbors/adjacency.h"
#include "wavemap/core/utils/neighbors/grid_adjacency.h"

namespace wavemap {
template <int dim>
struct GridNeighborhood {
  static constexpr int numNeighbors(Adjacency::Id adjacency);

  template <Adjacency::Id adjacency>
  static std::array<Index<dim>, GridNeighborhood<dim>::numNeighbors(adjacency)>
  generateIndexOffsets();

  template <typename VectorT, size_t array_length>
  static constexpr std::array<FloatingPoint, array_length> computeOffsetLengths(
      const std::array<VectorT, array_length>& offsets, FloatingPoint scale);
};
}  // namespace wavemap

#include "wavemap/core/utils/neighbors/impl/grid_neighborhood_inl.h"

#endif  // WAVEMAP_CORE_UTILS_NEIGHBORS_GRID_NEIGHBORHOOD_H_
