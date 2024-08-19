#ifndef WAVEMAP_CORE_UTILS_NEIGHBORS_IMPL_GRID_NEIGHBORHOOD_INL_H_
#define WAVEMAP_CORE_UTILS_NEIGHBORS_IMPL_GRID_NEIGHBORHOOD_INL_H_

#include <algorithm>

namespace wavemap {
template <int dim>
constexpr int GridNeighborhood<dim>::numNeighbors(Adjacency::Id adjacency) {
  const Adjacency::Mask adjacency_mask = Adjacency::toMask<dim>(adjacency);
  int num_neighbors = 0;
  for (int polygon_dim = 0; polygon_dim <= dim; ++polygon_dim) {
    if (bit_ops::is_bit_set(adjacency_mask, polygon_dim)) {
      const int num_polygons_in_boundary =
          int_math::exp2(dim - static_cast<int>(polygon_dim)) *
          int_math::binomial(dim, static_cast<int>(polygon_dim));
      num_neighbors += num_polygons_in_boundary;
    }
  }
  return num_neighbors;
}

template <int dim>
template <Adjacency::Id adjacency>
std::array<Index<dim>, GridNeighborhood<dim>::numNeighbors(adjacency)>
GridNeighborhood<dim>::generateIndexOffsets() {
  // Initialize the array
  std::array<Index<dim>, numNeighbors(adjacency)> neighbor_offsets{};
  // Iterate over all possible neighbor offsets, storing the ones that match
  size_t array_idx = 0u;
  for (const Index<dim>& offset :
       Grid<dim>(-Index<dim>::Ones(), Index<dim>::Ones())) {
    // Add the offset if its adjacency type matches the adjacency mask
    if (adjacency == Adjacency::kAny || isAdjacent(offset, adjacency)) {
      neighbor_offsets[array_idx] = offset;
      ++array_idx;
    }
  }
  // Return
  return neighbor_offsets;
}

template <int dim>
template <typename VectorT, size_t array_length>
constexpr std::array<FloatingPoint, array_length>
GridNeighborhood<dim>::computeOffsetLengths(
    const std::array<VectorT, array_length>& offsets, FloatingPoint scale) {
  std::array<FloatingPoint, array_length> distance_offsets{};
  std::transform(offsets.begin(), offsets.end(), distance_offsets.begin(),
                 [scale](const VectorT& offset) {
                   return scale * offset.template cast<FloatingPoint>().norm();
                 });
  return distance_offsets;
}
}  // namespace wavemap

#endif  // WAVEMAP_CORE_UTILS_NEIGHBORS_IMPL_GRID_NEIGHBORHOOD_INL_H_
