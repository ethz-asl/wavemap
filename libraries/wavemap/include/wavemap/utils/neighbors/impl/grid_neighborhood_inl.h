#ifndef WAVEMAP_UTILS_NEIGHBORS_IMPL_GRID_NEIGHBORHOOD_INL_H_
#define WAVEMAP_UTILS_NEIGHBORS_IMPL_GRID_NEIGHBORHOOD_INL_H_

#include <algorithm>

namespace wavemap {
template <int dim>
constexpr int GridNeighborhood<dim>::numNeighbors(
    AdjacencyType adjacency_type) {
  return int_math::exp2(dim - static_cast<int>(adjacency_type)) *
         int_math::binomial(dim, static_cast<int>(adjacency_type));
}

template <int dim>
constexpr int GridNeighborhood<dim>::numNeighbors(
    AdjacencyMask adjacency_mask) {
  int num_neighbors = 0;
  for (int adjacency_type = 0; adjacency_type <= dim; ++adjacency_type) {
    if (bit_ops::is_bit_set(adjacency_mask, adjacency_type)) {
      num_neighbors += numNeighbors(static_cast<AdjacencyType>(adjacency_type));
    }
  }
  return num_neighbors;
}

template <int dim>
template <AdjacencyMask adjacency_mask>
std::array<Index<dim>, GridNeighborhood<dim>::numNeighbors(adjacency_mask)>
GridNeighborhood<dim>::generateIndexOffsets() {
  static_assert(dim <= 3);
  // Initialize the array
  constexpr int num_neighbors = numNeighbors(adjacency_mask);
  std::array<Index<dim>, num_neighbors> neighbor_offsets{};
  // Iterate over all possible neighbor offsets, storing the ones that match
  size_t array_idx = 0u;
  for (const Index<dim>& offset :
       Grid<dim>(-Index<dim>::Ones(), Index<dim>::Ones())) {
    // Add the offset if its adjacency type matches the adjacency mask
    if (adjacency_mask == kAdjacencyAny || isAdjacent(offset, adjacency_mask)) {
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

#endif  // WAVEMAP_UTILS_NEIGHBORS_IMPL_GRID_NEIGHBORHOOD_INL_H_
