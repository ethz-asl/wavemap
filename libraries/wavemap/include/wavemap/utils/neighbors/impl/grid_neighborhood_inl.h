#ifndef WAVEMAP_UTILS_NEIGHBORS_IMPL_GRID_NEIGHBORHOOD_INL_H_
#define WAVEMAP_UTILS_NEIGHBORS_IMPL_GRID_NEIGHBORHOOD_INL_H_

#include "wavemap/utils/iterate/grid_iterator.h"

namespace wavemap {
template <int dim>
std::array<Index<dim>, grid_neighborhood<dim>::kNumNeighbors>
grid_neighborhood<dim>::generateIndexOffsets() {
  std::array<Index<dim>, kNumNeighbors> neighbor_offsets{};
  size_t array_idx = 0u;
  for (const Index<dim>& index :
       Grid<dim>(-Index<dim>::Ones(), Index<dim>::Ones())) {
    if (index != Index<dim>::Zero()) {
      neighbor_offsets[array_idx] = index;
      ++array_idx;
    }
  }
  return neighbor_offsets;
}

template <int dim>
std::array<FloatingPoint, grid_neighborhood<dim>::kNumNeighbors>
grid_neighborhood<dim>::generateDistanceOffsets(FloatingPoint cell_width) {
  std::array<FloatingPoint, kNumNeighbors> distance_offsets{};
  size_t array_idx = 0u;
  for (const Index<dim>& index_offset : generateIndexOffsets()) {
    distance_offsets[array_idx] =
        cell_width * index_offset.template cast<FloatingPoint>().norm();
    ++array_idx;
  }
  return distance_offsets;
}
}  // namespace wavemap

#endif  // WAVEMAP_UTILS_NEIGHBORS_IMPL_GRID_NEIGHBORHOOD_INL_H_
