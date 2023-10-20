#include "wavemap/utils/sdf/cell_neighborhoods.h"

#include "wavemap/utils/iterate/grid_iterator.h"

namespace wavemap::neighborhood {
std::array<Index3D, 26> generateNeighborIndexOffsets() {
  std::array<Index3D, 26> neighbor_offsets{};
  size_t array_idx = 0u;
  for (const Index3D& index : Grid<3>(-Index3D::Ones(), Index3D::Ones())) {
    if (index != Index3D::Zero()) {
      neighbor_offsets[array_idx] = index;
      ++array_idx;
    }
  }
  return neighbor_offsets;
}

std::array<FloatingPoint, 26> generateNeighborDistanceOffsets(
    FloatingPoint cell_width) {
  std::array<FloatingPoint, 26> distance_offsets{};
  size_t array_idx = 0u;
  for (const Index3D& index_offset : generateNeighborIndexOffsets()) {
    distance_offsets[array_idx] =
        cell_width * index_offset.cast<FloatingPoint>().norm();
    ++array_idx;
  }
  return distance_offsets;
}
}  // namespace wavemap::neighborhood
