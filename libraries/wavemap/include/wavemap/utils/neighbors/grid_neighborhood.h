#ifndef WAVEMAP_UTILS_NEIGHBORS_GRID_NEIGHBORHOOD_H_
#define WAVEMAP_UTILS_NEIGHBORS_GRID_NEIGHBORHOOD_H_

#include "wavemap/common.h"
#include "wavemap/utils/math/int_math.h"

namespace wavemap {
template <int dim>
struct grid_neighborhood {
  static constexpr int kNumNeighbors = int_math::pow(3, dim) - 1;

  static std::array<Index<dim>, kNumNeighbors> generateIndexOffsets();
  static std::array<FloatingPoint, kNumNeighbors> generateDistanceOffsets(
      FloatingPoint cell_width);
};
}  // namespace wavemap

#include "wavemap/utils/neighbors/impl/grid_neighborhood_inl.h"

#endif  // WAVEMAP_UTILS_NEIGHBORS_GRID_NEIGHBORHOOD_H_
