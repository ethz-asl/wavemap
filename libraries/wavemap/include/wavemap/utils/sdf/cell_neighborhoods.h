#ifndef WAVEMAP_UTILS_SDF_CELL_NEIGHBORHOODS_H_
#define WAVEMAP_UTILS_SDF_CELL_NEIGHBORHOODS_H_

#include "wavemap/common.h"

namespace wavemap::neighborhood {
std::array<Index3D, 26> generateNeighborIndexOffsets();
std::array<FloatingPoint, 26> generateNeighborDistanceOffsets(
    FloatingPoint cell_width);
}  // namespace wavemap::neighborhood

#endif  // WAVEMAP_UTILS_SDF_CELL_NEIGHBORHOODS_H_
