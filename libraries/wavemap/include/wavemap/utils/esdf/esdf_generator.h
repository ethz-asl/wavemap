#ifndef WAVEMAP_UTILS_ESDF_ESDF_GENERATOR_H_
#define WAVEMAP_UTILS_ESDF_ESDF_GENERATOR_H_

#include "wavemap/data_structure/volumetric/hashed_blocks.h"
#include "wavemap/data_structure/volumetric/hashed_wavelet_octree.h"
#include "wavemap/iterator/grid_iterator.h"

namespace wavemap {
HashedBlocks generateEsdf(const HashedWaveletOctree& occupancy_layer,
                          FloatingPoint occupancy_threshold = 0.f,
                          FloatingPoint max_distance = 2.f);
}  // namespace wavemap

#endif  // WAVEMAP_UTILS_ESDF_ESDF_GENERATOR_H_
