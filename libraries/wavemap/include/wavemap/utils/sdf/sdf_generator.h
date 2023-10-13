#ifndef WAVEMAP_UTILS_SDF_SDF_GENERATOR_H_
#define WAVEMAP_UTILS_SDF_SDF_GENERATOR_H_

#include "wavemap/data_structure/volumetric/hashed_blocks.h"
#include "wavemap/data_structure/volumetric/hashed_wavelet_octree.h"
#include "wavemap/utils/iterate/grid_iterator.h"
#include "wavemap/utils/query/query_accelerator.h"
#include "wavemap/utils/sdf/bucket_queue.h"

namespace wavemap {
namespace neighborhood {
std::array<Index3D, 26> generateNeighborIndexOffsets();
std::array<FloatingPoint, 26> generateNeighborDistanceOffsets(
    FloatingPoint cell_width);
}  // namespace neighborhood

class QuasiEuclideanSDFGenerator {
 public:
  static constexpr FloatingPoint kMaxMultiplicativeError = 0.125f + 1e-3f;

  explicit QuasiEuclideanSDFGenerator(FloatingPoint max_distance = 2.f,
                                      FloatingPoint occupancy_threshold = 0.f)
      : occupancy_threshold_(occupancy_threshold),
        max_distance_(max_distance) {}

  HashedBlocks generate(const HashedWaveletOctree& occupancy_map);

  FloatingPoint getOccupancyThreshold() const { return occupancy_threshold_; }
  FloatingPoint getMaxDistance() const { return max_distance_; }

  static bool isUnobserved(FloatingPoint occupancy_value) {
    return std::abs(occupancy_value) < 1e-3f;
  }
  static bool isObserved(FloatingPoint occupancy_value) {
    return !isUnobserved(occupancy_value);
  }
  bool isFree(FloatingPoint occupancy_value) const {
    return occupancy_value < occupancy_threshold_;
  }
  bool isOccupied(FloatingPoint occupancy_value) const {
    return occupancy_threshold_ < occupancy_value;
  }

 private:
  static constexpr FloatingPoint kTolerance = 1e-2f;
  inline static const auto kNeighborIndexOffsets =
      neighborhood::generateNeighborIndexOffsets();

  const FloatingPoint occupancy_threshold_;
  const FloatingPoint max_distance_;

  void seed(const HashedWaveletOctree& occupancy_map, HashedBlocks& sdf,
            BucketQueue<Index3D>& open_queue) const;
  void propagate(const HashedWaveletOctree& occupancy_map, HashedBlocks& sdf,
                 BucketQueue<Index3D>& open_queue) const;
};
}  // namespace wavemap

#endif  // WAVEMAP_UTILS_SDF_SDF_GENERATOR_H_
