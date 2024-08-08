#ifndef WAVEMAP_CORE_UTILS_SDF_QUASI_EUCLIDEAN_SDF_GENERATOR_H_
#define WAVEMAP_CORE_UTILS_SDF_QUASI_EUCLIDEAN_SDF_GENERATOR_H_

#include "wavemap/core/data_structure/bucket_queue.h"
#include "wavemap/core/map/hashed_blocks.h"
#include "wavemap/core/map/hashed_wavelet_octree.h"
#include "wavemap/core/utils/neighbors/grid_neighborhood.h"
#include "wavemap/core/utils/query/occupancy_classifier.h"

namespace wavemap {
class QuasiEuclideanSDFGenerator {
 public:
  static constexpr FloatingPoint kMaxRelativeUnderEstimate = 1e-2f;
  static constexpr FloatingPoint kMaxRelativeOverEstimate = 0.125f + 1e-2f;

  explicit QuasiEuclideanSDFGenerator(FloatingPoint max_distance = 2.f,
                                      FloatingPoint occupancy_threshold = 0.f)
      : max_distance_(max_distance), classifier_(occupancy_threshold) {}

  HashedBlocks generate(const HashedWaveletOctree& occupancy_map) const;

  FloatingPoint getMaxDistance() const { return max_distance_; }

 private:
  inline static const auto kNeighborIndexOffsets =
      GridNeighborhood<3>::generateIndexOffsets<Adjacency::kAnyDisjoint>();

  const FloatingPoint max_distance_;
  const OccupancyClassifier classifier_;

  void seed(const HashedWaveletOctree& occupancy_map, HashedBlocks& sdf,
            BucketQueue<Index3D>& open_queue) const;
  void propagate(const HashedWaveletOctree& occupancy_map, HashedBlocks& sdf,
                 BucketQueue<Index3D>& open_queue) const;
};
}  // namespace wavemap

#endif  // WAVEMAP_CORE_UTILS_SDF_QUASI_EUCLIDEAN_SDF_GENERATOR_H_
