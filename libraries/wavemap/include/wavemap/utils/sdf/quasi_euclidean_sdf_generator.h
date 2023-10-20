#ifndef WAVEMAP_UTILS_SDF_QUASI_EUCLIDEAN_SDF_GENERATOR_H_
#define WAVEMAP_UTILS_SDF_QUASI_EUCLIDEAN_SDF_GENERATOR_H_

#include "wavemap/data_structure/bucket_queue.h"
#include "wavemap/map/hashed_blocks.h"
#include "wavemap/map/hashed_wavelet_octree.h"
#include "wavemap/utils/query/occupancy_classifier.h"
#include "wavemap/utils/sdf/cell_neighborhoods.h"

namespace wavemap {
class QuasiEuclideanSDFGenerator {
 public:
  static constexpr FloatingPoint kMaxMultiplicativeError = 0.125f + 1e-3f;

  explicit QuasiEuclideanSDFGenerator(FloatingPoint max_distance = 2.f,
                                      FloatingPoint occupancy_threshold = 0.f)
      : max_distance_(max_distance), classifier_(occupancy_threshold) {}

  HashedBlocks generate(const HashedWaveletOctree& occupancy_map);

  FloatingPoint getMaxDistance() const { return max_distance_; }

 private:
  static constexpr FloatingPoint kTolerance = 1e-2f;
  inline static const auto kNeighborIndexOffsets =
      neighborhood::generateNeighborIndexOffsets();

  const FloatingPoint max_distance_;
  const OccupancyClassifier classifier_;

  void seed(const HashedWaveletOctree& occupancy_map, HashedBlocks& sdf,
            BucketQueue<Index3D>& open_queue) const;
  void propagate(const HashedWaveletOctree& occupancy_map, HashedBlocks& sdf,
                 BucketQueue<Index3D>& open_queue) const;
};
}  // namespace wavemap

#endif  // WAVEMAP_UTILS_SDF_QUASI_EUCLIDEAN_SDF_GENERATOR_H_
