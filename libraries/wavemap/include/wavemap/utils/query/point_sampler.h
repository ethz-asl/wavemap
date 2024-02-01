#ifndef WAVEMAP_UTILS_QUERY_POINT_SAMPLER_H_
#define WAVEMAP_UTILS_QUERY_POINT_SAMPLER_H_

#include "wavemap/common.h"
#include "wavemap/data_structure/aabb.h"
#include "wavemap/map/hashed_blocks.h"
#include "wavemap/map/hashed_wavelet_octree.h"
#include "wavemap/utils/query/occupancy.h"
#include "wavemap/utils/query/occupancy_classifier.h"
#include "wavemap/utils/query/query_accelerator.h"
#include "wavemap/utils/random_number_generator.h"

namespace wavemap {
class PointSampler {
 public:
  explicit PointSampler(const HashedWaveletOctree& occupancy_map,
                        OccupancyClassifier classifier = OccupancyClassifier{})
      : occupancy_map_(occupancy_map), classifier_(classifier) {}

  std::optional<Index3D> getRandomBlock();

  std::optional<Point3D> getRandomPoint(
      Occupancy::Id occupancy_type,
      const std::optional<AABB<Point3D>>& aabb = std::nullopt,
      size_t max_attempts = 1000);
  std::optional<Point3D> getRandomPoint(
      Occupancy::Mask occupancy_mask,
      const std::optional<AABB<Point3D>>& aabb = std::nullopt,
      size_t max_attempts = 1000);

  std::optional<Point3D> getRandomCollisionFreePoint(
      const HashedBlocks& esdf, FloatingPoint robot_radius,
      const std::optional<AABB<Point3D>>& aabb = std::nullopt,
      size_t max_attempts = 1000);

 private:
  const HashedWaveletOctree& occupancy_map_;
  const OccupancyClassifier classifier_;

  QueryAccelerator<HashedWaveletOctree> query_accelerator_{occupancy_map_};

  RandomNumberGenerator rng_;
  const IndexElement block_height_ = occupancy_map_.getTreeHeight();
  const FloatingPoint min_cell_width_ = occupancy_map_.getMinCellWidth();
  const FloatingPoint min_cell_width_inv_ = 1.f / min_cell_width_;

  std::optional<Point3D> getRandomPointImpl(
      Occupancy::Mask occupancy_mask, const std::optional<AABB<Point3D>>& aabb,
      size_t& attempt_idx, size_t max_attempts);

  Point3D getRandomPointInAABB(const AABB<Point3D>& aabb);
  Point3D getRandomPointInBlock(const Index3D& block_index);
};

}  // namespace wavemap

#include "wavemap/utils/query/impl/point_sampler_inl.h"

#endif  // WAVEMAP_UTILS_QUERY_POINT_SAMPLER_H_
