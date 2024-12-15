#ifndef WAVEMAP_CORE_UTILS_QUERY_POINT_SAMPLER_H_
#define WAVEMAP_CORE_UTILS_QUERY_POINT_SAMPLER_H_

#include <random>
#include <utility>

#include "wavemap/core/common.h"
#include "wavemap/core/utils/geometry/aabb.h"
#include "wavemap/core/utils/query/classified_map.h"
#include "wavemap/core/utils/query/occupancy.h"
#include "wavemap/core/utils/random_number_generator.h"

namespace wavemap {
class PointSampler {
 public:
  explicit PointSampler(ClassifiedMap::ConstPtr classified_map,
                        size_t random_seed = std::random_device()())
      : classified_map_(std::move(classified_map)), rng_(random_seed) {}

  std::optional<Index3D> getRandomBlock();

  std::optional<Point3D> getRandomPoint(
      Occupancy::Id occupancy_type,
      const std::optional<AABB<Point3D>>& aabb = std::nullopt,
      size_t max_attempts = 1000);
  std::optional<Point3D> getRandomPoint(
      Occupancy::Mask occupancy_mask,
      const std::optional<AABB<Point3D>>& aabb = std::nullopt,
      size_t max_attempts = 1000);

 private:
  const ClassifiedMap::ConstPtr classified_map_;

  RandomNumberGenerator rng_;

  const IndexElement block_height_ = classified_map_->getTreeHeight();
  const FloatingPoint min_cell_width_ = classified_map_->getMinCellWidth();
  const FloatingPoint min_cell_width_inv_ = 1.f / min_cell_width_;

  std::optional<Point3D> getRandomPointImpl(
      Occupancy::Mask occupancy_mask, const std::optional<AABB<Point3D>>& aabb,
      size_t& attempt_idx, size_t max_attempts);

  Point3D getRandomPointInAABB(const AABB<Point3D>& aabb);
  Point3D getRandomPointInBlock(const Index3D& block_index);
};
}  // namespace wavemap

#include "wavemap/core/utils/query/impl/point_sampler_inl.h"

#endif  // WAVEMAP_CORE_UTILS_QUERY_POINT_SAMPLER_H_
