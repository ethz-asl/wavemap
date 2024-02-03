#ifndef WAVEMAP_UTILS_QUERY_POINT_SAMPLER_H_
#define WAVEMAP_UTILS_QUERY_POINT_SAMPLER_H_

#include <utility>

#include "wavemap/common.h"
#include "wavemap/data_structure/aabb.h"
#include "wavemap/utils/query/classified_map.h"
#include "wavemap/utils/query/occupancy.h"
#include "wavemap/utils/random_number_generator.h"

namespace wavemap {
class PointSampler {
 public:
  explicit PointSampler(ClassifiedMap::ConstPtr classified_map)
      : classified_map_(std::move(classified_map)) {}

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

#include "wavemap/utils/query/impl/point_sampler_inl.h"

#endif  // WAVEMAP_UTILS_QUERY_POINT_SAMPLER_H_
