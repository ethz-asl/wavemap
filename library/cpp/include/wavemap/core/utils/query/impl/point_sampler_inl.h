#ifndef WAVEMAP_CORE_UTILS_QUERY_IMPL_POINT_SAMPLER_INL_H_
#define WAVEMAP_CORE_UTILS_QUERY_IMPL_POINT_SAMPLER_INL_H_

namespace wavemap {
inline std::optional<Point3D> PointSampler::getRandomPoint(
    Occupancy::Id occupancy_type, const std::optional<AABB<Point3D>>& aabb,
    size_t max_attempts) {
  return getRandomPoint(Occupancy::toMask(occupancy_type), aabb, max_attempts);
}

inline std::optional<Point3D> PointSampler::getRandomPoint(
    Occupancy::Mask occupancy_mask, const std::optional<AABB<Point3D>>& aabb,
    size_t max_attempts) {
  size_t attempt_idx = 0;
  return getRandomPointImpl(occupancy_mask, aabb, attempt_idx, max_attempts);
}

inline Point3D PointSampler::getRandomPointInAABB(const AABB<Point3D>& aabb) {
  return {rng_.getRandomRealNumber(aabb.min[0], aabb.max[0]),
          rng_.getRandomRealNumber(aabb.min[1], aabb.max[1]),
          rng_.getRandomRealNumber(aabb.min[2], aabb.max[2])};
}

inline Point3D PointSampler::getRandomPointInBlock(const Index3D& block_index) {
  const auto block_aabb = convert::nodeIndexToAABB(
      OctreeIndex{block_height_, block_index}, min_cell_width_);
  return getRandomPointInAABB(block_aabb);
}
}  // namespace wavemap

#endif  // WAVEMAP_CORE_UTILS_QUERY_IMPL_POINT_SAMPLER_INL_H_
