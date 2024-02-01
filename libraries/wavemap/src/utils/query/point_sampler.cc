#include "wavemap/utils/query/point_sampler.h"

namespace wavemap {
std::optional<Index3D> PointSampler::getRandomBlock() {
  if (occupancy_map_.empty()) {
    return std::nullopt;
  }

  const size_t num_blocks = occupancy_map_.getHashMap().size();
  const size_t nth_block = rng_.getRandomInteger(0ul, num_blocks - 1ul);
  auto it = occupancy_map_.getHashMap().begin();
  std::advance(it, nth_block);
  if (it == occupancy_map_.getHashMap().end()) {
    return std::nullopt;
  }

  return it->first;
}

std::optional<Point3D> PointSampler::getRandomCollisionFreePoint(
    const HashedBlocks& esdf, FloatingPoint robot_radius,
    const std::optional<AABB<Point3D>>& aabb, size_t max_attempts) {
  for (size_t attempt_idx = 0; attempt_idx < max_attempts; ++attempt_idx) {
    auto point =
        getRandomPointImpl(Occupancy::kFree, aabb, attempt_idx, max_attempts);
    if (!point) {
      continue;
    }
    const auto index =
        convert::pointToNearestIndex(point.value(), min_cell_width_inv_);
    const FloatingPoint distance = esdf.getCellValue(index);
    if (robot_radius <= distance) {
      return point.value();
    }
  }

  return std::nullopt;
}

std::optional<Point3D> PointSampler::getRandomPointImpl(
    Occupancy::Mask occupancy_mask, const std::optional<AABB<Point3D>>& aabb,
    size_t& attempt_idx, size_t max_attempts) {
  if (occupancy_map_.empty()) {
    return std::nullopt;
  }

  for (; attempt_idx < max_attempts; ++attempt_idx) {
    Point3D point;
    if (aabb) {
      point = getRandomPointInAABB(aabb.value());
    } else if (const auto block_index = getRandomBlock(); block_index) {
      point = getRandomPointInBlock(block_index.value());
    } else {
      continue;
    }
    const auto index = convert::pointToNearestIndex(point, min_cell_width_inv_);
    const FloatingPoint log_odds = query_accelerator_.getCellValue(index);
    if (classifier_.is(log_odds, occupancy_mask)) {
      return point;
    }
  }

  return std::nullopt;
}
}  // namespace wavemap
