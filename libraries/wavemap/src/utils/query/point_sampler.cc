#include "wavemap/utils/query/point_sampler.h"

namespace wavemap {
std::optional<Index3D> PointSampler::getRandomBlock() {
  if (classified_map_->empty()) {
    return std::nullopt;
  }

  const size_t num_blocks = classified_map_->getHashMap().size();
  const size_t nth_block = rng_.getRandomInteger(0ul, num_blocks - 1ul);
  auto it = classified_map_->getHashMap().begin();
  std::advance(it, nth_block);
  if (it == classified_map_->getHashMap().end()) {
    return std::nullopt;
  }

  return it->first;
}

std::optional<Point3D> PointSampler::getRandomPointImpl(
    Occupancy::Mask occupancy_mask, const std::optional<AABB<Point3D>>& aabb,
    size_t& attempt_idx, size_t max_attempts) {
  if (classified_map_->empty()) {
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
    if (classified_map_->isFully(index, occupancy_mask)) {
      return point;
    }
  }

  return std::nullopt;
}
}  // namespace wavemap
