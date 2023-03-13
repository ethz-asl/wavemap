#ifndef WAVEMAP_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_HASHED_WAVELET_INTEGRATOR_INL_H_
#define WAVEMAP_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_HASHED_WAVELET_INTEGRATOR_INL_H_

#include <utility>

namespace wavemap {
inline void HashedWaveletIntegrator::recursiveTester(  // NOLINT
    const OctreeIndex& node_index,
    HashedWaveletIntegrator::BlockList& job_list) {
  const AABB<Point3D> block_aabb =
      convert::nodeIndexToAABB(node_index, min_cell_width_);
  const UpdateType update_type = range_image_intersector_->determineUpdateType(
      block_aabb, posed_range_image_->getRotationMatrixInverse(),
      posed_range_image_->getOrigin());
  if (update_type == UpdateType::kFullyUnobserved) {
    return;
  }

  if (node_index.height == tree_height_) {
    // Get the block
    if (update_type == UpdateType::kPossiblyOccupied) {
      job_list.emplace_back(node_index);
      return;
    }
    if (occupancy_map_->hasBlock(node_index.position)) {
      const auto& block = occupancy_map_->getBlock(node_index.position);
      if (min_log_odds_ + kNoiseThreshold / 10.f <= block.getRootScale()) {
        job_list.emplace_back(node_index);
      }
    }
    return;
  }

  for (const auto& child_index : node_index.computeChildIndices()) {
    recursiveTester(child_index, job_list);
  }
}
}  // namespace wavemap

#endif  // WAVEMAP_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_HASHED_WAVELET_INTEGRATOR_INL_H_
