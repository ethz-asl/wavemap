#ifndef WAVEMAP_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_HASHED_CHUNKED_WAVELET_INTEGRATOR_INL_H_
#define WAVEMAP_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_HASHED_CHUNKED_WAVELET_INTEGRATOR_INL_H_

#include <utility>

namespace wavemap {
inline void HashedChunkedWaveletIntegrator::recursiveTester(  // NOLINT
    const OctreeIndex& node_index,
    HashedChunkedWaveletIntegrator::BlockList& update_job_list) {
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
      update_job_list.emplace_back(node_index.position);
      return;
    }
    if (occupancy_map_->hasBlock(node_index.position)) {
      const auto& block = occupancy_map_->getBlock(node_index.position);
      if (min_log_odds_shrunk_ <= block.getRootScale()) {
        // Add the block to the job list
        update_job_list.emplace_back(node_index.position);
      }
    }
    return;
  }

  for (const auto& child_index : node_index.computeChildIndices()) {
    recursiveTester(child_index, update_job_list);
  }
}

inline void HashedChunkedWaveletIntegrator::updateLeavesBatch(
    const OctreeIndex& parent_index, FloatingPoint& parent_value,
    HaarCoefficients<FloatingPoint, 3>::Details& parent_details) {
  auto child_values = HashedChunkedWaveletOctreeBlock::Transform::backward(
      {parent_value, parent_details});
  for (NdtreeIndexRelativeChild relative_child_idx = 0;
       relative_child_idx < OctreeIndex::kNumChildren; ++relative_child_idx) {
    const auto child_index = parent_index.computeChildIndex(relative_child_idx);
    FloatingPoint& child_value = child_values[relative_child_idx];
    const Point3D W_child_center =
        convert::nodeIndexToCenterPoint(child_index, min_cell_width_);
    const Point3D C_child_center =
        posed_range_image_->getPoseInverse() * W_child_center;
    const FloatingPoint sample = computeUpdate(C_child_center);
    child_value = std::clamp(sample + child_value, min_log_odds_padded_,
                             max_log_odds_padded_);
  }
  const auto [new_value, new_details] =
      HashedChunkedWaveletOctreeBlock::Transform::forward(child_values);
  parent_details = new_details;
  parent_value = new_value;
}
}  // namespace wavemap

#endif  // WAVEMAP_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_HASHED_CHUNKED_WAVELET_INTEGRATOR_INL_H_
