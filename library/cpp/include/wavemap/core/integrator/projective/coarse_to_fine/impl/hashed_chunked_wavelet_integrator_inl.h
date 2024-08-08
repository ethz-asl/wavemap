#ifndef WAVEMAP_CORE_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_HASHED_CHUNKED_WAVELET_INTEGRATOR_INL_H_
#define WAVEMAP_CORE_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_HASHED_CHUNKED_WAVELET_INTEGRATOR_INL_H_

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
    if (const auto* block = occupancy_map_->getBlock(node_index.position);
        block) {
      if (min_log_odds_shrunk_ <= block->getRootScale()) {
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
  // Decompress
  auto child_values = HashedChunkedWaveletOctreeBlock::Transform::backward(
      {parent_value, parent_details});

  // Get child center points in world frame W
  Eigen::Matrix<FloatingPoint, 3, OctreeIndex::kNumChildren> child_centers;
  for (int child_idx = 0; child_idx < OctreeIndex::kNumChildren; ++child_idx) {
    const auto child_index = parent_index.computeChildIndex(child_idx);
    child_centers.col(child_idx) =
        convert::nodeIndexToCenterPoint(child_index, min_cell_width_);
  }

  // Transform into sensor frame C
  const auto& T_C_W = posed_range_image_->getPoseInverse();
  for (int child_idx = 0; child_idx < OctreeIndex::kNumChildren; ++child_idx) {
    child_centers.col(child_idx) = T_C_W * child_centers.col(child_idx);
  }

  // Compute updated values
  for (int child_idx = 0; child_idx < OctreeIndex::kNumChildren; ++child_idx) {
    const FloatingPoint sample = computeUpdate(child_centers.col(child_idx));
    FloatingPoint& child_value = child_values[child_idx];
    child_value = sample + child_value;
  }

  // Threshold
  for (int child_idx = 0; child_idx < OctreeIndex::kNumChildren; ++child_idx) {
    FloatingPoint& child_value = child_values[child_idx];
    child_value = (child_value < min_log_odds_padded_) ? min_log_odds_padded_
                                                       : child_value;
    child_value = (max_log_odds_padded_ < child_value) ? max_log_odds_padded_
                                                       : child_value;
  }

  // Compress
  const auto [new_value, new_details] =
      HashedChunkedWaveletOctreeBlock::Transform::forward(child_values);
  parent_details = new_details;
  parent_value = new_value;
}
}  // namespace wavemap

#endif  // WAVEMAP_CORE_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_HASHED_CHUNKED_WAVELET_INTEGRATOR_INL_H_
