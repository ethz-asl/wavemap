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

inline FloatingPoint
HashedWaveletIntegrator::recursiveSamplerCompressor(  // NOLINT
    const OctreeIndex& node_index, FloatingPoint node_value,
    typename HashedWaveletOctree::NodeType& parent_node,
    NdtreeIndexRelativeChild relative_child_index) {
  // If we're at the leaf level, directly update the node
  if (node_index.height == config_.termination_height) {
    const Point3D W_node_center =
        convert::nodeIndexToCenterPoint(node_index, min_cell_width_);
    const Point3D C_node_center =
        posed_range_image_->getPoseInverse() * W_node_center;
    const FloatingPoint sample = computeUpdate(C_node_center);
    return std::clamp(sample + node_value, min_log_odds_ - kNoiseThreshold,
                      max_log_odds_ + kNoiseThreshold) -
           node_value;
  }

  // Otherwise, test whether the current node is fully occupied;
  // free or unknown; or fully unknown
  const AABB<Point3D> W_cell_aabb =
      convert::nodeIndexToAABB(node_index, min_cell_width_);
  const UpdateType update_type = range_image_intersector_->determineUpdateType(
      W_cell_aabb, posed_range_image_->getRotationMatrixInverse(),
      posed_range_image_->getOrigin());

  // If we're fully in unknown space,
  // there's no need to evaluate this node or its children
  if (update_type == UpdateType::kFullyUnobserved) {
    return 0.f;
  }

  // We can also stop here if the cell will result in a free space update (or
  // zero) and the map is already saturated free
  if (update_type != UpdateType::kPossiblyOccupied &&
      node_value < min_log_odds_ + kNoiseThreshold / 10.f) {
    return 0.f;
  }

  // Test if the worst-case error for the intersection type at the current
  // resolution falls within the acceptable approximation error
  const FloatingPoint node_width = W_cell_aabb.width<0>();
  const Point3D W_node_center =
      W_cell_aabb.min + Vector3D::Constant(node_width / 2.f);
  const Point3D C_node_center =
      posed_range_image_->getPoseInverse() * W_node_center;
  const FloatingPoint d_C_cell =
      projection_model_->cartesianToSensorZ(C_node_center);
  const FloatingPoint bounding_sphere_radius =
      kUnitCubeHalfDiagonal * node_width;
  HashedWaveletOctree::NodeType* node =
      parent_node.getChild(relative_child_index);
  if (measurement_model_->computeWorstCaseApproximationError(
          update_type, d_C_cell, bounding_sphere_radius) <
      config_.termination_update_error) {
    const FloatingPoint sample = computeUpdate(C_node_center);
    if (!node || !node->hasAtLeastOneChild()) {
      return std::clamp(sample + node_value, min_log_odds_ - kNoiseThreshold,
                        max_log_odds_ + kNoiseThreshold) -
             node_value;
    } else {
      return sample;
    }
  }

  // Since the approximation error would still be too big, refine
  if (!node) {
    // Allocate the current node if it has not yet been allocated
    node = parent_node.allocateChild(relative_child_index);
  }
  const HashedWaveletOctree::Coefficients::CoefficientsArray
      child_scale_coefficients =
          HashedWaveletOctree::Transform::backward({node_value, node->data()});
  HashedWaveletOctree::Coefficients::CoefficientsArray
      child_scale_coefficient_updates;
  for (NdtreeIndexRelativeChild relative_child_idx = 0;
       relative_child_idx < OctreeIndex::kNumChildren; ++relative_child_idx) {
    const OctreeIndex child_index =
        node_index.computeChildIndex(relative_child_idx);
    const FloatingPoint child_value =
        child_scale_coefficients[relative_child_idx];
    child_scale_coefficient_updates[relative_child_idx] =
        recursiveSamplerCompressor(child_index, child_value, *node,
                                   relative_child_idx);
  }

  // Update the current node's wavelet detail coefficients
  const auto [scale_update, detail_updates] =
      HashedWaveletOctree::Transform::forward(child_scale_coefficient_updates);
  node->data() += detail_updates;

  // Propagate the wavelet scale coefficient upward
  return scale_update;
}
}  // namespace wavemap

#endif  // WAVEMAP_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_HASHED_WAVELET_INTEGRATOR_INL_H_
