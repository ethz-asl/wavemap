#ifndef WAVEMAP_CORE_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_HASHED_WAVELET_INTEGRATOR_INL_H_
#define WAVEMAP_CORE_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_HASHED_WAVELET_INTEGRATOR_INL_H_

#include <memory>
#include <stack>
#include <utility>

#include <wavemap/core/utils/profile/profiler_interface.h>

namespace wavemap {
template <typename CellDataT>
void HashedWaveletIntegratorT<CellDataT>::updateMap() {
  ProfilerZoneScoped;
  // Update the range image intersector
  {
    ProfilerZoneScopedN("updateRangeImageIntersector");
    range_image_intersector_ = std::make_shared<RangeImageIntersector>(
        posed_range_image_, projection_model_, *measurement_model_,
        config_.min_range, config_.max_range);
  }

  // Find all the indices of blocks that need updating
  BlockList blocks_to_update;
  {
    ProfilerZoneScopedN("selectBlocksToUpdate");
    const auto [fov_min_idx, fov_max_idx] =
        getFovMinMaxIndices(posed_range_image_->getOrigin());
    for (const auto& block_index :
         Grid(fov_min_idx.position, fov_max_idx.position)) {
      recursiveTester(OctreeIndex{fov_min_idx.height, block_index},
                      blocks_to_update);
    }
  }

  // Make sure the to-be-updated blocks are allocated
  for (const auto& block_index : blocks_to_update) {
    occupancy_map_->getOrAllocateBlock(block_index);
  }

  // Update it with the threadpool
  for (const auto& block_index : blocks_to_update) {
    thread_pool_->add_task([this, block_index]() {
      if (auto* block = occupancy_map_->getBlock(block_index); block) {
        updateBlock(*block, block_index);
      }
    });
  }
  thread_pool_->wait_all();
}

template <typename CellDataT>
std::pair<OctreeIndex, OctreeIndex>
HashedWaveletIntegratorT<CellDataT>::getFovMinMaxIndices(
    const Point3D& sensor_origin) const {
  const int height = 1 + std::max(static_cast<int>(std::ceil(std::log2(
                                      config_.max_range / min_cell_width_))),
                                  tree_height_);
  const OctreeIndex fov_min_idx = convert::indexAndHeightToNodeIndex<3>(
      convert::pointToFloorIndex<3>(
          sensor_origin - Vector3D::Constant(config_.max_range),
          min_cell_width_inv_) -
          occupancy_map_->getBlockSize(),
      height);
  const OctreeIndex fov_max_idx = convert::indexAndHeightToNodeIndex<3>(
      convert::pointToCeilIndex<3>(
          sensor_origin + Vector3D::Constant(config_.max_range),
          min_cell_width_inv_) +
          occupancy_map_->getBlockSize(),
      height);
  return {fov_min_idx, fov_max_idx};
}

template <typename CellDataT>
void HashedWaveletIntegratorT<CellDataT>::recursiveTester(  // NOLINT
    const OctreeIndex& node_index,
    HashedWaveletIntegratorT<CellDataT>::BlockList& update_job_list) {
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
      if (min_log_odds_ + kNoiseThreshold / 10.f <=
          occupancyOf(block->getRootScale())) {
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

template <typename CellDataT>
void HashedWaveletIntegratorT<CellDataT>::updateBlock(
    Block& block, const BlockIndex& block_index) {
  ProfilerZoneScoped;
  block.setNeedsPruning();
  block.setLastUpdatedStamp();

  struct StackElement {
    typename OctreeType::NodeRefType parent_node;
    const OctreeIndex parent_node_index;
    NdtreeIndexRelativeChild next_child_idx;
    typename Coefficients::CoefficientsArray child_scale_coefficients;
  };
  std::stack<StackElement> stack;

  typename OctreeType::NodeRefType root_node = block.getRootNode();
  typename Coefficients::Scale& root_node_scale = block.getRootScale();
  stack.emplace(StackElement{root_node,
                             {tree_height_, block_index},
                             0,
                             Block::Transform::backward(
                                 {root_node_scale, root_node.data()})});

  while (!stack.empty()) {
    // If the current stack element has fully been processed, propagate upward
    if (OctreeIndex::kNumChildren <= stack.top().next_child_idx) {
      const auto [scale, details] =
          Block::Transform::forward(stack.top().child_scale_coefficients);
      stack.top().parent_node.data() = details;
      stack.pop();
      if (stack.empty()) {
        root_node_scale = scale;
        return;
      } else {
        const NdtreeIndexRelativeChild current_child_idx =
            stack.top().next_child_idx - 1;
        stack.top().child_scale_coefficients[current_child_idx] = scale;
        continue;
      }
    }

    // Evaluate stack element's active child
    const NdtreeIndexRelativeChild current_child_idx =
        stack.top().next_child_idx;
    ++stack.top().next_child_idx;
    DCHECK_GE(current_child_idx, 0);
    DCHECK_LT(current_child_idx, OctreeIndex::kNumChildren);

    typename OctreeType::NodeRefType parent_node = stack.top().parent_node;
    CellDataT& node_value =
        stack.top().child_scale_coefficients[current_child_idx];
    const OctreeIndex node_index =
        stack.top().parent_node_index.computeChildIndex(current_child_idx);
    DCHECK_GE(node_index.height, 0);

    // If we're at the leaf level, directly update the node
    if (node_index.height <= termination_height_) {
      const Point3D W_node_center =
          convert::nodeIndexToCenterPoint(node_index, min_cell_width_);
      const Point3D C_node_center =
          posed_range_image_->getPoseInverse() * W_node_center;
      const FloatingPoint sample = computeUpdate(C_node_center);
      addClampedOccupancyUpdate(node_value, sample,
                                min_log_odds_ - kNoiseThreshold,
                                max_log_odds_ + kNoiseThreshold);
      continue;
    }

    // Otherwise, test whether the current node is fully occupied;
    // free or unknown; or fully unknown
    const AABB<Point3D> W_cell_aabb =
        convert::nodeIndexToAABB(node_index, min_cell_width_);
    const UpdateType update_type =
        range_image_intersector_->determineUpdateType(
            W_cell_aabb, posed_range_image_->getRotationMatrixInverse(),
            posed_range_image_->getOrigin());

    // If we're fully in unknown space,
    // there's no need to evaluate this node or its children
    if (update_type == UpdateType::kFullyUnobserved) {
      continue;
    }

    // We can also stop here if the cell will result in a free space update
    // (or zero) and the map is already saturated free
    if (update_type != UpdateType::kPossiblyOccupied &&
        occupancyOf(node_value) < min_log_odds_ + kNoiseThreshold / 10.f) {
      continue;
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
    typename OctreeType::NodePtrType node =
        parent_node.getChild(node_index.computeRelativeChildIndex());
    if (measurement_model_->computeWorstCaseApproximationError(
            update_type, d_C_cell, bounding_sphere_radius) <
        config_.termination_update_error) {
      const FloatingPoint sample = computeUpdate(C_node_center);
      if (!node || !node->hasAtLeastOneChild()) {
        addClampedOccupancyUpdate(node_value, sample,
                                  min_log_odds_ - kNoiseThreshold,
                                  max_log_odds_ + kNoiseThreshold);
      } else {
        node_value += sample;
        block.setNeedsThresholding();
      }
      continue;
    }

    // Since the approximation error would still be too big, refine
    if (!node) {
      // Allocate the current node if it has not yet been allocated
      node = &parent_node.getOrAllocateChild(
          node_index.computeRelativeChildIndex());
    }
    stack.emplace(StackElement{*node, node_index, 0,
                               Block::Transform::backward(
                                   {node_value, node->data()})});
  }
}
}  // namespace wavemap

#endif  // WAVEMAP_CORE_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_HASHED_WAVELET_INTEGRATOR_INL_H_
