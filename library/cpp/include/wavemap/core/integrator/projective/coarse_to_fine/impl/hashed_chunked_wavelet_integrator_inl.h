#ifndef WAVEMAP_CORE_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_HASHED_CHUNKED_WAVELET_INTEGRATOR_INL_H_
#define WAVEMAP_CORE_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_HASHED_CHUNKED_WAVELET_INTEGRATOR_INL_H_

#include <algorithm>
#include <memory>
#include <utility>

#include <wavemap/core/utils/profile/profiler_interface.h>

namespace wavemap {
template <typename CellDataT>
void HashedChunkedWaveletIntegratorT<CellDataT>::updateMap() {
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
HashedChunkedWaveletIntegratorT<CellDataT>::getFovMinMaxIndices(
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
void HashedChunkedWaveletIntegratorT<CellDataT>::updateBlock(
    Block& block,
    const BlockIndex& block_index) {
  ProfilerZoneScoped;
  block.setNeedsPruning();
  block.setLastUpdatedStamp();

  bool block_needs_thresholding = block.getNeedsThresholding();
  const OctreeIndex root_node_index{tree_height_, block_index};
  updateNodeRecursive(block.getRootNode(), root_node_index,
                      block.getRootScale(), block_needs_thresholding);
  block.setNeedsThresholding(block_needs_thresholding);
}

template <typename CellDataT>
void HashedChunkedWaveletIntegratorT<CellDataT>::updateNodeRecursive(  // NOLINT
    typename OctreeType::NodeRefType node,
    const OctreeIndex& node_index, CellDataT& node_value,
    bool& block_needs_thresholding) {
  // Decompress child values
  auto& node_details = node.data();
  auto child_values = Block::Transform::backward(
      {node_value, node_details});

  // Handle each child
  for (NdtreeIndexRelativeChild relative_child_idx = 0;
       relative_child_idx < OctreeIndex::kNumChildren; ++relative_child_idx) {
    const OctreeIndex child_index =
        node_index.computeChildIndex(relative_child_idx);
    CellDataT& child_value = child_values[relative_child_idx];

    // Test whether it is fully occupied; free or unknown; or fully unknown
    const AABB<Point3D> W_child_aabb =
        convert::nodeIndexToAABB(child_index, min_cell_width_);
    const UpdateType update_type =
        range_image_intersector_->determineUpdateType(
            W_child_aabb, posed_range_image_->getRotationMatrixInverse(),
            posed_range_image_->getOrigin());

    // If we're fully in unknown space,
    // there's no need to evaluate this node or its children
    if (update_type == UpdateType::kFullyUnobserved) {
      continue;
    }

    // We can also stop here if the cell will result in a free space update
    // (or zero) and the map is already saturated free
    if (update_type != UpdateType::kPossiblyOccupied &&
        occupancyOf(child_value) < min_log_odds_shrunk_) {
      continue;
    }

    // Test if the worst-case error for the intersection type at the current
    // resolution falls within the acceptable approximation error
    const FloatingPoint child_width = W_child_aabb.width<0>();
    const Point3D W_child_center =
        W_child_aabb.min + Vector3D::Constant(child_width / 2.f);
    const Point3D C_child_center =
        posed_range_image_->getPoseInverse() * W_child_center;
    const FloatingPoint d_C_child =
        projection_model_->cartesianToSensorZ(C_child_center);
    const FloatingPoint bounding_sphere_radius =
        kUnitCubeHalfDiagonal * child_width;
    if (measurement_model_->computeWorstCaseApproximationError(
            update_type, d_C_child, bounding_sphere_radius) <
        config_.termination_update_error) {
      const FloatingPoint sample = computeUpdate(C_child_center);
      child_value += sample;
      block_needs_thresholding = true;
      continue;
    }

    // Since the approximation error would still be too big, refine
    auto child_node = node.getOrAllocateChild(relative_child_idx);
    auto& child_details = child_node.data();

    // If we're at the leaf level, directly compute the update
    if (child_index.height <= termination_height_ + 1) {
      updateLeavesBatch(child_index, child_value, child_details);
    } else {
      // Otherwise, recurse
      DCHECK_GE(child_index.height, 0);
      updateNodeRecursive(child_node, child_index, child_value,
                          block_needs_thresholding);
    }
  }

  // Compress
  const auto [new_value, new_details] =
      Block::Transform::forward(child_values);
  node_details = new_details;
  node_value = new_value;
}
template <typename CellDataT>
inline void HashedChunkedWaveletIntegratorT<CellDataT>::recursiveTester(  // NOLINT
    const OctreeIndex& node_index,
    HashedChunkedWaveletIntegratorT<CellDataT>::BlockList& update_job_list) {
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
      if (min_log_odds_shrunk_ <= occupancyOf(block->getRootScale())) {
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
inline void HashedChunkedWaveletIntegratorT<CellDataT>::updateLeavesBatch(
    const OctreeIndex& parent_index, CellDataT& parent_value,
    typename OctreeType::NodeDataType& parent_details) {
  // Decompress
  auto child_values = Block::Transform::backward(
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
    CellDataT& child_value = child_values[child_idx];
    addClampedOccupancyUpdate(child_value, sample,
                              min_log_odds_padded_, max_log_odds_padded_);
  }

  // Compress
  const auto [new_value, new_details] =
      Block::Transform::forward(child_values);
  parent_details = new_details;
  parent_value = new_value;
}
}  // namespace wavemap

#endif  // WAVEMAP_CORE_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_HASHED_CHUNKED_WAVELET_INTEGRATOR_INL_H_
