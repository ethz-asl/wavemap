#include "wavemap/integrator/projective/coarse_to_fine/hashed_chunked_wavelet_integrator.h"

#include <stack>

#include <tracy/Tracy.hpp>

namespace wavemap {
void HashedChunkedWaveletIntegrator::updateMap() {
  ZoneScoped;
  // Update the range image intersector
  {
    ZoneScopedN("updateRangeImageIntersector");
    range_image_intersector_ = std::make_shared<RangeImageIntersector>(
        posed_range_image_, projection_model_, *measurement_model_,
        config_.min_range, config_.max_range);
  }

  // Find all the indices of blocks that need updating
  BlockList blocks_to_update;
  {
    ZoneScopedN("selectBlocksToUpdate");
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
      auto& block = occupancy_map_->getBlock(block_index);
      updateBlock(block, block_index);
    });
  }
  thread_pool_->wait_all();
}

std::pair<OctreeIndex, OctreeIndex>
HashedChunkedWaveletIntegrator::getFovMinMaxIndices(
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

void HashedChunkedWaveletIntegrator::updateBlock(
    HashedChunkedWaveletOctree::Block& block,
    const HashedChunkedWaveletOctree::BlockIndex& block_index) {
  ZoneScoped;
  block.setNeedsPruning();
  block.setLastUpdatedStamp();

  const OctreeIndex root_node_index{tree_height_, block_index};
  updateNodeRecursive(block.getRootChunk(), root_node_index, 0u,
                      block.getRootScale(),
                      block.getRootChunk().nodeHasAtLeastOneChild(0u),
                      block.getNeedsThresholding());
}

void HashedChunkedWaveletIntegrator::updateNodeRecursive(  // NOLINT
    HashedChunkedWaveletOctreeBlock::NodeChunkType& parent_chunk,
    const OctreeIndex& parent_node_index, LinearIndex parent_in_chunk_index,
    FloatingPoint& parent_value,
    HashedChunkedWaveletOctreeBlock::NodeChunkType::BitRef parent_has_child,
    bool& block_needs_thresholding) {
  auto& parent_details = parent_chunk.nodeData(parent_in_chunk_index);
  auto child_values = HashedChunkedWaveletOctreeBlock::Transform::backward(
      {parent_value, parent_details});

  // Handle all the children
  for (NdtreeIndexRelativeChild relative_child_idx = 0;
       relative_child_idx < OctreeIndex::kNumChildren; ++relative_child_idx) {
    const OctreeIndex child_index =
        parent_node_index.computeChildIndex(relative_child_idx);
    const int child_height = child_index.height;
    FloatingPoint& child_value = child_values[relative_child_idx];

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
        child_value < min_log_odds_shrunk_) {
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
    const MortonIndex morton_code = convert::nodeIndexToMorton(child_index);
    const int parent_height = child_height + 1;
    const int parent_chunk_top_height =
        chunk_height_ * int_math::div_round_up(parent_height, chunk_height_);

    HashedChunkedWaveletOctreeBlock::NodeChunkType* chunk_containing_child;
    LinearIndex child_node_in_chunk_index;
    if (child_height % chunk_height_ != 0) {
      chunk_containing_child = &parent_chunk;
      child_node_in_chunk_index = OctreeIndex::computeTreeTraversalDistance(
          morton_code, parent_chunk_top_height, child_height);
    } else {
      const LinearIndex linear_child_index =
          OctreeIndex::computeLevelTraversalDistance(
              morton_code, parent_chunk_top_height, child_height);
      chunk_containing_child = parent_chunk.getChild(linear_child_index);
      if (!chunk_containing_child) {
        chunk_containing_child = parent_chunk.allocateChild(linear_child_index);
      }
      child_node_in_chunk_index = 0u;
    }

    auto& child_details =
        chunk_containing_child->nodeData(child_node_in_chunk_index);
    auto child_has_children = chunk_containing_child->nodeHasAtLeastOneChild(
        child_node_in_chunk_index);

    // If we're at the leaf level, directly compute the update
    if (child_height == config_.termination_height + 1) {
      updateLeavesBatch(child_index, child_value, child_details);
    } else {
      // Otherwise, recurse
      DCHECK_GE(child_height, 0);
      updateNodeRecursive(*chunk_containing_child, child_index,
                          child_node_in_chunk_index, child_value,
                          child_has_children, block_needs_thresholding);
    }

    if (child_has_children || data_utils::is_nonzero(child_details)) {
      parent_has_child = true;
    }
  }

  const auto [new_value, new_details] =
      HashedChunkedWaveletOctreeBlock::Transform::forward(child_values);
  parent_details = new_details;
  parent_value = new_value;
}
}  // namespace wavemap
