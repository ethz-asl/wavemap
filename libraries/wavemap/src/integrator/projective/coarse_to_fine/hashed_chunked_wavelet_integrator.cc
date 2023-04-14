#include "wavemap/integrator/projective/coarse_to_fine/hashed_chunked_wavelet_integrator.h"

#include <stack>

namespace wavemap {
void HashedChunkedWaveletIntegrator::updateMap() {
  // Update the range image intersector
  range_image_intersector_ = std::make_shared<RangeImageIntersector>(
      posed_range_image_, projection_model_, *measurement_model_,
      config_.min_range, config_.max_range);

  // Find all the indices of blocks that need updating
  BlockList blocks_to_update;
  const auto [fov_min_idx, fov_max_idx] =
      getFovMinMaxIndices(posed_range_image_->getOrigin());
  for (const auto& block_index :
       Grid(fov_min_idx.position, fov_max_idx.position)) {
    recursiveTester(OctreeIndex{fov_min_idx.height, block_index},
                    blocks_to_update);
  }

  // Make sure the to-be-updated blocks are allocated
  for (const auto& block_index : blocks_to_update) {
    occupancy_map_->getOrAllocateBlock(block_index.position);
  }

  // Update it with the threadpool
  for (const auto& block_index : blocks_to_update) {
    thread_pool_.add_task([this, block_index]() {
      auto& block = occupancy_map_->getBlock(block_index.position);
      updateBlock(block, block_index);
    });
  }
  thread_pool_.wait_all();
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
    HashedChunkedWaveletOctree::Block& block, const OctreeIndex& block_index) {
  block.setNeedsPruning();
  block.setLastUpdatedStamp();

  std::stack<StackElement, std::vector<StackElement>> stack;
  stack.emplace(
      StackElement{block.getRootChunk(), block_index, 0,
                   HashedChunkedWaveletOctree::Transform::backward(
                       {block.getRootScale(), block.getRootChunk().data(0u)})});
  while (!stack.empty()) {
    // If the current stack element has fully been processed, propagate upward
    if (OctreeIndex::kNumChildren <= stack.top().next_child_idx) {
      const auto [new_scale, new_details] =
          HashedChunkedWaveletOctree::Transform::forward(
              stack.top().child_scale_coefficients);
      const MortonCode morton_code =
          convert::nodeIndexToMorton(stack.top().parent_node_index);
      const int parent_height = stack.top().parent_node_index.height;
      const int chunk_top_height =
          chunk_height_ * int_math::div_round_up(parent_height, chunk_height_);
      const LinearIndex value_index = OctreeIndex::computeTreeTraversalDistance(
          morton_code, chunk_top_height, parent_height);
      stack.top().parent_chunk.data(value_index) = new_details;
      stack.pop();
      if (stack.empty()) {
        block.getRootScale() = new_scale;
        return;
      } else {
        const NdtreeIndexRelativeChild current_child_idx =
            stack.top().next_child_idx - 1;
        stack.top().child_scale_coefficients[current_child_idx] = new_scale;
        continue;
      }
    }

    // Evaluate stack element's active child
    const NdtreeIndexRelativeChild current_child_idx =
        stack.top().next_child_idx;
    ++stack.top().next_child_idx;
    DCHECK_GE(current_child_idx, 0);
    DCHECK_LT(current_child_idx, OctreeIndex::kNumChildren);

    HashedChunkedWaveletOctree::NodeChunkType& parent_chunk =
        stack.top().parent_chunk;
    const OctreeIndex node_index =
        stack.top().parent_node_index.computeChildIndex(current_child_idx);
    FloatingPoint& node_value =
        stack.top().child_scale_coefficients[current_child_idx];
    DCHECK_GE(node_index.height, 0);

    // If we're at the leaf level, directly update the node
    if (node_index.height == config_.termination_height) {
      const Point3D W_node_center =
          convert::nodeIndexToCenterPoint(node_index, min_cell_width_);
      const Point3D C_node_center =
          posed_range_image_->getPoseInverse() * W_node_center;
      const FloatingPoint sample = computeUpdate(C_node_center);
      node_value = std::clamp(sample + node_value, min_log_odds_padded_,
                              max_log_odds_padded_);
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
        node_value < min_log_odds_shrunk_) {
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
    if (measurement_model_->computeWorstCaseApproximationError(
            update_type, d_C_cell, bounding_sphere_radius) <
        config_.termination_update_error) {
      const FloatingPoint sample = computeUpdate(C_node_center);
      node_value += sample;
      block.setNeedsThresholding();
      continue;
    }

    // Since the approximation error would still be too big, refine
    const MortonCode morton_code = convert::nodeIndexToMorton(node_index);
    const int node_height = node_index.height;
    const int parent_height = node_height + 1;
    const int parent_chunk_top_height =
        chunk_height_ * int_math::div_round_up(parent_height, chunk_height_);
    if (node_height % chunk_height_ != 0) {
      const LinearIndex value_index = OctreeIndex::computeTreeTraversalDistance(
          morton_code, parent_chunk_top_height, node_height);
      stack.emplace(
          StackElement{parent_chunk, node_index, 0,
                       HashedChunkedWaveletOctree::Transform::backward(
                           {node_value, parent_chunk.data(value_index)})});
    } else {
      const LinearIndex linear_child_index =
          OctreeIndex::computeLevelTraversalDistance(
              morton_code, parent_chunk_top_height, node_height);
      HashedChunkedWaveletOctree::NodeChunkType* child_chunk =
          parent_chunk.getChild(linear_child_index);
      if (!child_chunk) {
        child_chunk = parent_chunk.allocateChild(linear_child_index);
      }
      constexpr LinearIndex kValueIndex = 0u;
      stack.emplace(
          StackElement{*child_chunk, node_index, 0,
                       HashedChunkedWaveletOctree::Transform::backward(
                           {node_value, child_chunk->data(kValueIndex)})});
    }
  }
}
}  // namespace wavemap
