#include "wavemap/integrator/projective/coarse_to_fine/hashed_wavelet_integrator.h"

namespace wavemap {
void HashedWaveletIntegrator::updateMap() {
  // Update the range image intersector
  range_image_intersector_ = std::make_shared<RangeImageIntersector>(
      posed_range_image_, projection_model_, *measurement_model_,
      config_.min_range, config_.max_range);

  // Find all the indices of blocks that need updating
  std::vector<BlockList> job_lists;
  const auto [fov_min_idx, fov_max_idx] =
      getFovMinMaxIndices(posed_range_image_->getOrigin());
  for (const auto& block_index :
       Grid(fov_min_idx.position, fov_max_idx.position)) {
    auto& thread_job_list = job_lists.emplace_back();
    recursiveTester(OctreeIndex{fov_min_idx.height, block_index},
                    thread_job_list);
  }
  thread_pool_.wait_all();

  // Update the blocks
  for (const auto& job_list : job_lists) {
    for (const auto& block_node_index : job_list) {
      // Make sure the block has been allocated
      occupancy_map_->getOrAllocateBlock(block_node_index.position);

      // Update it in the threadpool
      thread_pool_.add_task([this, block_node_index]() {
        // Recursively update all relevant cells
        auto& block = occupancy_map_->getBlock(block_node_index.position);
        HashedWaveletOctree::Coefficients::CoefficientsArray
            child_scale_coefficient_updates;
        for (NdtreeIndexRelativeChild relative_child_idx = 0;
             relative_child_idx < OctreeIndex::kNumChildren;
             ++relative_child_idx) {
          const OctreeIndex& child_index =
              block_node_index.computeChildIndex(relative_child_idx);
          child_scale_coefficient_updates[relative_child_idx] =
              recursiveSamplerCompressor(child_index, block.getRootScale(),
                                         block.getRootNode(),
                                         relative_child_idx);
        }
        const auto [scale_update, detail_updates] =
            HashedWaveletOctree::Transform::forward(
                child_scale_coefficient_updates);
        block.getRootNode().data() += detail_updates;
        block.getRootScale() += scale_update;
      });
    }
  }
  thread_pool_.wait_all();
}

std::pair<OctreeIndex, OctreeIndex>
HashedWaveletIntegrator::getFovMinMaxIndices(
    const Point3D& sensor_origin) const {
  const IndexElement height =
      1 + std::max(static_cast<IndexElement>(std::ceil(
                       std::log2(config_.max_range / min_cell_width_))),
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
}  // namespace wavemap
