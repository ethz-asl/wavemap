#include "wavemap/integrator/projective/coarse_to_fine/hashed_wavelet_integrator.h"

namespace wavemap {
void HashedWaveletIntegrator::updateMap() {
  // Update the range image intersector
  range_image_intersector_ = std::make_shared<RangeImageIntersector>(
      posed_range_image_, projection_model_, *measurement_model_,
      config_.min_range, config_.max_range);

  const IndexElement tree_height = occupancy_map_->getTreeHeight();
  const Index3D fov_min_idx = int_math::div_exp2_floor(
      convert::pointToFloorIndex<3>(posed_range_image_->getOrigin() -
                                        Vector3D::Constant(config_.max_range),
                                    1.f / min_cell_width_),
      tree_height);
  const Index3D fov_max_idx = int_math::div_exp2_floor(
      convert::pointToCeilIndex<3>(posed_range_image_->getOrigin() +
                                       Vector3D::Constant(config_.max_range),
                                   1.f / min_cell_width_),
      tree_height);
  for (const auto& block_index : Grid(fov_min_idx, fov_max_idx)) {
    // Check if the block needs updating
    const OctreeIndex block_node_index{tree_height, block_index};
    const auto block_aabb =
        convert::nodeIndexToAABB(block_node_index, min_cell_width_);
    const auto block_update_type =
        range_image_intersector_->determineUpdateType(
            block_aabb, posed_range_image_->getRotationMatrixInverse(),
            posed_range_image_->getOrigin());
    if (block_update_type == UpdateType::kFullyUnobserved) {
      continue;
    }

    // Get the block
    auto& block = occupancy_map_->getBlock(block_index);

    // Recursively update all relevant cells
    HashedWaveletOctree::Coefficients::CoefficientsArray
        child_scale_coefficient_updates;
    for (NdtreeIndexRelativeChild relative_child_idx = 0;
         relative_child_idx < OctreeIndex::kNumChildren; ++relative_child_idx) {
      const OctreeIndex& child_index =
          block_node_index.computeChildIndex(relative_child_idx);
      child_scale_coefficient_updates[relative_child_idx] =
          recursiveSamplerCompressor(child_index, block.getRootScale(),
                                     block.getRootNode(), relative_child_idx);
    }
    const auto [scale_update, detail_updates] =
        HashedWaveletOctree::Transform::forward(
            child_scale_coefficient_updates);
    block.getRootNode().data() += detail_updates;
    block.getRootScale() += scale_update;
  }
}
}  // namespace wavemap
