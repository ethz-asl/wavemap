#include "wavemap/core/integrator/projective/coarse_to_fine/wavelet_integrator.h"

namespace wavemap {
void WaveletIntegrator::updateMap() {
  // Update the range image intersector
  range_image_intersector_ = std::make_shared<RangeImageIntersector>(
      posed_range_image_, projection_model_, *measurement_model_,
      config_.min_range, config_.max_range);

  // Recursively update all relevant cells
  const auto first_child_indices = occupancy_map_->getFirstChildIndices();
  WaveletOctree::Coefficients::CoefficientsArray
      child_scale_coefficient_updates;
  for (NdtreeIndexRelativeChild relative_child_idx = 0;
       relative_child_idx < OctreeIndex::kNumChildren; ++relative_child_idx) {
    const OctreeIndex& child_index = first_child_indices[relative_child_idx];
    child_scale_coefficient_updates[relative_child_idx] =
        recursiveSamplerCompressor(child_index, occupancy_map_->getRootScale(),
                                   occupancy_map_->getRootNode(),
                                   relative_child_idx);
  }
  const auto [scale_update, detail_updates] =
      WaveletOctree::Transform::forward(child_scale_coefficient_updates);
  occupancy_map_->getRootNode().data() += detail_updates;
  occupancy_map_->getRootScale() += scale_update;
}
}  // namespace wavemap
