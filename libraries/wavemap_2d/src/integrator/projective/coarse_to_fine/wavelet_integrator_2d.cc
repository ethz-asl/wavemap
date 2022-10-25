#include "wavemap_2d/integrator/projective/coarse_to_fine/wavelet_integrator_2d.h"

namespace wavemap {
WaveletIntegrator2D::WaveletIntegrator2D(
    const PointcloudIntegratorConfig& config,
    CircularProjector projection_model,
    ContinuousVolumetricLogOdds<2> measurement_model,
    VolumetricDataStructure2D::Ptr occupancy_map)
    : ScanwiseIntegrator2D(config, std::move(projection_model),
                           std::move(measurement_model),
                           std::move(occupancy_map)),
      min_cell_width_(occupancy_map_->getMinCellWidth()) {
  // Get a pointer to the underlying specialized quadtree data structure
  wavelet_tree_ =
      std::dynamic_pointer_cast<WaveletQuadtreeInterface>(occupancy_map_);
  CHECK(wavelet_tree_) << "Wavelet integrator can only be used with "
                          "volumetric data structures based on wavelet trees.";
}

void WaveletIntegrator2D::integratePointcloud(
    const PosedPointcloud<Point2D>& pointcloud) {
  if (!isPointcloudValid(pointcloud)) {
    return;
  }

  // Compute the range image and the scan's AABB
  updateRangeImage(pointcloud, *posed_range_image_);
  range_image_intersector_ = std::make_shared<RangeImage1DIntersector>(
      posed_range_image_, config_.max_range,
      measurement_model_.getAngleThreshold(),
      measurement_model_.getRangeThresholdInFrontOfSurface(),
      measurement_model_.getRangeThresholdBehindSurface());

  // Recursively update all relevant cells
  const auto first_child_indices = wavelet_tree_->getFirstChildIndices();
  WaveletQuadtreeInterface::Coefficients::CoefficientsArray
      child_scale_coefficient_updates;
  for (QuadtreeIndex::RelativeChild relative_child_idx = 0;
       relative_child_idx < QuadtreeIndex::kNumChildren; ++relative_child_idx) {
    const QuadtreeIndex& child_index = first_child_indices[relative_child_idx];
    child_scale_coefficient_updates[relative_child_idx] =
        recursiveSamplerCompressor(child_index, wavelet_tree_->getRootNode(),
                                   relative_child_idx);
  }
  const auto [scale_update, detail_updates] =
      WaveletQuadtreeInterface::Transform::forward(
          child_scale_coefficient_updates);
  wavelet_tree_->getRootNode().data() += detail_updates;
  wavelet_tree_->getRootScale() += scale_update;
}
}  // namespace wavemap
