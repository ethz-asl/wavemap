#include "wavemap_2d/integrator/projective/coarse_to_fine/wavelet_integrator.h"

namespace wavemap {
WaveletIntegrator::WaveletIntegrator(
    VolumetricDataStructure2D::Ptr occupancy_map)
    : ScanwiseIntegrator2D(std::move(occupancy_map)),
      min_cell_width_(occupancy_map_->getMinCellWidth()) {
  // Get a pointer to the underlying specialized quadtree data structure
  wavelet_tree_ = dynamic_cast<WaveletQuadtreeInterface*>(occupancy_map_.get());
  CHECK(wavelet_tree_) << "Wavelet integrator can only be used with "
                          "volumetric data structures based on wavelet trees.";
}

void WaveletIntegrator::integratePointcloud(
    const PosedPointcloud<Point2D>& pointcloud) {
  if (!isPointcloudValid(pointcloud)) {
    return;
  }

  // Compute the range image and the scan's AABB
  if (!posed_range_image_) {
    posed_range_image_ =
        std::make_shared<PosedRangeImage1D>(pointcloud, circular_projector_);
  }
  posed_range_image_->importPointcloud(pointcloud, circular_projector_);
  range_image_intersector_ =
      std::make_shared<RangeImageIntersector>(posed_range_image_);

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
