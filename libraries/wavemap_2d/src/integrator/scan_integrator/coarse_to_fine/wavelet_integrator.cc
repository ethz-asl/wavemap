#include "wavemap_2d/integrator/scan_integrator/coarse_to_fine/wavelet_integrator.h"

namespace wavemap {
WaveletIntegrator::WaveletIntegrator(VolumetricDataStructure::Ptr occupancy_map)
    : PointcloudIntegrator(std::move(occupancy_map)),
      min_cell_width_(occupancy_map_->getMinCellWidth()) {
  // Get a pointer to the underlying specialized quadtree data structure
  wavelet_tree_ = dynamic_cast<WaveletTreeInterface*>(occupancy_map_.get());
  CHECK(wavelet_tree_) << "Wavelet integrator can only be used with "
                          "volumetric data structures based on wavelet trees.";
}

void WaveletIntegrator::integratePointcloud(
    const PosedPointcloud<Point2D, Transformation2D>& pointcloud) {
  if (!isPointcloudValid(pointcloud)) {
    return;
  }

  // TODO(victorr): Check that the pointcloud's angular resolution is lower
  //                than the angular uncertainty of the beam model. This is
  //                necessary since this measurement integrator assumes the
  //                beams don't overlap, i.e. for each sample point we only
  //                evaluate the contribution from the nearest beam.

  // Compute the range image and the scan's AABB
  // TODO(victorr): Make the FoV and number of beams configurable
  if (!posed_range_image_) {
    posed_range_image_ =
        std::make_shared<PosedRangeImage>(-kHalfPi, kHalfPi, pointcloud.size());
  }
  posed_range_image_->importPointcloud(pointcloud);
  range_image_intersector_ =
      std::make_shared<RangeImageIntersector>(posed_range_image_);

  // Recursively update all relevant cells
  const auto first_child_indices = wavelet_tree_->getFirstChildIndices();
  WaveletTreeInterface::ChildScaleCoefficients child_scale_coefficient_updates;
  for (QuadtreeIndex::RelativeChild relative_child_idx = 0;
       relative_child_idx < QuadtreeIndex::kNumChildren; ++relative_child_idx) {
    const QuadtreeIndex& child_index = first_child_indices[relative_child_idx];
    child_scale_coefficient_updates[relative_child_idx] =
        recursiveSamplerCompressor(child_index, wavelet_tree_->getRootNode(),
                                   relative_child_idx);
  }
  const auto [scale_update, detail_updates] =
      WaveletTreeInterface::HaarWaveletType::forward(
          child_scale_coefficient_updates);
  wavelet_tree_->getRootNode().data() += detail_updates;
  wavelet_tree_->getRootScale() += scale_update;
}
}  // namespace wavemap
