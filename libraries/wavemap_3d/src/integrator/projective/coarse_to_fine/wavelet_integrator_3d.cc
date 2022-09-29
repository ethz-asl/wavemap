#include "wavemap_3d/integrator/projective/coarse_to_fine/wavelet_integrator_3d.h"

namespace wavemap {
WaveletIntegrator3D::WaveletIntegrator3D(
    VolumetricDataStructure3D::Ptr occupancy_map)
    : ScanwiseIntegrator3D(std::move(occupancy_map)),
      min_cell_width_(occupancy_map_->getMinCellWidth()),
      posed_range_image_(
          std::make_shared<PosedRangeImage2D>(spherical_projector_)) {
  // Get a pointer to the underlying specialized octree data structure
  wavelet_tree_ = dynamic_cast<WaveletOctreeInterface*>(occupancy_map_.get());
  CHECK(wavelet_tree_) << "Wavelet integrator can only be used with "
                          "volumetric data structures based on wavelet trees.";
}

void WaveletIntegrator3D::integratePointcloud(
    const PosedPointcloud<Point3D>& pointcloud) {
  if (!isPointcloudValid(pointcloud)) {
    return;
  }

  // Compute the range image and the scan's AABB
  posed_range_image_->importPointcloud(pointcloud, spherical_projector_);
  range_image_intersector_ =
      std::make_shared<RangeImage2DIntersector>(posed_range_image_);

  // Recursively update all relevant cells
  const auto first_child_indices = wavelet_tree_->getFirstChildIndices();
  WaveletOctreeInterface::Coefficients::CoefficientsArray
      child_scale_coefficient_updates;
  for (OctreeIndex::RelativeChild relative_child_idx = 0;
       relative_child_idx < OctreeIndex::kNumChildren; ++relative_child_idx) {
    const OctreeIndex& child_index = first_child_indices[relative_child_idx];
    child_scale_coefficient_updates[relative_child_idx] =
        recursiveSamplerCompressor(child_index, wavelet_tree_->getRootScale(),
                                   wavelet_tree_->getRootNode(),
                                   relative_child_idx,
                                   RangeImage2DIntersector::Cache{});
  }
  const auto [scale_update, detail_updates] =
      WaveletOctreeInterface::Transform::forward(
          child_scale_coefficient_updates);
  wavelet_tree_->getRootNode().data() += detail_updates;
  wavelet_tree_->getRootScale() += scale_update;
}
}  // namespace wavemap
