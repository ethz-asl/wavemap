#ifndef WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_WAVELET_INTEGRATOR_2D_INL_H_
#define WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_WAVELET_INTEGRATOR_2D_INL_H_

namespace wavemap {
inline bool WaveletIntegrator2D::isApproximationErrorAcceptable(
    RangeImage1DIntersector::IntersectionType intersection_type,
    FloatingPoint sphere_center_distance,
    FloatingPoint bounding_sphere_radius) {
  switch (intersection_type) {
    case RangeImage1DIntersector::IntersectionType::kFreeOrUnknown:
      return bounding_sphere_radius / sphere_center_distance <
             kMaxAcceptableUpdateError / kMaxGradientOverRangeFullyInside;
    case RangeImage1DIntersector::IntersectionType::kPossiblyOccupied:
      return bounding_sphere_radius <
             kMaxAcceptableUpdateError / kMaxGradientOnBoundary;
    default:
      return true;
  }
}

inline FloatingPoint WaveletIntegrator2D::recursiveSamplerCompressor(  // NOLINT
    const QuadtreeIndex& node_index,
    typename WaveletQuadtreeInterface::NodeType& parent_node,
    QuadtreeIndex::RelativeChild relative_child_index) {
  const AABB<Point2D> W_cell_aabb =
      convert::nodeIndexToAABB(node_index, min_cell_width_);
  const RangeImage1DIntersector::IntersectionType intersection_type =
      range_image_intersector_->determineIntersectionType(
          posed_range_image_->getPose(), W_cell_aabb, circular_projector_);
  if (intersection_type ==
      RangeImage1DIntersector::IntersectionType::kFullyUnknown) {
    return 0.f;
  }

  const FloatingPoint node_width = W_cell_aabb.width<0>();
  const Point2D W_node_center =
      W_cell_aabb.min + Vector2D::Constant(node_width / 2.f);
  const Point2D C_node_center =
      posed_range_image_->getPoseInverse() * W_node_center;
  FloatingPoint d_C_cell = C_node_center.norm();
  const FloatingPoint bounding_sphere_radius =
      kUnitCubeHalfDiagonal * node_width;
  if (node_index.height == 0 ||
      isApproximationErrorAcceptable(intersection_type, d_C_cell,
                                     bounding_sphere_radius)) {
    FloatingPoint angle_C_cell =
        CircularProjector::bearingToAngle(C_node_center);
    return computeUpdate(*posed_range_image_, d_C_cell, angle_C_cell);
  }

  WaveletQuadtreeInterface::NodeType* node =
      parent_node.getChild(relative_child_index);
  if (!node) {
    node = parent_node.allocateChild(relative_child_index);
  }

  WaveletQuadtreeInterface::Coefficients::CoefficientsArray
      child_scale_coefficient_updates;
  for (QuadtreeIndex::RelativeChild relative_child_idx = 0;
       relative_child_idx < QuadtreeIndex::kNumChildren; ++relative_child_idx) {
    const QuadtreeIndex child_index =
        node_index.computeChildIndex(relative_child_idx);
    child_scale_coefficient_updates[relative_child_idx] =
        recursiveSamplerCompressor(child_index, *node, relative_child_idx);
  }

  const auto [scale_update, detail_updates] =
      WaveletQuadtreeInterface::Transform::forward(
          child_scale_coefficient_updates);
  node->data() += detail_updates;

  return scale_update;
}
}  // namespace wavemap

#endif  // WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_WAVELET_INTEGRATOR_2D_INL_H_