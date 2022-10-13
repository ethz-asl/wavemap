#ifndef WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_WAVELET_INTEGRATOR_2D_INL_H_
#define WAVEMAP_2D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_WAVELET_INTEGRATOR_2D_INL_H_

namespace wavemap {
inline bool WaveletIntegrator2D::isApproximationErrorAcceptable(
    IntersectionType intersection_type, FloatingPoint sphere_center_distance,
    FloatingPoint bounding_sphere_radius) const {
  switch (intersection_type) {
    case IntersectionType::kFreeOrUnknown:
      return bounding_sphere_radius / sphere_center_distance <
             kMaxAcceptableUpdateError / max_gradient_over_range_fully_inside_;
    case IntersectionType::kPossiblyOccupied:
      return bounding_sphere_radius <
             kMaxAcceptableUpdateError / max_gradient_on_boundary_;
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
  const IntersectionType intersection_type =
      range_image_intersector_->determineIntersectionType(
          posed_range_image_->getPose(), W_cell_aabb, projection_model_);
  if (intersection_type == IntersectionType::kFullyUnknown) {
    return 0.f;
  }

  const FloatingPoint node_width = W_cell_aabb.width<0>();
  const Point2D W_node_center =
      W_cell_aabb.min + Vector2D::Constant(node_width / 2.f);
  const Point2D C_node_center =
      posed_range_image_->getPoseInverse() * W_node_center;
  FloatingPoint d_C_cell = C_node_center.norm();
  const FloatingPoint bounding_sphere_radius =
      kUnitSquareHalfDiagonal * node_width;
  if (node_index.height == 0 ||
      isApproximationErrorAcceptable(intersection_type, d_C_cell,
                                     bounding_sphere_radius)) {
    const FloatingPoint angle_C_cell =
        CircularProjector::bearingToAngle(C_node_center);
    const FloatingPoint sample = computeUpdate(d_C_cell, angle_C_cell);
    return sample;
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
