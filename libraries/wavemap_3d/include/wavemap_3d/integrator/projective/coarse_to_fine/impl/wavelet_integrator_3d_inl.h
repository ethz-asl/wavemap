#ifndef WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_WAVELET_INTEGRATOR_3D_INL_H_
#define WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_WAVELET_INTEGRATOR_3D_INL_H_

namespace wavemap {
inline bool WaveletIntegrator3D::isApproximationErrorAcceptable(
    IntersectionType intersection_type, FloatingPoint sphere_center_distance,
    FloatingPoint bounding_sphere_radius) {
  switch (intersection_type) {
    case IntersectionType::kFreeOrUnknown:
      return bounding_sphere_radius / sphere_center_distance <
             kMaxAcceptableUpdateError / kMaxGradientOverRangeFullyInside;
    case IntersectionType::kPossiblyOccupied:
      return bounding_sphere_radius <
             kMaxAcceptableUpdateError / kMaxGradientOnBoundary;
    default:
      return true;
  }
}

inline FloatingPoint WaveletIntegrator3D::recursiveSamplerCompressor(  // NOLINT
    const OctreeIndex& node_index,
    typename WaveletOctreeInterface::NodeType& parent_node,
    OctreeIndex::RelativeChild relative_child_index) {
  const AABB<Point3D> W_cell_aabb =
      convert::nodeIndexToAABB(node_index, min_cell_width_);
  const IntersectionType intersection_type =
      range_image_intersector_->determineIntersectionType(
          posed_range_image_->getPose(), W_cell_aabb, spherical_projector_);
  if (intersection_type == IntersectionType::kFullyUnknown) {
    return 0.f;
  }

  const FloatingPoint node_width = W_cell_aabb.width<0>();
  const Point3D W_node_center =
      W_cell_aabb.min + Vector3D::Constant(node_width / 2.f);
  const Point3D C_node_center =
      posed_range_image_->getPoseInverse() * W_node_center;
  const FloatingPoint d_C_cell = C_node_center.norm();
  const FloatingPoint bounding_sphere_radius =
      kUnitCubeHalfDiagonal * node_width;
  if (node_index.height == 0 ||
      isApproximationErrorAcceptable(intersection_type, d_C_cell,
                                     bounding_sphere_radius)) {
    const Vector2D spherical_C_cell =
        SphericalProjector::bearingToSpherical(C_node_center);
    const FloatingPoint sample =
        computeUpdate(*posed_range_image_, d_C_cell, spherical_C_cell);
    return sample;
  }

  WaveletOctreeInterface::NodeType* node =
      parent_node.getChild(relative_child_index);
  if (!node) {
    node = parent_node.allocateChild(relative_child_index);
  }

  WaveletOctreeInterface::Coefficients::CoefficientsArray
      child_scale_coefficient_updates;
  for (OctreeIndex::RelativeChild relative_child_idx = 0;
       relative_child_idx < OctreeIndex::kNumChildren; ++relative_child_idx) {
    const OctreeIndex child_index =
        node_index.computeChildIndex(relative_child_idx);
    child_scale_coefficient_updates[relative_child_idx] =
        recursiveSamplerCompressor(child_index, *node, relative_child_idx);
  }

  const auto [scale_update, detail_updates] =
      WaveletOctreeInterface::Transform::forward(
          child_scale_coefficient_updates);
  node->data() += detail_updates;

  return scale_update;
}
}  // namespace wavemap

#endif  // WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_WAVELET_INTEGRATOR_3D_INL_H_
