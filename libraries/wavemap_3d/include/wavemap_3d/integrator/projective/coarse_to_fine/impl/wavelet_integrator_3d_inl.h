#ifndef WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_WAVELET_INTEGRATOR_3D_INL_H_
#define WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_WAVELET_INTEGRATOR_3D_INL_H_

#include <wavemap_common/data_structure/volumetric/cell_types/occupancy_cell.h>

namespace wavemap {
inline bool WaveletIntegrator3D::isApproximationErrorAcceptable(
    IntersectionType intersection_type, FloatingPoint sphere_center_distance,
    FloatingPoint bounding_sphere_radius) const {
  switch (intersection_type) {
    case IntersectionType::kFreeOrUnknown:
      return bounding_sphere_radius < (kMaxAcceptableUpdateError /
                                       max_gradient_over_range_fully_inside_) *
                                          sphere_center_distance;
    case IntersectionType::kPossiblyOccupied:
      return bounding_sphere_radius <
             kMaxAcceptableUpdateError / max_gradient_on_boundary_;
    default:
      return true;
  }
}

inline FloatingPoint WaveletIntegrator3D::recursiveSamplerCompressor(  // NOLINT
    const OctreeIndex& node_index, FloatingPoint node_value,
    typename WaveletOctreeInterface::NodeType& parent_node,
    OctreeIndex::RelativeChild relative_child_index,
    RangeImage2DIntersector::Cache cache) {
  constexpr FloatingPoint kNoiseThreshold = 1e-4f;

  // If we're at the leaf level, directly update the node
  if (node_index.height == 0) {
    const Point3D W_node_center =
        convert::nodeIndexToCenterPoint(node_index, min_cell_width_);
    const Point3D C_node_center =
        posed_range_image_->getPoseInverse() * W_node_center;
    const FloatingPoint sample = computeUpdate(C_node_center);
    return std::clamp(sample + node_value,
                      SaturatingOccupancyCell::kLowerBound - kNoiseThreshold,
                      SaturatingOccupancyCell::kUpperBound + kNoiseThreshold) -
           node_value;
  }

  // Otherwise, test whether the current node is fully occupied;
  // free or unknown; or fully unknown
  const AABB<Point3D> W_cell_aabb =
      convert::nodeIndexToAABB(node_index, min_cell_width_);
  const IntersectionType intersection_type =
      range_image_intersector_->determineIntersectionType(
          posed_range_image_->getPose(), W_cell_aabb, cache);

  // If we're fully in unknown space,
  // there's no need to evaluate this node or its children
  if (intersection_type == IntersectionType::kFullyUnknown) {
    return 0.f;
  }

  // We can also stop here if the cell will result in a free space update (or
  // zero) and the map is already saturated free
  if (intersection_type == IntersectionType::kFreeOrUnknown &&
      node_value <
          SaturatingOccupancyCell::kLowerBound + kNoiseThreshold / 10.f) {
    return 0.f;
  }

  // Test if the worst-case error for the intersection type at the current
  // resolution falls within the acceptable approximation error
  const FloatingPoint node_width = W_cell_aabb.width<0>();
  const Point3D W_node_center =
      W_cell_aabb.min + Vector3D::Constant(node_width / 2.f);
  const Point3D C_node_center =
      posed_range_image_->getPoseInverse() * W_node_center;
  const FloatingPoint d_C_cell =
      projection_model_->cartesianToSensor(C_node_center).z();
  const FloatingPoint bounding_sphere_radius =
      kUnitCubeHalfDiagonal * node_width;
  WaveletOctreeInterface::NodeType* node =
      parent_node.getChild(relative_child_index);
  if (isApproximationErrorAcceptable(intersection_type, d_C_cell,
                                     bounding_sphere_radius)) {
    const FloatingPoint sample = computeUpdate(C_node_center);
    if (!node || !node->hasAtLeastOneChild()) {
      return std::clamp(
                 sample + node_value,
                 SaturatingOccupancyCell::kLowerBound - kNoiseThreshold,
                 SaturatingOccupancyCell::kUpperBound + kNoiseThreshold) -
             node_value;
    } else {
      return sample;
    }
  }

  // Since the approximation error would still be too big, refine
  if (!node) {
    // Allocate the current node if it has not yet been allocated
    node = parent_node.allocateChild(relative_child_index);
  }
  const WaveletOctreeInterface::Coefficients::CoefficientsArray
      child_scale_coefficients = WaveletOctreeInterface::Transform::backward(
          {node_value, node->data()});
  WaveletOctreeInterface::Coefficients::CoefficientsArray
      child_scale_coefficient_updates;
  for (OctreeIndex::RelativeChild relative_child_idx = 0;
       relative_child_idx < OctreeIndex::kNumChildren; ++relative_child_idx) {
    const OctreeIndex child_index =
        node_index.computeChildIndex(relative_child_idx);
    const FloatingPoint child_value =
        child_scale_coefficients[relative_child_idx];
    child_scale_coefficient_updates[relative_child_idx] =
        recursiveSamplerCompressor(child_index, child_value, *node,
                                   relative_child_idx, cache);
  }

  // Update the current node's wavelet detail coefficients
  const auto [scale_update, detail_updates] =
      WaveletOctreeInterface::Transform::forward(
          child_scale_coefficient_updates);
  node->data() += detail_updates;

  // Propagate the wavelet scale coefficient upward
  return scale_update;
}
}  // namespace wavemap

#endif  // WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_WAVELET_INTEGRATOR_3D_INL_H_
