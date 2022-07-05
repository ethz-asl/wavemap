#ifndef WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_COARSE_TO_FINE_IMPL_WAVELET_INTEGRATOR_INL_H_
#define WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_COARSE_TO_FINE_IMPL_WAVELET_INTEGRATOR_INL_H_

namespace wavemap {
inline bool WaveletIntegrator::isApproximationErrorAcceptable(
    RangeImageIntersector::IntersectionType intersection_type,
    FloatingPoint sphere_center_distance,
    FloatingPoint bounding_sphere_radius) {
  switch (intersection_type) {
    case RangeImageIntersector::IntersectionType::kFreeOrUnknown:
      return bounding_sphere_radius / sphere_center_distance <
             kMaxAcceptableUpdateError / kMaxGradientOverRangeFullyInside;
    case RangeImageIntersector::IntersectionType::kPossiblyOccupied:
      return bounding_sphere_radius <
             kMaxAcceptableUpdateError / kMaxGradientOnBoundary;
    default:
      return true;
  }
}

inline FloatingPoint WaveletIntegrator::sampleUpdateAtPoint(
    const RangeImage& range_image, FloatingPoint d_C_cell,
    FloatingPoint azimuth_angle_C_cell) {
  if (d_C_cell < kEpsilon || BeamModel::kRangeMax < d_C_cell) {
    return 0.f;
  }

  const auto idx = range_image.angleToNearestIndex(azimuth_angle_C_cell);
  if (idx < 0 || range_image.getNumBeams() <= idx) {
    return 0.f;
  }
  const FloatingPoint measured_distance = range_image[idx];
  if (measured_distance + BeamModel::kRangeDeltaThresh < d_C_cell) {
    return 0.f;
  }

  const FloatingPoint beam_azimuth_angle = range_image.indexToAngle(idx);
  const FloatingPoint cell_to_beam_angle =
      std::abs(azimuth_angle_C_cell - beam_azimuth_angle);
  if (BeamModel::kAngleThresh < cell_to_beam_angle) {
    return 0.f;
  }

  return BeamModel::computeUpdate(d_C_cell, cell_to_beam_angle,
                                  measured_distance);
}

FloatingPoint WaveletIntegrator::recursiveSamplerCompressor(  // NOLINT
    const QuadtreeIndex& node_index,
    typename WaveletTreeInterface::NodeType& parent_node,
    QuadtreeIndex::RelativeChild relative_child_index) {
  const AABB<Point> W_cell_aabb =
      convert::nodeIndexToAABB(node_index, min_cell_width_);
  const RangeImageIntersector::IntersectionType intersection_type =
      range_image_intersector_->determineIntersectionType(
          posed_range_image_->getPose(), W_cell_aabb);
  if (intersection_type ==
      RangeImageIntersector::IntersectionType::kFullyUnknown) {
    return 0.f;
  }

  const FloatingPoint node_width = W_cell_aabb.width<0>();
  const Point W_node_center =
      W_cell_aabb.min + Vector::Constant(node_width / 2.f);
  const Point C_node_center =
      posed_range_image_->getPoseInverse() * W_node_center;
  FloatingPoint d_C_cell = C_node_center.norm();
  const FloatingPoint bounding_sphere_radius =
      kUnitCubeHalfDiagonal * node_width;
  if (node_index.height == 0 ||
      isApproximationErrorAcceptable(intersection_type, d_C_cell,
                                     bounding_sphere_radius)) {
    FloatingPoint angle_C_cell = RangeImage::bearingToAngle(C_node_center);
    return sampleUpdateAtPoint(*posed_range_image_, d_C_cell, angle_C_cell);
  }

  WaveletTreeInterface::NodeType* node =
      parent_node.getChild(relative_child_index);
  if (!node) {
    node = parent_node.allocateChild(relative_child_index);
  }

  WaveletTreeInterface::ChildScaleCoefficients child_scale_coefficient_updates;
  for (QuadtreeIndex::RelativeChild relative_child_idx = 0;
       relative_child_idx < QuadtreeIndex::kNumChildren; ++relative_child_idx) {
    const QuadtreeIndex child_index =
        node_index.computeChildIndex(relative_child_idx);
    child_scale_coefficient_updates[relative_child_idx] =
        recursiveSamplerCompressor(child_index, *node, relative_child_idx);
  }

  const auto [scale_update, detail_updates] =
      WaveletTreeInterface::HaarWaveletType::forward(
          child_scale_coefficient_updates);
  node->data() += detail_updates;

  return scale_update;
}
}  // namespace wavemap

#endif  // WAVEMAP_2D_INTEGRATOR_SCAN_INTEGRATOR_COARSE_TO_FINE_IMPL_WAVELET_INTEGRATOR_INL_H_
