#ifndef WAVEMAP_2D_INTEGRATOR_PROJECTIVE_IMPL_SCANWISE_INTEGRATOR_2D_INL_H_
#define WAVEMAP_2D_INTEGRATOR_PROJECTIVE_IMPL_SCANWISE_INTEGRATOR_2D_INL_H_

namespace wavemap {
inline FloatingPoint ScanwiseIntegrator2D::computeUpdate(
    const Point2D& C_cell_center) const {
  const FloatingPoint d_C_cell = C_cell_center.norm();
  if (d_C_cell < config_.min_range || config_.max_range < d_C_cell) {
    return 0.f;
  }

  const FloatingPoint azimuth_C_cell =
      CircularProjector::bearingToAngle(C_cell_center);
  FloatingPoint angle_remainder;
  const IndexElement idx =
      projection_model_.angleToNearestIndex(azimuth_C_cell, angle_remainder);
  if (idx < 0 || posed_range_image_->getNumBeams() <= idx) {
    return 0.f;
  }

  const FloatingPoint measured_distance = posed_range_image_->getRange(idx);
  if (measured_distance + measurement_model_.getRangeThresholdBehindSurface() <
      d_C_cell) {
    return 0.f;
  }

  const FloatingPoint cell_to_beam_angle = std::abs(angle_remainder);
  if (measurement_model_.getAngleThreshold() < cell_to_beam_angle) {
    return 0.f;
  }

  return measurement_model_.computeUpdate(d_C_cell, cell_to_beam_angle,
                                          measured_distance);
}
}  // namespace wavemap

#endif  // WAVEMAP_2D_INTEGRATOR_PROJECTIVE_IMPL_SCANWISE_INTEGRATOR_2D_INL_H_
