#ifndef WAVEMAP_3D_INTEGRATOR_PROJECTIVE_IMPL_SCANWISE_INTEGRATOR_3D_INL_H_
#define WAVEMAP_3D_INTEGRATOR_PROJECTIVE_IMPL_SCANWISE_INTEGRATOR_3D_INL_H_

namespace wavemap {
inline FloatingPoint ScanwiseIntegrator3D::computeUpdate(
    const Point3D& C_cell_center) const {
  const FloatingPoint d_C_cell = C_cell_center.norm();
  if (d_C_cell < config_.min_range || config_.max_range < d_C_cell) {
    return 0.f;
  }

  const Vector2D spherical_C_cell =
      SphericalProjector::bearingToSphericalApprox(C_cell_center);
  const Index2D image_idx =
      projection_model_.sphericalToNearestIndex(spherical_C_cell);
  if ((image_idx.array() < 0).any() ||
      (posed_range_image_->getDimensions().array() <= image_idx.array())
          .any()) {
    return 0.f;
  }

  const FloatingPoint measured_distance =
      posed_range_image_->getRange(image_idx);
  if (measured_distance + measurement_model_.getRangeThresholdBehindSurface() <
      d_C_cell) {
    return 0.f;
  }

  // Calculate the angle w.r.t. the beam
  // NOTE: The cell is only affected by the measurement if the cell to beam
  //       angle is below the measurement model's angle threshold, which is very
  //       small. When computing the cell to beam angle using
  //       cos(angle)=dot(C_cell,C_beam)/(|C_cell|*|C_beam|), we can therefore
  //       use the small angle approximation instead of std::acos().
  const Vector3D& bearing = bearing_image_.getBearing(image_idx);
  const FloatingPoint cell_beam_projection =
      C_cell_center.dot(bearing) / d_C_cell;
  const FloatingPoint one_minus_cell_beam_projection =
      1.f - cell_beam_projection;
  FloatingPoint cell_to_beam_angle = 0.f;
  if (0.f < one_minus_cell_beam_projection) {
    cell_to_beam_angle = std::sqrt(2.f * one_minus_cell_beam_projection);
    if (measurement_model_.getAngleThreshold() < cell_to_beam_angle) {
      return 0.f;
    }
  }

  return measurement_model_.computeUpdate(d_C_cell, cell_to_beam_angle,
                                          measured_distance);
}
}  // namespace wavemap

#endif  // WAVEMAP_3D_INTEGRATOR_PROJECTIVE_IMPL_SCANWISE_INTEGRATOR_3D_INL_H_
