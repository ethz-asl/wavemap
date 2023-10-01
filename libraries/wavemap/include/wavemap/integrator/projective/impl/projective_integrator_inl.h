#ifndef WAVEMAP_INTEGRATOR_PROJECTIVE_IMPL_PROJECTIVE_INTEGRATOR_INL_H_
#define WAVEMAP_INTEGRATOR_PROJECTIVE_IMPL_PROJECTIVE_INTEGRATOR_INL_H_

namespace wavemap {
inline FloatingPoint ProjectiveIntegrator::computeUpdate(
    const Point3D& C_cell_center) const {
  const auto sensor_coordinates =
      projection_model_->cartesianToSensor(C_cell_center);

  // Check if we're outside the min/max range
  // NOTE: For spherical (e.g. LiDAR) projection models, sensor_coordinates.z()
  //       corresponds to the range, whereas for camera models it corresponds to
  //       the depth.
  if (sensor_coordinates.normal < config_.min_range ||
      config_.max_range < sensor_coordinates.normal) {
    return 0.f;
  }

  return measurement_model_->computeUpdate(sensor_coordinates);
}
}  // namespace wavemap

#endif  // WAVEMAP_INTEGRATOR_PROJECTIVE_IMPL_PROJECTIVE_INTEGRATOR_INL_H_
