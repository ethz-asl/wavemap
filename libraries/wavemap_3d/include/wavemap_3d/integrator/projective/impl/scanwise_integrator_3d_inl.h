#ifndef WAVEMAP_3D_INTEGRATOR_PROJECTIVE_IMPL_SCANWISE_INTEGRATOR_3D_INL_H_
#define WAVEMAP_3D_INTEGRATOR_PROJECTIVE_IMPL_SCANWISE_INTEGRATOR_3D_INL_H_

namespace wavemap {
inline FloatingPoint ScanwiseIntegrator3D::computeUpdate(
    const Point3D& C_cell_center) const {
  const Vector3D sensor_coordinates =
      projection_model_.cartesianToSensor(C_cell_center);

  // Check if we're outside the min/max range
  // NOTE: For spherical (e.g. LiDAR) projection models, sensor_coordinates[2]
  //       corresponds to the range, whereas for camera models it corresponds to
  //       the depth.
  if (sensor_coordinates[2] < config_.min_range ||
      config_.max_range < sensor_coordinates[2]) {
    return 0.f;
  }

  const Index2D image_idx =
      projection_model_.imageToNearestIndex(sensor_coordinates.head<2>());
  if (!posed_range_image_->isIndexWithinBounds(image_idx)) {
    return 0.f;
  }

  const FloatingPoint measured_distance =
      posed_range_image_->getRange(image_idx);
  if (measured_distance + measurement_model_.getRangeThresholdBehindSurface() <
      sensor_coordinates[2]) {
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
      C_cell_center.dot(bearing) / sensor_coordinates[2];
  const FloatingPoint one_minus_cell_beam_projection =
      1.f - cell_beam_projection;
  FloatingPoint cell_to_beam_angle = 0.f;
  if (0.f < one_minus_cell_beam_projection) {
    cell_to_beam_angle = std::sqrt(2.f * one_minus_cell_beam_projection);
    if (measurement_model_.getAngleThreshold() < cell_to_beam_angle) {
      return 0.f;
    }
  }

  if (sensor_coordinates[2] <
      measured_distance -
          measurement_model_.getRangeThresholdInFrontOfSurface()) {
    return measurement_model_.computeFreeSpaceUpdate(cell_to_beam_angle);
  } else {
    return measurement_model_.computeUpdate(
        sensor_coordinates[2], cell_to_beam_angle, measured_distance);
  }
}
}  // namespace wavemap

#endif  // WAVEMAP_3D_INTEGRATOR_PROJECTIVE_IMPL_SCANWISE_INTEGRATOR_3D_INL_H_
