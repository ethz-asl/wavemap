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

  const auto [image_idx, cell_offset] =
      projection_model_.imageToNearestIndexAndOffset(
          sensor_coordinates.head<2>());
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
  const Vector2D cell_to_beam_offset =
      bearing_image_.getBeamOffset(image_idx) - cell_offset;
  const FloatingPoint cell_to_beam_angle =
      Vector2D(cell_to_beam_offset.x(),
               cell_to_beam_offset.y() * std::cos(sensor_coordinates[0]))
          .norm();
  if (measurement_model_.getAngleThreshold() < cell_to_beam_angle) {
    return 0.f;
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
