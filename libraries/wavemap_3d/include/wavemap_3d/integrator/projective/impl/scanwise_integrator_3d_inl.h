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

  // Calculate the error norm between the beam and cell projected into the image
  // NOTE: For spherical (e.g. LiDAR) projection models, the error norm
  //       corresponds to the relative angle between the beam and the ray
  //       through the cell, whereas for camera models it corresponds to the
  //       reprojection error in pixels.
  const Vector2D cell_to_beam_offset =
      bearing_image_.getBeamOffset(image_idx) - cell_offset;
  const FloatingPoint cell_to_beam_image_error_norm =
      projection_model_.imageOffsetToErrorNorm(sensor_coordinates.head<2>(),
                                               cell_to_beam_offset);
  if (measurement_model_.getAngleThreshold() < cell_to_beam_image_error_norm) {
    return 0.f;
  }

  if (sensor_coordinates[2] <
      measured_distance -
          measurement_model_.getRangeThresholdInFrontOfSurface()) {
    return measurement_model_.computeFreeSpaceUpdate(
        cell_to_beam_image_error_norm);
  } else {
    return measurement_model_.computeUpdate(sensor_coordinates[2],
                                            cell_to_beam_image_error_norm,
                                            measured_distance);
  }
}
}  // namespace wavemap

#endif  // WAVEMAP_3D_INTEGRATOR_PROJECTIVE_IMPL_SCANWISE_INTEGRATOR_3D_INL_H_
