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
  // NOTE: For spherical models, the cell-to-beam offset corresponds to the
  //       cell-to-beam angle (i.e. angle between the ray from the sensor to the
  //       cell and the ray from the sensor to the beam end-point). For camera
  //       models, it corresponds to the offset in pixel space.
  //       Note that the cell-to-beam angle would normally be computed with
  //       angle = acos(dot(C_beam, C_cell) / (|C_beam|*|C_cell|)). However, the
  //       angle is guaranteed to be very small since the beam and cell-ray map
  //       to the same (narrow) single pixel interval by construction. We can
  //       therefore compute the cell-to-beam angle by directly taking the norm
  //       of the elevation and azimuth angle differences. Roughly speaking,
  //       this is because we operate in such a small neighborhood in spherical
  //       coordinates that it's well approximated by its tangent plane.
  const Vector2D& beam_offset = bearing_image_.getBeamOffset(image_idx);
  const FloatingPoint cell_to_beam_offset = (beam_offset - cell_offset).norm();
  if (measurement_model_.getAngleThreshold() < cell_to_beam_offset) {
    return 0.f;
  }

  if (sensor_coordinates[2] <
      measured_distance -
          measurement_model_.getRangeThresholdInFrontOfSurface()) {
    return measurement_model_.computeFreeSpaceUpdate(cell_to_beam_offset);
  } else {
    return measurement_model_.computeUpdate(
        sensor_coordinates[2], cell_to_beam_offset, measured_distance);
  }
}
}  // namespace wavemap

#endif  // WAVEMAP_3D_INTEGRATOR_PROJECTIVE_IMPL_SCANWISE_INTEGRATOR_3D_INL_H_
