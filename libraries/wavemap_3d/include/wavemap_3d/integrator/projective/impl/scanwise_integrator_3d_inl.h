#ifndef WAVEMAP_3D_INTEGRATOR_PROJECTIVE_IMPL_SCANWISE_INTEGRATOR_3D_INL_H_
#define WAVEMAP_3D_INTEGRATOR_PROJECTIVE_IMPL_SCANWISE_INTEGRATOR_3D_INL_H_

namespace wavemap {
inline FloatingPoint ScanwiseIntegrator3D::computeUpdate(
    const Point3D& C_cell_center) const {
  const Vector3D sensor_coordinates =
      projection_model_->cartesianToSensor(C_cell_center);

  // Check if we're outside the min/max range
  // NOTE: For spherical (e.g. LiDAR) projection models, sensor_coordinates.z()
  //       corresponds to the range, whereas for camera models it corresponds to
  //       the depth.
  if (sensor_coordinates.z() < config_.min_range ||
      config_.max_range < sensor_coordinates.z()) {
    return 0.f;
  }

  const auto [image_indices, cell_offsets] =
      projection_model_->imageToNearestIndicesAndOffsets(
          sensor_coordinates.head<2>());

  FloatingPoint update = 0.f;
  for (int neighbor_idx = 0; neighbor_idx < 4; ++neighbor_idx) {
    const Index2D& image_idx = image_indices[neighbor_idx];
    const Vector2D& cell_offset = cell_offsets[neighbor_idx];

    if (!posed_range_image_->isIndexWithinBounds(image_idx)) {
      continue;
    }

    const FloatingPoint measured_distance =
        posed_range_image_->getRange(image_idx);
    if (measured_distance +
            measurement_model_.getRangeThresholdBehindSurface() <
        sensor_coordinates.z()) {
      continue;
    }

    // Calculate the error norm between the beam and cell projected into the
    // image NOTE: For spherical (e.g. LiDAR) projection models, the error norm
    //       corresponds to the relative angle between the beam and the ray
    //       through the cell, whereas for camera models it corresponds to the
    //       reprojection error in pixels.
    const Vector2D cell_to_beam_offset =
        beam_offset_image_.getBeamOffset(image_idx) - cell_offset;
    const FloatingPoint cell_to_beam_image_error_norm =
        projection_model_->imageOffsetToErrorNorm(sensor_coordinates.head<2>(),
                                                  cell_to_beam_offset);
    if (measurement_model_.getAngleThreshold() <
        cell_to_beam_image_error_norm) {
      continue;
    }

    if (sensor_coordinates.z() <
        measured_distance -
            measurement_model_.getRangeThresholdInFrontOfSurface()) {
      update += measurement_model_.computeFreeSpaceUpdate(
          cell_to_beam_image_error_norm);
    } else {
      update += measurement_model_.computeUpdate(sensor_coordinates.z(),
                                                 cell_to_beam_image_error_norm,
                                                 measured_distance);
    }
  }

  return update;
}
}  // namespace wavemap

#endif  // WAVEMAP_3D_INTEGRATOR_PROJECTIVE_IMPL_SCANWISE_INTEGRATOR_3D_INL_H_
