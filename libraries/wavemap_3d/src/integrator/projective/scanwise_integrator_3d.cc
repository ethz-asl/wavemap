#include "wavemap_3d/integrator/projective/scanwise_integrator_3d.h"

namespace wavemap {
void ScanwiseIntegrator3D::updateRangeImage(
    const PosedPointcloud<Point3D>& pointcloud,
    PosedRangeImage2D& posed_range_image,
    BeamOffsetImage2D& bearing_image) const {
  posed_range_image.resetToInitialValue();
  posed_range_image.setPose(pointcloud.getPose());
  for (const auto& C_point : pointcloud.getPointsLocal()) {
    // Filter out noisy points and compute point's range
    if (!isMeasurementValid(C_point)) {
      continue;
    }

    // Calculate the range image index
    const Vector3D sensor_coordinates =
        projection_model_->cartesianToSensor(C_point);

    const auto [range_image_index, beam_to_pixel_offset] =
        projection_model_->imageToNearestIndexAndOffset(
            sensor_coordinates.head<2>());
    if (!posed_range_image_->isIndexWithinBounds(range_image_index)) {
      // Prevent out-of-bounds access
      continue;
    }

    // Add the point to the range image, if multiple points hit the same image
    // pixel, keep the closest point
    const FloatingPoint range = sensor_coordinates[2];
    const FloatingPoint old_range_value =
        posed_range_image.getRange(range_image_index);
    if (old_range_value < config_.min_range || range < old_range_value) {
      posed_range_image.getRange(range_image_index) = range;
      bearing_image.getBeamOffset(range_image_index) = beam_to_pixel_offset;
    }
  }
}
}  // namespace wavemap
