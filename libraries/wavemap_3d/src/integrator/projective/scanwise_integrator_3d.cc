#include "wavemap_3d/integrator/projective/scanwise_integrator_3d.h"

namespace wavemap {
void ScanwiseIntegrator3D::updateRangeImage(
    const PosedPointcloud<Point3D>& pointcloud,
    PosedRangeImage2D& posed_range_image, BearingImage2D& bearing_image) const {
  posed_range_image.resetToInitialValue();
  posed_range_image.setPose(pointcloud.getPose());
  for (const auto& C_point : pointcloud.getPointsLocal()) {
    // Filter out noisy points and compute point's range
    const FloatingPoint range = C_point.norm();
    if (!isMeasurementValid(C_point, range)) {
      continue;
    }

    // Calculate the range image index
    const Index2D range_image_index =
        projection_model_.bearingToNearestIndex(C_point);
    if ((range_image_index.array() < 0).any() ||
        (posed_range_image.getDimensions().array() <= range_image_index.array())
            .any()) {
      // Prevent out-of-bounds access
      continue;
    }

    // Add the point to the range image, if multiple points hit the same image
    // pixel, keep the closest point
    const FloatingPoint old_range_value =
        posed_range_image.getRange(range_image_index);
    if (old_range_value < config_.min_range || range < old_range_value) {
      posed_range_image.getRange(range_image_index) = range;
      bearing_image.getBearing(range_image_index) = C_point / range;
    }
  }
}
}  // namespace wavemap
