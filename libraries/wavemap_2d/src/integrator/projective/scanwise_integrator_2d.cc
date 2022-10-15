#include "wavemap_2d/integrator/projective/scanwise_integrator_2d.h"

namespace wavemap {
void ScanwiseIntegrator2D::updateRangeImage(
    const PosedPointcloud<Point2D>& pointcloud,
    PosedRangeImage1D& posed_range_image) const {
  posed_range_image.resetToInitialValue();
  posed_range_image.setPose(pointcloud.getPose());
  for (const auto& C_point : pointcloud.getPointsLocal()) {
    // Filter out noisy points and compute point's range
    if (C_point.hasNaN()) {
      LOG(WARNING) << "Skipping measurement whose endpoint contains NaNs:\n"
                   << C_point;
      continue;
    }
    const FloatingPoint range = C_point.norm();
    if (range < 1e-3f) {
      continue;
    }
    if (1e3f < range) {
      LOG(INFO) << "Skipping measurement with suspicious length: " << range;
      continue;
    }

    // Calculate the range image index
    const IndexElement range_image_index =
        projection_model_.bearingToNearestIndex(C_point);
    if (range_image_index < 0 ||
        posed_range_image.getNumBeams() <= range_image_index) {
      // Prevent out-of-bounds access
      continue;
    }

    // Add the point to the range image
    // If multiple points hit the same image pixel, keep the closest point
    const FloatingPoint old_range_value =
        posed_range_image.getRange(range_image_index);
    if (old_range_value < config_.min_range || range < old_range_value) {
      posed_range_image.getRange(range_image_index) = range;
    }
  }
}
}  // namespace wavemap
