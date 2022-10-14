#include "wavemap_3d/integrator/projective/scanwise_integrator_3d.h"

namespace wavemap {
void ScanwiseIntegrator3D::updateRangeImage(
    const PosedPointcloud<Point3D>& pointcloud) {
  posed_range_image_->resetToInitialValue();
  posed_range_image_->setPose(pointcloud.getPose());
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
    const Index2D range_image_index =
        projection_model_.bearingToNearestIndex(C_point);
    if ((range_image_index.array() < 0).any() ||
        (posed_range_image_->getDimensions().array() <=
         range_image_index.array())
            .any()) {
      // Prevent out-of-bounds access
      continue;
    }

    // Add the point to the range image, if multiple points hit the same image
    // pixel, keep the closest point
    const FloatingPoint old_range_value =
        posed_range_image_->getRange(range_image_index);
    if (old_range_value < config_.min_range || range < old_range_value) {
      posed_range_image_->getRange(range_image_index) = range;
      bearing_image_.getBearing(range_image_index) = C_point / range;
    }
  }
}
}  // namespace wavemap
