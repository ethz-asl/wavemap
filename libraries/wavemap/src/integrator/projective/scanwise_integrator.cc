#include "wavemap/integrator/projective/scanwise_integrator.h"

namespace wavemap {
void ScanwiseIntegrator::integratePointcloud(
    const PosedPointcloud<Point<3>>& pointcloud) {
  if (!isPointcloudValid(pointcloud)) {
    return;
  }
  importPointcloud(pointcloud);
  updateMap();
}

void ScanwiseIntegrator::integrateRangeImage(const PosedImage<>& range_image) {
  importRangeImage(range_image);
  updateMap();
}

void ScanwiseIntegrator::importPointcloud(
    const PosedPointcloud<Point3D>& pointcloud) {
  // Reset the posed range image and the beam offset image
  posed_range_image_->resetToInitialValue();
  posed_range_image_->setPose(pointcloud.getPose());
  beam_offset_image_.setToConstant(Vector2D::Zero());

  // Import all the points
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
        posed_range_image_->at(range_image_index);
    if (old_range_value < config_.min_range || range < old_range_value) {
      posed_range_image_->at(range_image_index) = range;
      beam_offset_image_.at(range_image_index) = beam_to_pixel_offset;
    }
  }
}

void ScanwiseIntegrator::importRangeImage(
    const PosedImage<>& range_image_input) {
  posed_range_image_ = std::make_shared<PosedImage<>>(range_image_input);
  beam_offset_image_.setToConstant(Vector2D::Zero());
}
}  // namespace wavemap
