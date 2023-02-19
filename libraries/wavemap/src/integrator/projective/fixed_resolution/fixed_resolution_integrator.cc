#include "wavemap/integrator/projective/fixed_resolution/fixed_resolution_integrator.h"

#include "wavemap/iterator/grid_iterator.h"

namespace wavemap {
void FixedResolutionIntegrator::importPointcloud(
    const PosedPointcloud<wavemap::Point3D>& pointcloud) {
  // Reset the posed range image, beam offset image and aabb
  posed_range_image_->resetToInitialValue();
  posed_range_image_->setPose(pointcloud.getPose());
  beam_offset_image_.setToConstant(Vector2D::Zero());
  aabb_ = AABB<Point3D>{};

  // Import all the points while updating the AABB
  aabb_.includePoint(pointcloud.getOrigin());  // sensor origin
  for (const auto& C_point : pointcloud.getPointsLocal()) {
    // Filter out noisy points and compute point's range
    if (!isMeasurementValid(C_point)) {
      continue;
    }

    // Add the point to the range image
    const Vector3D sensor_coordinates =
        projection_model_->cartesianToSensor(C_point);
    const auto [range_image_index, beam_to_pixel_offset] =
        projection_model_->imageToNearestIndexAndOffset(
            sensor_coordinates.head<2>());
    if (!posed_range_image_->isIndexWithinBounds(range_image_index)) {
      // Prevent out-of-bounds access
      continue;
    }

    // Add the point to the range image
    // If multiple points hit the same image pixel, keep the closest point
    const FloatingPoint range = sensor_coordinates[2];
    const FloatingPoint old_range_value =
        posed_range_image_->getRange(range_image_index);
    if (old_range_value < config_.min_range || range < old_range_value) {
      posed_range_image_->getRange(range_image_index) = range;
      beam_offset_image_.getBeamOffset(range_image_index) =
          beam_to_pixel_offset;
    }

    // Update the AABB (in world frame)
    Point3D C_point_truncated = getEndPointOrMaxRange(Point3D::Zero(), C_point,
                                                      range, config_.max_range);
    const Point3D W_point_truncated = pointcloud.getPose() * C_point_truncated;
    aabb_.includePoint(W_point_truncated);
  }

  // Pad the aabb to account for the beam uncertainties
  const FloatingPoint max_lateral_component =
      measurement_model_.getCombinedThreshold(
          config_.max_range +
          measurement_model_.getRangeThresholdBehindSurface());
  aabb_.min -= Vector3D::Constant(max_lateral_component);
  aabb_.max += Vector3D::Constant(max_lateral_component);
}

void FixedResolutionIntegrator::importRangeImage(
    const wavemap::PosedRangeImage2D& range_image_input) {
  // Load the range image
  ScanwiseIntegrator::importRangeImage(range_image_input);

  // Update the AABB to contain the camera frustum
  aabb_ = AABB<Point3D>{};
  aabb_.includePoint(posed_range_image_->getPose().getPosition());  // sensor
  for (int corner_idx = 0; corner_idx < 4; ++corner_idx) {
    const Index2D frustum_corner_image_index =
        posed_range_image_->getDimensions().cwiseProduct(
            Index2D{corner_idx & 1, (corner_idx >> 1) & 1});
    const Vector2D frustum_corner_coordinate =
        projection_model_->indexToImage(frustum_corner_image_index);
    const Point3D C_frustum_corner = projection_model_->sensorToCartesian(
        frustum_corner_coordinate, config_.max_range);
    const Point3D W_frustum_corner =
        posed_range_image_->getPose() * C_frustum_corner;
    aabb_.includePoint(W_frustum_corner);
  }

  // Pad the aabb to account for the beam uncertainties
  const FloatingPoint max_lateral_component =
      measurement_model_.getCombinedThreshold(
          config_.max_range +
          measurement_model_.getRangeThresholdBehindSurface());
  aabb_.min -= Vector3D::Constant(max_lateral_component);
  aabb_.max += Vector3D::Constant(max_lateral_component);
}

void FixedResolutionIntegrator::updateMap() {
  // Compute the min and max map indices that could be affected by the cloud
  const FloatingPoint min_cell_width = occupancy_map_->getMinCellWidth();
  const FloatingPoint min_cell_width_inv = 1.f / min_cell_width;
  const Index3D aabb_min_index =
      convert::pointToFloorIndex(aabb_.min, min_cell_width_inv);
  const Index3D aabb_max_index =
      convert::pointToCeilIndex(aabb_.max, min_cell_width_inv);

  // Iterate over all the cells in the AABB and update the map when needed
  const Transformation3D T_C_W = posed_range_image_->getPoseInverse();
  for (const Index3D& index : Grid(aabb_min_index, aabb_max_index)) {
    const Point3D W_cell_center =
        convert::indexToCenterPoint(index, min_cell_width);
    const Point3D C_cell_center = T_C_W * W_cell_center;
    const FloatingPoint update = computeUpdate(C_cell_center);
    if (kEpsilon < std::abs(update)) {
      occupancy_map_->addToCellValue(index, update);
    }
  }
}
}  // namespace wavemap
