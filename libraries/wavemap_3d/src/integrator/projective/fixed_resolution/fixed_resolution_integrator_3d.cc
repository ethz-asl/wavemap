#include "wavemap_3d/integrator/projective/fixed_resolution/fixed_resolution_integrator_3d.h"

#include <wavemap_common/iterator/grid_iterator.h>

namespace wavemap {
void FixedResolutionIntegrator3D::integratePointcloud(
    const PosedPointcloud<Point3D>& pointcloud) {
  if (!isPointcloudValid(pointcloud)) {
    return;
  }

  // Compute the range image and the scan's AABB
  const auto aabb = computeRangeImageAndAABB(pointcloud);

  // Compute the min and max map indices that could be affected by the cloud
  const FloatingPoint min_cell_width = occupancy_map_->getMinCellWidth();
  const FloatingPoint min_cell_width_inv = 1.f / min_cell_width;
  const Index3D aabb_min_index =
      convert::pointToFloorIndex(aabb.min, min_cell_width_inv);
  const Index3D aabb_max_index =
      convert::pointToCeilIndex(aabb.max, min_cell_width_inv);

  // Iterate over all the cells in the AABB and update the map when needed
  const Transformation3D T_C_W = pointcloud.getPose().inverse();
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

AABB<Point3D> FixedResolutionIntegrator3D::computeRangeImageAndAABB(
    const PosedPointcloud<Point3D>& pointcloud) {
  AABB<Point3D> aabb;
  posed_range_image_->resetToInitialValue();
  posed_range_image_->setPose(pointcloud.getPose());
  for (const auto& C_point : pointcloud.getPointsLocal()) {
    // Filter out noisy points and compute point's range
    if (!isMeasurementValid(C_point)) {
      continue;
    }

    // Add the point to the range image
    const Vector3D sensor_coordinates =
        projection_model_.cartesianToSensor(C_point);
    const Index2D range_image_index =
        projection_model_.imageToNearestIndex(sensor_coordinates.head<2>());
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
      // TODO(victorr): Generalize this for all sensor models
      bearing_image_.getBearing(range_image_index) = C_point.normalized();
    }

    // Update the AABB (in world frame)
    Point3D C_point_truncated = getEndPointOrMaxRange(Point3D::Zero(), C_point,
                                                      range, config_.max_range);
    const Point3D W_point_truncated = pointcloud.getPose() * C_point_truncated;
    aabb.includePoint(W_point_truncated);
  }

  // Pad the aabb to account for the beam uncertainties
  const FloatingPoint max_lateral_component =
      measurement_model_.getCombinedThreshold(
          config_.max_range +
          measurement_model_.getRangeThresholdBehindSurface());
  aabb.min -= Vector3D::Constant(max_lateral_component);
  aabb.max += Vector3D::Constant(max_lateral_component);

  return aabb;
}
}  // namespace wavemap
