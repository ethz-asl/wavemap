#include "wavemap_2d/integrator/projective/fixed_resolution/fixed_resolution_integrator_2d.h"

#include <wavemap_common/iterator/grid_iterator.h>

namespace wavemap {
void FixedResolutionIntegrator2D::integratePointcloud(
    const PosedPointcloud<Point2D>& pointcloud) {
  if (!isPointcloudValid(pointcloud)) {
    return;
  }

  // Compute the range image and the scan's AABB
  const auto aabb = computeRangeImageAndAABB(pointcloud);

  // Compute the min and max map indices that could be affected by the cloud
  const FloatingPoint min_cell_width = occupancy_map_->getMinCellWidth();
  const FloatingPoint min_cell_width_inv = 1.f / min_cell_width;
  const Index2D aabb_min_index =
      convert::pointToFloorIndex(aabb.min, min_cell_width_inv);
  const Index2D aabb_max_index =
      convert::pointToCeilIndex(aabb.max, min_cell_width_inv);

  // Iterate over all the cells in the AABB and update the map when needed
  const Transformation2D T_C_W = pointcloud.getPose().inverse();
  for (const Index2D& index : Grid(aabb_min_index, aabb_max_index)) {
    const Point2D W_cell_center =
        convert::indexToCenterPoint(index, min_cell_width);
    const Point2D C_cell_center = T_C_W * W_cell_center;
    const FloatingPoint update = computeUpdate(C_cell_center);
    if (kEpsilon < std::abs(update)) {
      occupancy_map_->addToCellValue(index, update);
    }
  }
}

AABB<Point2D> FixedResolutionIntegrator2D::computeRangeImageAndAABB(
    const PosedPointcloud<Point2D>& pointcloud) {
  posed_range_image_->resetToInitialValue();
  posed_range_image_->setPose(pointcloud.getPose());

  AABB<Point2D> aabb;

  for (const auto& C_point : pointcloud.getPointsLocal()) {
    // Filter out noisy points and compute point's range
    const FloatingPoint range = C_point.norm();
    if (!isMeasurementValid(C_point)) {
      continue;
    }

    // Add the point to the range image
    const IndexElement range_image_index =
        projection_model_.bearingToNearestIndex(C_point);
    if (!posed_range_image_->isIndexWithinBounds(range_image_index)) {
      // Prevent out-of-bounds access
      continue;
    }
    posed_range_image_->getRange(range_image_index) = range;

    // Update the AABB (in world frame)
    const Point2D C_point_truncated = getEndPointOrMaxRange(
        Point2D::Zero(), C_point, range, config_.max_range);
    const Point2D W_point_truncated = pointcloud.getPose() * C_point_truncated;
    aabb.includePoint(W_point_truncated);
  }

  // Pad the aabb to account for the beam uncertainties
  const FloatingPoint max_lateral_component =
      measurement_model_.getCombinedThreshold(
          config_.max_range +
          measurement_model_.getRangeThresholdBehindSurface());
  aabb.min -= Vector2D::Constant(max_lateral_component);
  aabb.max += Vector2D::Constant(max_lateral_component);

  return aabb;
}
}  // namespace wavemap
