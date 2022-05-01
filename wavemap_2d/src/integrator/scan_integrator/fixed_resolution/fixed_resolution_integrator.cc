#include "wavemap_2d/integrator/scan_integrator/fixed_resolution/fixed_resolution_integrator.h"

#include "wavemap_2d/iterator/grid_iterator.h"

namespace wavemap_2d {
void FixedResolutionIntegrator::integratePointcloud(
    const PosedPointcloud<>& pointcloud) {
  if (!isPointcloudValid(pointcloud)) {
    return;
  }

  // Compute the range image and the scan's AABB
  // TODO(victorr): Make this configurable
  // TODO(victorr): Avoid reallocating the range image (zero and reuse instead)
  RangeImage range_image(-M_PI_2f32, M_PI_2f32, pointcloud.size());
  AABB<Point> aabb;
  computeRangeImageAndAABB(pointcloud, range_image, aabb);

  // Compute the min and max map indices that could be affected by the cloud
  const FloatingPoint resolution = occupancy_map_->getResolution();
  const FloatingPoint resolution_inv = 1.f / resolution;
  const Index aabb_min_index =
      computeFloorIndexForPoint(aabb.min, resolution_inv);
  const Index aabb_max_index =
      computeCeilIndexForPoint(aabb.max, resolution_inv);

  // Iterate over all the cells in the AABB and update the map when needed
  const Transformation T_C_W = pointcloud.getPose().inverse();
  for (const Index& index : Grid(aabb_min_index, aabb_max_index)) {
    const Point W_cell_center = computeCenterFromIndex(index, resolution);
    const Point C_cell_center = T_C_W * W_cell_center;
    const FloatingPoint update =
        computeUpdateForCell(range_image, C_cell_center);
    if (kEpsilon < std::abs(update)) {
      occupancy_map_->addToCellValue(index, update);
    }
  }
}

void FixedResolutionIntegrator::computeRangeImageAndAABB(
    const PosedPointcloud<>& pointcloud, RangeImage& range_image,
    AABB<Point>& aabb) {
  for (const auto& C_point : pointcloud.getPointsLocal()) {
    // Filter out noisy points and compute point's range
    if (C_point.hasNaN()) {
      LOG(WARNING) << "Skipping measurement whose endpoint contains NaNs:\n"
                   << C_point;
      continue;
    }
    const FloatingPoint range = C_point.norm();
    if (1e3 < range) {
      LOG(INFO) << "Skipping measurement with suspicious length: " << range;
      continue;
    }

    // Add the point to the range image
    const RangeImageIndex range_image_index =
        range_image.bearingToNearestIndex(C_point);
    CHECK_GE(range_image_index, 0);
    CHECK_LT(range_image_index, range_image.getNBeams());
    range_image[range_image_index] = range;

    // Update the AABB (in world frame)
    Point C_point_truncated = C_point;
    if (BeamModel::kRangeMax < range) {
      C_point_truncated *= BeamModel::kRangeMax / range;
    }
    const Point W_point_truncated = pointcloud.getPose() * C_point_truncated;
    aabb.includePoint(W_point_truncated);
  }

  // Pad the aabb to account for the beam uncertainties
  const FloatingPoint max_lateral_component =
      std::max(std::sin(BeamModel::kAngleThresh) *
                   (BeamModel::kRangeMax + BeamModel::kRangeDeltaThresh),
               BeamModel::kRangeDeltaThresh);
  aabb.min -= Vector::Constant(max_lateral_component);
  aabb.max += Vector::Constant(max_lateral_component);
}
}  // namespace wavemap_2d
