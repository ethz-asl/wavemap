#include "wavemap_2d/integrator/scan_integrator/fixed_resolution/fixed_resolution_integrator.h"

#include "wavemap_2d/iterator/grid_iterator.h"

namespace wavemap_2d {
void FixedResolutionIntegrator::integratePointcloud(
    const PosedPointcloud<>& pointcloud) {
  if (!isPointcloudValid(pointcloud)) {
    return;
  }

  // TODO(victorr): Check that the pointcloud's angular resolution is lower than
  //                the angular uncertainty of the beam model. This is necessary
  //                since this measurement integrator assumes the beams don't
  //                overlap, i.e. for each sample point we only evaluate the
  //                contribution from the nearest beam.

  // Compute the range image and the scan's AABB
  // TODO(victorr): Make this configurable
  // TODO(victorr): Avoid reallocating the range image (zero and reuse instead)
  const auto [range_image, aabb] = computeRangeImageAndAABB(
      pointcloud, -kHalfPi, kHalfPi, pointcloud.size());

  // Compute the min and max map indices that could be affected by the cloud
  const FloatingPoint min_cell_width = occupancy_map_->getMinCellWidth();
  const FloatingPoint min_cell_width_inv = 1.f / min_cell_width;
  const Index aabb_min_index =
      convert::pointToFloorIndex(aabb.min, min_cell_width_inv);
  const Index aabb_max_index =
      convert::pointToCeilIndex(aabb.max, min_cell_width_inv);

  // Iterate over all the cells in the AABB and update the map when needed
  const Transformation T_C_W = pointcloud.getPose().inverse();
  for (const Index& index : Grid(aabb_min_index, aabb_max_index)) {
    const Point W_cell_center =
        convert::indexToCenterPoint(index, min_cell_width);
    const Point C_cell_center = T_C_W * W_cell_center;
    const FloatingPoint update =
        computeUpdateForCell(range_image, C_cell_center);
    if (kEpsilon < std::abs(update)) {
      occupancy_map_->addToCellValue(index, update);
    }
  }
}

std::pair<RangeImage, AABB<Point>>
FixedResolutionIntegrator::computeRangeImageAndAABB(
    const PosedPointcloud<>& pointcloud, FloatingPoint min_angle,
    FloatingPoint max_angle, Eigen::Index num_beams) {
  RangeImage range_image(min_angle, max_angle, num_beams);
  AABB<Point> aabb;

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
    CHECK_LT(range_image_index, range_image.getNumBeams());
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

  return {range_image, aabb};
}
}  // namespace wavemap_2d
