#include "wavemap_3d/integrator/projective/fixed_resolution/fixed_resolution_integrator_3d.h"

#include <wavemap_common/iterator/grid_iterator.h>

namespace wavemap {
void FixedResolutionIntegrator3D::integratePointcloud(
    const PosedPointcloud<Point3D>& pointcloud) {
  if (!isPointcloudValid(pointcloud)) {
    return;
  }

  // Compute the range image and the scan's AABB
  const auto [range_image, aabb] =
      computeRangeImageAndAABB(pointcloud, spherical_projector_);

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
    const FloatingPoint d_C_cell = C_cell_center.norm();
    const Vector2D spherical_C_cell =
        SphericalProjector::bearingToSpherical(C_cell_center);
    const FloatingPoint update =
        computeUpdate(range_image, d_C_cell, spherical_C_cell);
    if (kEpsilon < std::abs(update)) {
      occupancy_map_->addToCellValue(index, update);
    }
  }
}

std::pair<RangeImage2D, AABB<Point3D>>
FixedResolutionIntegrator3D::computeRangeImageAndAABB(
    const PosedPointcloud<Point3D>& pointcloud,
    const SphericalProjector& spherical_projector) {
  RangeImage2D range_image(spherical_projector);
  AABB<Point3D> aabb;

  for (const auto& C_point : pointcloud.getPointsLocal()) {
    // Filter out noisy points and compute point's range
    if (C_point.hasNaN()) {
      LOG(WARNING) << "Skipping measurement whose endpoint contains NaNs:\n"
                   << C_point;
      continue;
    }
    const FloatingPoint range = C_point.norm();
    if (range < kEpsilon) {
      continue;
    }
    if (1e3f < range) {
      LOG(INFO) << "Skipping measurement with suspicious length: " << range;
      continue;
    }

    // Add the point to the range image
    const Index2D range_image_index =
        spherical_projector.bearingToNearestIndex(C_point);
    if ((range_image_index.array() < 0).any() ||
        (range_image.getDimensions().array() <= range_image_index.array())
            .any()) {
      // Prevent out-of-bounds access
      continue;
    }
    range_image[range_image_index] = range;

    // Update the AABB (in world frame)
    Point3D C_point_truncated = C_point;
    if (ContinuousVolumetricLogOdds<3>::kRangeMax < range) {
      C_point_truncated *= ContinuousVolumetricLogOdds<3>::kRangeMax / range;
    }
    const Point3D W_point_truncated = pointcloud.getPose() * C_point_truncated;
    aabb.includePoint(W_point_truncated);
  }

  // Pad the aabb to account for the beam uncertainties
  const FloatingPoint max_lateral_component =
      std::max(std::sin(ContinuousVolumetricLogOdds<3>::kAngleThresh) *
                   (ContinuousVolumetricLogOdds<3>::kRangeMax +
                    ContinuousVolumetricLogOdds<3>::kRangeDeltaThresh),
               ContinuousVolumetricLogOdds<3>::kRangeDeltaThresh);
  aabb.min -= Vector3D::Constant(max_lateral_component);
  aabb.max += Vector3D::Constant(max_lateral_component);

  return {range_image, aabb};
}
}  // namespace wavemap
