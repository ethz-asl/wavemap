#ifndef WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_INL_RANGE_IMAGE_2D_INTERSECTOR_INL_H_
#define WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_INL_RANGE_IMAGE_2D_INTERSECTOR_INL_H_

#include <wavemap_common/integrator/measurement_model/range_and_angle/continuous_volumetric_log_odds.h>

namespace wavemap {
inline RangeImage2DIntersector::MinMaxAnglePair
RangeImage2DIntersector::getAabbMinMaxProjectedAngle(
    const Transformation3D& T_W_C, const AABB<Point3D>& W_aabb) {
  // If the sensor is contained in the AABB, it overlaps with the full range
  if (W_aabb.containsPoint(T_W_C.getPosition())) {
    return {Vector2D::Constant(-kPi), Vector2D::Constant(kPi)};
  }

  // TODO(victorr): Optimize this
  const AABB<Point3D>::Corners C_t_C_corners =
      T_W_C.inverse().transformVectorized(W_aabb.corners());
  MinMaxAnglePair angle_intervals_standard;
  MinMaxAnglePair angle_intervals_if_wrapping;
  for (int corner_idx = 0; corner_idx < AABB<Point3D>::kNumCorners;
       ++corner_idx) {
    const Vector2D angles =
        SphericalProjector::bearingToSpherical(C_t_C_corners.col(corner_idx));
    angle_intervals_standard.min_spherical_coordinates =
        angle_intervals_standard.min_spherical_coordinates.cwiseMin(angles);
    angle_intervals_standard.max_spherical_coordinates =
        angle_intervals_standard.max_spherical_coordinates.cwiseMax(angles);

    for (const int axis : {0, 1}) {
      if (0.f < angles[axis]) {
        angle_intervals_if_wrapping.min_spherical_coordinates[axis] = std::min(
            angle_intervals_if_wrapping.min_spherical_coordinates[axis],
            angles[axis]);
      } else {
        angle_intervals_if_wrapping.max_spherical_coordinates[axis] = std::max(
            angle_intervals_if_wrapping.max_spherical_coordinates[axis],
            angles[axis]);
      }
    }
  }

  const Eigen::Matrix<bool, 2, 1> angle_interval_wraps_around =
      kPi < (angle_intervals_standard.max_spherical_coordinates -
             angle_intervals_standard.min_spherical_coordinates)
                .array();

  const Vector2D min_spherical_coordinates = angle_interval_wraps_around.select(
      angle_intervals_if_wrapping.min_spherical_coordinates,
      angle_intervals_standard.min_spherical_coordinates);
  const Vector2D max_spherical_coordinates = angle_interval_wraps_around.select(
      angle_intervals_if_wrapping.max_spherical_coordinates,
      angle_intervals_standard.max_spherical_coordinates);
  return {min_spherical_coordinates, max_spherical_coordinates};
}

inline IntersectionType RangeImage2DIntersector::determineIntersectionType(
    const Transformation3D& T_W_C, const AABB<Point3D>& W_cell_aabb,
    const SphericalProjector& spherical_projector) const {
  // Get the min and max distances from any point in the cell (which is an
  // axis-aligned cube) to the sensor's center
  // NOTE: The min distance is 0 if the cell contains the sensor's center.
  const FloatingPoint d_C_cell_closest =
      W_cell_aabb.minDistanceTo(T_W_C.getPosition());
  if (ContinuousVolumetricLogOdds<2>::kRangeMax < d_C_cell_closest) {
    return IntersectionType::kFullyUnknown;
  }
  const FloatingPoint d_C_cell_furthest =
      W_cell_aabb.maxDistanceTo(T_W_C.getPosition());

  // Get the min and max angles for any point in the cell projected into the
  // range image
  auto [min_spherical_coordinates, max_spherical_coordinates] =
      getAabbMinMaxProjectedAngle(T_W_C, W_cell_aabb);

  // Pad the min and max angles with the BeamModel's angle threshold to
  // account for the beam's non-zero width (angular uncertainty)
  min_spherical_coordinates -=
      Vector2D::Constant(ContinuousVolumetricLogOdds<2>::kAngleThresh);
  max_spherical_coordinates +=
      Vector2D::Constant(ContinuousVolumetricLogOdds<2>::kAngleThresh);

  // If the angle wraps around Pi, we can't use the hierarchical range image
  const bool any_angle_range_wraps_pi =
      (max_spherical_coordinates.array() < min_spherical_coordinates.array())
          .any();
  if (any_angle_range_wraps_pi) {
    if ((max_spherical_coordinates.array() <
         spherical_projector.getMinAngles().array())
            .all() &&
        (spherical_projector.getMaxAngles().array() <
         min_spherical_coordinates.array())
            .all()) {
      // No parts of the cell can be affected by the measurement update
      return IntersectionType::kFullyUnknown;
    } else {
      // Make sure the cell gets enqueued for refinement, as we can't
      // guarantee anything about its children
      return IntersectionType::kPossiblyOccupied;
    }
  }

  // Check if the cell is outside the observed range
  if ((spherical_projector.getMaxAngles().array() <
       min_spherical_coordinates.array())
          .all() ||
      (max_spherical_coordinates.array() <
       spherical_projector.getMinAngles().array())
          .all()) {
    return IntersectionType::kFullyUnknown;
  }

  // Convert the angles to range image indices
  const Index2D min_image_idx =
      spherical_projector.sphericalToFloorIndex(min_spherical_coordinates)
          .cwiseMax(Index2D::Zero());
  const Index2D max_image_idx =
      spherical_projector.sphericalToCeilIndex(max_spherical_coordinates)
          .cwiseMin(spherical_projector.getDimensions() - Index2D::Ones());

  // Check if the cell overlaps with the approximate but conservative distance
  // bounds of the hierarchical range image
  const Bounds distance_bounds =
      hierarchical_range_image_.getRangeBounds(min_image_idx, max_image_idx);
  if (distance_bounds.upper +
          ContinuousVolumetricLogOdds<2>::kRangeDeltaThresh <
      d_C_cell_closest) {
    return IntersectionType::kFullyUnknown;
  } else if (d_C_cell_furthest <
             distance_bounds.lower -
                 ContinuousVolumetricLogOdds<2>::kRangeDeltaThresh) {
    return IntersectionType::kFreeOrUnknown;
  } else {
    return IntersectionType::kPossiblyOccupied;
  }
}
}  // namespace wavemap

#endif  // WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_INL_RANGE_IMAGE_2D_INTERSECTOR_INL_H_
