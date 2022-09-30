#ifndef WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_RANGE_IMAGE_2D_INTERSECTOR_INL_H_
#define WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_RANGE_IMAGE_2D_INTERSECTOR_INL_H_

#include <algorithm>
#include <bitset>
#include <limits>

#include <wavemap_common/integrator/measurement_model/range_and_angle/continuous_volumetric_log_odds.h>
#include <wavemap_common/utils/angle_utils.h>
#include <wavemap_common/utils/approximate_trigonometry.h>

namespace wavemap {
inline RangeImage2DIntersector::MinMaxAnglePair
RangeImage2DIntersector::getAabbMinMaxProjectedAngle(
    const Transformation3D& T_W_C, const AABB<Point3D>& W_aabb) {
  Cache cache{};
  return getAabbMinMaxProjectedAngle(T_W_C, W_aabb, cache);
}

inline RangeImage2DIntersector::MinMaxAnglePair
RangeImage2DIntersector::getAabbMinMaxProjectedAngle(
    const Transformation3D& T_W_C, const AABB<Point3D>& W_aabb, Cache& cache) {
  MinMaxAnglePair angle_intervals;

  // If the sensor is contained in the AABB, it overlaps with the full range
  if (W_aabb.containsPoint(T_W_C.getPosition())) {
    angle_intervals = {Vector2D::Constant(-kPi), Vector2D::Constant(kPi)};
    return angle_intervals;
  }

  const Transformation3D T_C_W = T_W_C.inverse();

  if (cache.has_value()) {
    const Point3D min_elevation_corner_point =
        T_C_W * W_aabb.corner_point(cache.value().min_corner_indices[0]);
    angle_intervals.min_spherical_coordinates[0] =
        approximate::atan2()(min_elevation_corner_point.z(),
                             min_elevation_corner_point.head<2>().norm());

    const Point3D min_azimuth_corner_point =
        T_C_W * W_aabb.corner_point(cache.value().min_corner_indices[1]);
    angle_intervals.min_spherical_coordinates[1] = approximate::atan2()(
        min_azimuth_corner_point.y(), min_azimuth_corner_point.x());

    const Point3D max_elevation_corner_point =
        T_C_W * W_aabb.corner_point(cache.value().max_corner_indices[0]);
    angle_intervals.max_spherical_coordinates[0] =
        approximate::atan2()(max_elevation_corner_point.z(),
                             max_elevation_corner_point.head<2>().norm());

    const Point3D max_azimuth_corner_point =
        T_C_W * W_aabb.corner_point(cache.value().max_corner_indices[1]);
    angle_intervals.max_spherical_coordinates[1] = approximate::atan2()(
        max_azimuth_corner_point.y(), max_azimuth_corner_point.x());

    for (int axis : {0, 1}) {
      auto& min_angle = angle_intervals.min_spherical_coordinates[axis];
      auto& max_angle = angle_intervals.max_spherical_coordinates[axis];

      if (kPi < max_angle - min_angle) {
        min_angle += approximate::atan2::kWorstCaseError;
        max_angle -= approximate::atan2::kWorstCaseError;
      } else {
        min_angle -= approximate::atan2::kWorstCaseError;
        max_angle += approximate::atan2::kWorstCaseError;
      }

      min_angle = angle_math::normalize_near(min_angle);
      max_angle = angle_math::normalize_near(max_angle);
    }

    return angle_intervals;
  }

  const AABB<Point3D>::Corners C_t_C_corners =
      T_C_W.transformVectorized(W_aabb.corner_matrix());
  Eigen::Matrix<FloatingPoint, 2, 8> spherical_C_corners;
  std::bitset<3> all_positive{0b111};
  std::bitset<3> all_negative{0b111};
  for (int corner_idx = 0; corner_idx < AABB<Point3D>::kNumCorners;
       ++corner_idx) {
    const Point3D& C_t_C_corner = C_t_C_corners.col(corner_idx);
    spherical_C_corners.col(corner_idx) =
        SphericalProjector::bearingToSpherical(C_t_C_corner);
    for (int dim_idx = 0; dim_idx < 3; ++dim_idx) {
      if (bool is_negative = std::signbit(C_t_C_corner[dim_idx]); is_negative) {
        all_positive.set(dim_idx, false);
      } else {
        all_negative.set(dim_idx, false);
      }
    }
  }
  const bool all_corners_in_same_octant = (all_positive | all_negative).all();

  if (all_corners_in_same_octant) {
    cache.emplace();
    for (const int axis : {0, 1}) {
      auto& min_angle = angle_intervals.min_spherical_coordinates[axis];
      auto& max_angle = angle_intervals.max_spherical_coordinates[axis];

      min_angle = spherical_C_corners.row(axis).minCoeff(
          &cache.value().min_corner_indices[axis]);
      max_angle = spherical_C_corners.row(axis).maxCoeff(
          &cache.value().max_corner_indices[axis]);

      const bool angle_interval_wraps_around = kPi < (max_angle - min_angle);
      if (angle_interval_wraps_around) {
        min_angle = MinMaxAnglePair::kInitialMin;
        max_angle = MinMaxAnglePair::kInitialMax;
        for (int corner_idx = 0; corner_idx < AABB<Point3D>::kNumCorners;
             ++corner_idx) {
          const FloatingPoint angle = spherical_C_corners(axis, corner_idx);
          if (0.f < angle) {
            if (min_angle < angle) {
              min_angle = angle;
              cache.value().min_corner_indices[axis] = corner_idx;
            }
          } else {
            if (angle < max_angle) {
              max_angle = angle;
              cache.value().max_corner_indices[axis] = corner_idx;
            }
          }
        }
      }
    }

    return angle_intervals;
  }

  for (const int axis : {0, 1}) {
    auto& min_angle = angle_intervals.min_spherical_coordinates[axis];
    auto& max_angle = angle_intervals.max_spherical_coordinates[axis];

    min_angle = spherical_C_corners.row(axis).minCoeff();
    max_angle = spherical_C_corners.row(axis).maxCoeff();

    const bool angle_interval_wraps_around = kPi < (max_angle - min_angle);
    if (angle_interval_wraps_around) {
      min_angle = MinMaxAnglePair::kInitialMin;
      max_angle = MinMaxAnglePair::kInitialMax;
      for (int corner_idx = 0; corner_idx < AABB<Point3D>::kNumCorners;
           ++corner_idx) {
        const FloatingPoint angle = spherical_C_corners(axis, corner_idx);
        if (0.f < angle) {
          min_angle = std::min(min_angle, angle);
        } else {
          max_angle = std::max(max_angle, angle);
        }
      }
    }
  }

  return angle_intervals;
}

inline IntersectionType RangeImage2DIntersector::determineIntersectionType(
    const Transformation3D& T_W_C, const AABB<Point3D>& W_cell_aabb,
    const SphericalProjector& spherical_projector) const {
  Cache cache{};
  return determineIntersectionType(T_W_C, W_cell_aabb, spherical_projector,
                                   cache);
}

inline IntersectionType RangeImage2DIntersector::determineIntersectionType(
    const Transformation3D& T_W_C, const AABB<Point3D>& W_cell_aabb,
    const SphericalProjector& spherical_projector, Cache& cache) const {
  // Get the min and max distances from any point in the cell (which is an
  // axis-aligned cube) to the sensor's center
  // NOTE: The min distance is 0 if the cell contains the sensor's center.
  const FloatingPoint d_C_cell_closest =
      W_cell_aabb.minDistanceTo(T_W_C.getPosition());
  if (ContinuousVolumetricLogOdds<3>::kRangeMax < d_C_cell_closest) {
    return IntersectionType::kFullyUnknown;
  }

  // Get the min and max angles for any point in the cell projected into the
  // range image
  auto [min_spherical_coordinates, max_spherical_coordinates] =
      getAabbMinMaxProjectedAngle(T_W_C, W_cell_aabb, cache);

  // Pad the min and max angles with the BeamModel's angle threshold to
  // account for the beam's non-zero width (angular uncertainty)
  min_spherical_coordinates -=
      Vector2D::Constant(ContinuousVolumetricLogOdds<3>::kAngleThresh);
  max_spherical_coordinates +=
      Vector2D::Constant(ContinuousVolumetricLogOdds<3>::kAngleThresh);

  // If the angle wraps around Pi, we can't use the hierarchical range image
  const bool elevation_range_wraps_pi =
      max_spherical_coordinates.x() < min_spherical_coordinates.x();
  const bool azimuth_range_wraps_pi =
      max_spherical_coordinates.y() < min_spherical_coordinates.y();
  if (elevation_range_wraps_pi ||
      (!kAzimuthAllowedToWrapAround && azimuth_range_wraps_pi)) {
    const bool elevation_range_fully_outside_fov =
        max_spherical_coordinates.x() <
            spherical_projector.getMinAngles().x() &&
        spherical_projector.getMaxAngles().x() < min_spherical_coordinates.x();
    const bool azimuth_range_fully_outside_fov =
        max_spherical_coordinates.y() <
            spherical_projector.getMinAngles().y() &&
        spherical_projector.getMaxAngles().y() < min_spherical_coordinates.y();
    if ((!elevation_range_wraps_pi || elevation_range_fully_outside_fov) &&
        (kAzimuthAllowedToWrapAround || !azimuth_range_wraps_pi ||
         azimuth_range_fully_outside_fov)) {
      // No parts of the cell can be affected by the measurement update
      return IntersectionType::kFullyUnknown;
    } else {
      // Make sure the cell gets enqueued for refinement, as we can't
      // guarantee anything about its children
      return IntersectionType::kPossiblyOccupied;
    }
  }

  // Check if the cell is outside the FoV
  if ((max_spherical_coordinates.array() <
       spherical_projector.getMinAngles().array())
          .any() ||
      (spherical_projector.getMaxAngles().array() <
       min_spherical_coordinates.array())
          .any()) {
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
  const FloatingPoint d_C_cell_furthest =
      W_cell_aabb.maxDistanceTo(T_W_C.getPosition());
  constexpr FloatingPoint kRangeMax = MeasurementModelBase<3>::kRangeMax;
  constexpr FloatingPoint kRangeUncertaintyThreshold =
      ContinuousVolumetricLogOdds<3>::kRangeDeltaThresh;
  if (distance_bounds.upper + kRangeUncertaintyThreshold < d_C_cell_closest) {
    return IntersectionType::kFullyUnknown;
  } else if (d_C_cell_furthest <
                 distance_bounds.lower - kRangeUncertaintyThreshold ||
             kRangeMax + kRangeUncertaintyThreshold < distance_bounds.lower) {
    return IntersectionType::kFreeOrUnknown;
  } else {
    return IntersectionType::kPossiblyOccupied;
  }
}
}  // namespace wavemap

#endif  // WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_RANGE_IMAGE_2D_INTERSECTOR_INL_H_
