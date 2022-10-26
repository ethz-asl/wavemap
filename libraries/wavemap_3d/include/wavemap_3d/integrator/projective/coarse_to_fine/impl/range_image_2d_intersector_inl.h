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
    const Transformation3D& T_W_C, const AABB<Point3D>& W_aabb) const {
  Cache cache{};
  return getAabbMinMaxProjectedAngle(T_W_C, W_aabb, *projection_model_, cache);
}

inline RangeImage2DIntersector::MinMaxAnglePair
RangeImage2DIntersector::getAabbMinMaxProjectedAngle(
    const Transformation3D& T_W_C, const AABB<Point3D>& W_aabb,
    RangeImage2DIntersector::Cache& cache) const {
  return getAabbMinMaxProjectedAngle(T_W_C, W_aabb, *projection_model_, cache);
}

inline RangeImage2DIntersector::MinMaxAnglePair
RangeImage2DIntersector::getAabbMinMaxProjectedAngle(
    const Transformation3D& T_W_C, const AABB<Point3D>& W_aabb,
    const Image2DProjectionModel& projection_model) {
  Cache cache{};
  return getAabbMinMaxProjectedAngle(T_W_C, W_aabb, projection_model, cache);
}

inline RangeImage2DIntersector::MinMaxAnglePair
RangeImage2DIntersector::getAabbMinMaxProjectedAngle(
    const Transformation3D& T_W_C, const AABB<Point3D>& W_aabb,
    const Image2DProjectionModel& projection_model,
    RangeImage2DIntersector::Cache& cache) {
  MinMaxAnglePair angle_intervals;

  // If the sensor is contained in the AABB, it overlaps with the full range
  if (W_aabb.containsPoint(T_W_C.getPosition())) {
    return {projection_model.getMinImageCoordinates(),
            projection_model.getMaxImageCoordinates()};
  }

  const Transformation3D T_C_W = T_W_C.inverse();

  if (cache.has_value()) {
    const Point3D min_elevation_corner_point =
        T_C_W * W_aabb.corner_point(cache.value().min_corner_indices[0]);
    angle_intervals.min_spherical_coordinates[0] =
        projection_model.cartesianToImageX(min_elevation_corner_point);

    const Point3D min_azimuth_corner_point =
        T_C_W * W_aabb.corner_point(cache.value().min_corner_indices[1]);
    angle_intervals.min_spherical_coordinates[1] =
        projection_model.cartesianToImageY(min_azimuth_corner_point);

    const Point3D max_elevation_corner_point =
        T_C_W * W_aabb.corner_point(cache.value().max_corner_indices[0]);
    angle_intervals.max_spherical_coordinates[0] =
        projection_model.cartesianToImageX(max_elevation_corner_point);

    const Point3D max_azimuth_corner_point =
        T_C_W * W_aabb.corner_point(cache.value().max_corner_indices[1]);
    angle_intervals.max_spherical_coordinates[1] =
        projection_model.cartesianToImageY(max_azimuth_corner_point);

    return angle_intervals;
  }

  Eigen::Matrix<FloatingPoint, 2, 8> spherical_C_corners;
  std::bitset<3> all_positive{0b111};
  std::bitset<3> all_negative{0b111};
  for (int corner_idx = 0; corner_idx < AABB<Point3D>::kNumCorners;
       ++corner_idx) {
    const Point3D C_t_C_corner = T_C_W * W_aabb.corner_point(corner_idx);
    spherical_C_corners.col(corner_idx) =
        projection_model.cartesianToImage(C_t_C_corner);
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
    const Transformation3D& T_W_C, const AABB<Point3D>& W_cell_aabb) const {
  Cache cache{};
  return determineIntersectionType(T_W_C, W_cell_aabb, cache);
}

inline IntersectionType RangeImage2DIntersector::determineIntersectionType(
    const Transformation3D& T_W_C, const AABB<Point3D>& W_cell_aabb,
    Cache& cache) const {
  // Get the min and max distances from any point in the cell (which is an
  // axis-aligned cube) to the sensor's center
  // NOTE: The min distance is 0 if the cell contains the sensor's center.
  const FloatingPoint d_C_cell_closest =
      W_cell_aabb.minDistanceTo(T_W_C.getPosition());
  if (max_range_ < d_C_cell_closest) {
    return IntersectionType::kFullyUnknown;
  }

  // Get the min and max angles for any point in the cell projected into the
  // range image
  auto [min_spherical_coordinates, max_spherical_coordinates] =
      getAabbMinMaxProjectedAngle(T_W_C, W_cell_aabb, cache);

  // Pad the min and max angles with the BeamModel's angle threshold to
  // account for the beam's non-zero width (angular uncertainty)
  const Index2D min_image_index = projection_model_->imageToFloorIndex(
      min_spherical_coordinates - Vector2D::Constant(angle_threshold_));
  const Index2D max_image_index = projection_model_->imageToCeilIndex(
      max_spherical_coordinates + Vector2D::Constant(angle_threshold_));

  // If the angle wraps around Pi, we can't use the hierarchical range image
  const bool elevation_range_wraps_pi =
      max_image_index.x() < min_image_index.x();
  const bool azimuth_range_wraps_pi = max_image_index.y() < min_image_index.y();
  if (elevation_range_wraps_pi ||
      (!azimuth_wraps_pi_ && azimuth_range_wraps_pi)) {
    const bool elevation_range_fully_outside_fov =
        max_image_index.x() < 0 &&
        projection_model_->getNumRows() < min_image_index.x();
    const bool azimuth_range_fully_outside_fov =
        max_image_index.y() < 0 &&
        projection_model_->getNumColumns() < min_image_index.y();
    if ((!elevation_range_wraps_pi || elevation_range_fully_outside_fov) &&
        (azimuth_wraps_pi_ || !azimuth_range_wraps_pi ||
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
  if ((max_image_index.array() < 0).any() ||
      (projection_model_->getDimensions().array() <= min_image_index.array())
          .any()) {
    return IntersectionType::kFullyUnknown;
  }

  // Convert the angles to range image indices
  const Index2D min_image_idx_rectified =
      min_image_index.cwiseMax(Index2D::Zero());
  const Index2D max_image_idx_rectified = max_image_index.cwiseMin(
      projection_model_->getDimensions() - Index2D::Ones());

  // Check if the cell overlaps with the approximate but conservative distance
  // bounds of the hierarchical range image
  const FloatingPoint d_C_cell_furthest =
      W_cell_aabb.maxDistanceTo(T_W_C.getPosition());
  const FloatingPoint range_min = d_C_cell_closest - range_threshold_behind_;
  const FloatingPoint range_max = d_C_cell_furthest + range_threshold_in_front_;
  return hierarchical_range_image_.getIntersectionType(
      min_image_idx_rectified, max_image_idx_rectified, range_min, range_max);
}
}  // namespace wavemap

#endif  // WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_RANGE_IMAGE_2D_INTERSECTOR_INL_H_
