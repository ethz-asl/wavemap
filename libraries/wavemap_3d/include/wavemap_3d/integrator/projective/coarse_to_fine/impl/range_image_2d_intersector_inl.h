#ifndef WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_RANGE_IMAGE_2D_INTERSECTOR_INL_H_
#define WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_RANGE_IMAGE_2D_INTERSECTOR_INL_H_

#include <algorithm>
#include <bitset>
#include <limits>

#include <wavemap_common/integrator/measurement_model/range_and_angle/continuous_volumetric_log_odds.h>
#include <wavemap_common/utils/angle_utils.h>
#include <wavemap_common/utils/approximate_trigonometry.h>

namespace wavemap {
inline RangeImage2DIntersector::MinMaxSensorCoordinates
RangeImage2DIntersector::getAabbMinMaxProjectedAngle(
    const Transformation3D& T_W_C, const AABB<Point3D>& W_aabb) const {
  Cache cache{};
  return getAabbMinMaxProjectedAngle(T_W_C, W_aabb, *projection_model_, cache);
}

inline RangeImage2DIntersector::MinMaxSensorCoordinates
RangeImage2DIntersector::getAabbMinMaxProjectedAngle(
    const Transformation3D& T_W_C, const AABB<Point3D>& W_aabb,
    RangeImage2DIntersector::Cache& cache) const {
  return getAabbMinMaxProjectedAngle(T_W_C, W_aabb, *projection_model_, cache);
}

inline RangeImage2DIntersector::MinMaxSensorCoordinates
RangeImage2DIntersector::getAabbMinMaxProjectedAngle(
    const Transformation3D& T_W_C, const AABB<Point3D>& W_aabb,
    const Image2DProjectionModel& projection_model) {
  Cache cache{};
  return getAabbMinMaxProjectedAngle(T_W_C, W_aabb, projection_model, cache);
}

// TODO(victorr): Move and specialize getAabbMinMaxProjectedAngle method for
// each projector type
inline RangeImage2DIntersector::MinMaxSensorCoordinates
RangeImage2DIntersector::getAabbMinMaxProjectedAngle(
    const Transformation3D& T_W_C, const AABB<Point3D>& W_aabb,
    const Image2DProjectionModel& projection_model,
    RangeImage2DIntersector::Cache& cache) {
  MinMaxSensorCoordinates sensor_coordinate_interval;

  // If the sensor is contained in the AABB, it overlaps with the full range
  if (W_aabb.containsPoint(T_W_C.getPosition())) {
    sensor_coordinate_interval.min_sensor_coordinates.head<2>() =
        Vector2D::Constant(std::numeric_limits<FloatingPoint>::lowest());
    sensor_coordinate_interval.max_sensor_coordinates.head<2>() =
        Vector2D::Constant(std::numeric_limits<FloatingPoint>::max());
    sensor_coordinate_interval.min_sensor_coordinates.z() =
        std::numeric_limits<FloatingPoint>::lowest();
    sensor_coordinate_interval.max_sensor_coordinates.z() =
        std::numeric_limits<FloatingPoint>::max();
    return sensor_coordinate_interval;
  }

  const Transformation3D T_C_W = T_W_C.inverse();

  if (cache.has_value()) {
    for (auto [sensor_coordinates, corner_indices] :
         {std::pair{&sensor_coordinate_interval.min_sensor_coordinates,
                    cache.value().min_corner_indices},
          {&sensor_coordinate_interval.max_sensor_coordinates,
           cache.value().max_corner_indices}}) {
      for (const int axis : {0, 1, 2}) {
        const Point3D corner_point =
            T_C_W * W_aabb.corner_point(corner_indices[axis]);
        sensor_coordinates->operator[](axis) =
            projection_model.cartesianToSensor(corner_point)[axis];
      }
    }

    // TODO(victorr): Remove this nasty fix when refactoring
    sensor_coordinate_interval.min_sensor_coordinates.z() =
        std::min(sensor_coordinate_interval.min_sensor_coordinates.z(),
                 W_aabb.minDistanceTo(T_W_C.getPosition()));
    return sensor_coordinate_interval;
  }

  Eigen::Matrix<FloatingPoint, 3, 8> corner_sensor_coordinates;
  std::bitset<3> all_positive{0b111};
  std::bitset<3> all_negative{0b111};
  for (int corner_idx = 0; corner_idx < AABB<Point3D>::kNumCorners;
       ++corner_idx) {
    const Point3D C_t_C_corner = T_C_W * W_aabb.corner_point(corner_idx);
    // Check the octant of this corner point
    for (int dim_idx = 0; dim_idx < 3; ++dim_idx) {
      if (bool is_negative = std::signbit(C_t_C_corner[dim_idx]); is_negative) {
        all_positive.set(dim_idx, false);
      } else {
        all_negative.set(dim_idx, false);
      }
    }
    // Compute its sensor coordinates
    corner_sensor_coordinates.col(corner_idx) =
        projection_model.cartesianToSensor(C_t_C_corner);
    if (corner_sensor_coordinates.col(corner_idx).z() < 0.f) {
      corner_sensor_coordinates.col(corner_idx).z() = 0.f;
      corner_sensor_coordinates.col(corner_idx).head<2>() *= -1e6f;
    }
  }
  const bool all_corners_in_same_octant = (all_positive | all_negative).all();

  if (all_corners_in_same_octant) {
    cache.emplace();
    for (const int axis : {0, 1, 2}) {
      auto& min_coordinate =
          sensor_coordinate_interval.min_sensor_coordinates[axis];
      auto& max_coordinate =
          sensor_coordinate_interval.max_sensor_coordinates[axis];

      min_coordinate = corner_sensor_coordinates.row(axis).minCoeff(
          &cache.value().min_corner_indices[axis]);
      max_coordinate = corner_sensor_coordinates.row(axis).maxCoeff(
          &cache.value().max_corner_indices[axis]);

      const bool angle_interval_wraps_around =
          projection_model.sensorAxisCouldBePeriodic()[axis] &&
          kPi < (max_coordinate - min_coordinate);
      if (angle_interval_wraps_around) {
        min_coordinate = MinMaxSensorCoordinates::kInitialMin;
        max_coordinate = MinMaxSensorCoordinates::kInitialMax;
        for (int corner_idx = 0; corner_idx < AABB<Point3D>::kNumCorners;
             ++corner_idx) {
          const FloatingPoint angle =
              corner_sensor_coordinates(axis, corner_idx);
          if (0.f < angle) {
            if (min_coordinate < angle) {
              min_coordinate = angle;
              cache.value().min_corner_indices[axis] = corner_idx;
            }
          } else {
            if (angle < max_coordinate) {
              max_coordinate = angle;
              cache.value().max_corner_indices[axis] = corner_idx;
            }
          }
        }
      }
    }

    // TODO(victorr): Remove this nasty fix when refactoring
    sensor_coordinate_interval.min_sensor_coordinates.z() =
        std::min(sensor_coordinate_interval.min_sensor_coordinates.z(),
                 W_aabb.minDistanceTo(T_W_C.getPosition()));
    return sensor_coordinate_interval;
  }

  for (const int axis : {0, 1, 2}) {
    auto& min_coordinate =
        sensor_coordinate_interval.min_sensor_coordinates[axis];
    auto& max_coordinate =
        sensor_coordinate_interval.max_sensor_coordinates[axis];

    min_coordinate = corner_sensor_coordinates.row(axis).minCoeff();
    max_coordinate = corner_sensor_coordinates.row(axis).maxCoeff();

    const bool angle_interval_wraps_around =
        projection_model.sensorAxisCouldBePeriodic()[axis] &&
        kPi < (max_coordinate - min_coordinate);
    if (angle_interval_wraps_around) {
      min_coordinate = MinMaxSensorCoordinates::kInitialMin;
      max_coordinate = MinMaxSensorCoordinates::kInitialMax;
      for (int corner_idx = 0; corner_idx < AABB<Point3D>::kNumCorners;
           ++corner_idx) {
        const FloatingPoint angle = corner_sensor_coordinates(axis, corner_idx);
        if (0.f < angle) {
          min_coordinate = std::min(min_coordinate, angle);
        } else {
          max_coordinate = std::max(max_coordinate, angle);
        }
      }
    }
  }

  // TODO(victorr): Remove this nasty fix when refactoring
  sensor_coordinate_interval.min_sensor_coordinates.z() =
      std::min(sensor_coordinate_interval.min_sensor_coordinates.z(),
               W_aabb.minDistanceTo(T_W_C.getPosition()));
  return sensor_coordinate_interval;
}

inline IntersectionType RangeImage2DIntersector::determineIntersectionType(
    const Transformation3D& T_W_C, const AABB<Point3D>& W_cell_aabb) const {
  Cache cache{};
  return determineIntersectionType(T_W_C, W_cell_aabb, cache);
}

inline IntersectionType RangeImage2DIntersector::determineIntersectionType(
    const Transformation3D& T_W_C, const AABB<Point3D>& W_cell_aabb,
    Cache& cache) const {
  // Get the min and max angles for any point in the cell projected into the
  // range image
  auto [min_sensor_coordinates, max_sensor_coordinates] =
      getAabbMinMaxProjectedAngle(T_W_C, W_cell_aabb, cache);
  if (min_sensor_coordinates.z() < 0.f && 0.f < max_sensor_coordinates.z()) {
    return IntersectionType::kPossiblyOccupied;
  }
  if (max_range_ < min_sensor_coordinates.z() ||
      max_sensor_coordinates.z() <= 0.f) {
    return IntersectionType::kFullyUnknown;
  }

  // Pad the min and max angles with the BeamModel's angle threshold to
  // account for the beam's non-zero width (angular uncertainty)
  const Index2D min_image_index = projection_model_->imageToFloorIndex(
      min_sensor_coordinates.head<2>() - Vector2D::Constant(angle_threshold_));
  const Index2D max_image_index = projection_model_->imageToCeilIndex(
      max_sensor_coordinates.head<2>() + Vector2D::Constant(angle_threshold_));

  // If the angle wraps around Pi, we can't use the hierarchical range image
  const bool x_range_wraps_around = max_image_index.x() < min_image_index.x();
  const bool y_range_wraps_around = max_image_index.y() < min_image_index.y();
  if (x_range_wraps_around || (!y_axis_wraps_around_ && y_range_wraps_around)) {
    const bool x_range_fully_outside_fov =
        max_image_index.x() < 0 &&
        projection_model_->getNumRows() < min_image_index.x();
    const bool y_range_fully_outside_fov =
        max_image_index.y() < 0 &&
        projection_model_->getNumColumns() < min_image_index.y();
    if ((!x_range_wraps_around || x_range_fully_outside_fov) &&
        (y_axis_wraps_around_ || !y_range_wraps_around ||
         y_range_fully_outside_fov)) {
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
  const FloatingPoint min_z_coordinate =
      min_sensor_coordinates.z() - range_threshold_behind_;
  const FloatingPoint max_z_coordinate =
      max_sensor_coordinates.z() + range_threshold_in_front_;
  return hierarchical_range_image_.getIntersectionType(
      min_image_idx_rectified, max_image_idx_rectified, min_z_coordinate,
      max_z_coordinate);
}
}  // namespace wavemap

#endif  // WAVEMAP_3D_INTEGRATOR_PROJECTIVE_COARSE_TO_FINE_IMPL_RANGE_IMAGE_2D_INTERSECTOR_INL_H_
