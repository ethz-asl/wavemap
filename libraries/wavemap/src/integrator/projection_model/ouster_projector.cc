#include "wavemap/integrator/projection_model/ouster_projector.h"

#include "wavemap/utils/angle_utils.h"

namespace wavemap {
OusterProjector::OusterProjector(const OusterProjector::Config& config)
    : ProjectorBase(
          {config.elevation.num_cells, config.azimuth.num_cells},
          Vector2D(config.elevation.max_angle - config.elevation.min_angle,
                   config.azimuth.max_angle - config.azimuth.min_angle)
              .cwiseQuotient(Index2D(config.elevation.num_cells - 1,
                                     config.azimuth.num_cells - 1)
                                 .cast<FloatingPoint>()),
          {config.elevation.min_angle, config.azimuth.min_angle},
          {config.elevation.min_angle, config.azimuth.min_angle},
          {config.elevation.max_angle, config.azimuth.max_angle}),
      config_(config.checkValid()) {}

Eigen::Matrix<bool, 3, 1> OusterProjector::sensorAxisIsPeriodic() const {
  const FloatingPoint x_difference =
      angle_math::normalize_near(config_.elevation.max_angle +
                                 index_to_image_scale_factor_.x()) -
      config_.elevation.min_angle;
  const FloatingPoint y_difference =
      angle_math::normalize_near(config_.azimuth.max_angle +
                                 index_to_image_scale_factor_.y()) -
      config_.azimuth.min_angle;
  return {
      0.f <= x_difference && x_difference <= index_to_image_scale_factor_.x(),
      0.f <= y_difference && y_difference <= index_to_image_scale_factor_.y(),
      false};
}

AABB<Vector3D> OusterProjector::cartesianToSensorAABB(
    const AABB<Point3D>& W_aabb,
    const kindr::minimal::QuatTransformationTemplate<
        FloatingPoint>::RotationMatrix& R_C_W,
    const Point3D& t_W_C) const {
  AABB<Vector3D> sensor_coordinate_aabb;

  sensor_coordinate_aabb.min.z() = W_aabb.minDistanceTo(t_W_C);
  sensor_coordinate_aabb.max.z() = W_aabb.maxDistanceTo(t_W_C);

  const Point3D C_aabb_min = R_C_W * (W_aabb.min - t_W_C);
  const Transformation3D::RotationMatrix C_aabb_edges =
      R_C_W * (W_aabb.max - W_aabb.min).asDiagonal();

  std::array<Point3D, AABB<Point3D>::kNumCorners> C_aabb_corners;
  for (int corner_idx = 0; corner_idx < AABB<Point3D>::kNumCorners;
       ++corner_idx) {
    C_aabb_corners[corner_idx] = C_aabb_min;
    for (int dim_idx = 0; dim_idx < 3; ++dim_idx) {
      if ((corner_idx >> dim_idx) & 1) {
        C_aabb_corners[corner_idx] += C_aabb_edges.col(dim_idx);
      }
    }
  }

  std::array<Vector2D, AABB<Point3D>::kNumCorners> corner_sensor_coordinates;
  for (int corner_idx = 0; corner_idx < AABB<Point3D>::kNumCorners;
       ++corner_idx) {
    corner_sensor_coordinates[corner_idx] =
        cartesianToImage(C_aabb_corners[corner_idx]);
  }

  for (const int axis : {0, 1}) {
    FloatingPoint& min_coordinate = sensor_coordinate_aabb.min[axis];
    FloatingPoint& max_coordinate = sensor_coordinate_aabb.max[axis];
    for (int corner_idx = 0; corner_idx < AABB<Point3D>::kNumCorners;
         ++corner_idx) {
      const FloatingPoint coordinate =
          corner_sensor_coordinates[corner_idx][axis];
      min_coordinate = std::min(min_coordinate, coordinate);
      max_coordinate = std::max(max_coordinate, coordinate);
    }

    const bool angle_interval_wraps_around =
        sensorAxisCouldBePeriodic()[axis] &&
        kPi < (max_coordinate - min_coordinate);
    if (angle_interval_wraps_around) {
      min_coordinate = AABB<Vector3D>::kInitialMin;
      max_coordinate = AABB<Vector3D>::kInitialMax;
      for (int corner_idx = 0; corner_idx < AABB<Point3D>::kNumCorners;
           ++corner_idx) {
        const FloatingPoint angle = corner_sensor_coordinates[corner_idx][axis];
        if (0.f < angle) {
          min_coordinate = std::min(min_coordinate, angle);
        } else {
          max_coordinate = std::max(max_coordinate, angle);
        }
      }
    }
  }

  return sensor_coordinate_aabb;
}

DECLARE_CONFIG_MEMBERS(OusterProjectorConfig,
                    (elevation)
                    (azimuth)
                    (lidar_origin_to_beam_origin)
                    (lidar_origin_to_sensor_origin_z_offset));

bool OusterProjectorConfig::isValid(bool verbose) const {
  return elevation.isValid(verbose) && azimuth.isValid(verbose);
}
}  // namespace wavemap
