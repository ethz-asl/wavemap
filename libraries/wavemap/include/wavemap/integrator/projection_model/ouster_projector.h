#ifndef WAVEMAP_INTEGRATOR_PROJECTION_MODEL_OUSTER_PROJECTOR_H_
#define WAVEMAP_INTEGRATOR_PROJECTION_MODEL_OUSTER_PROJECTOR_H_

#include <algorithm>
#include <utility>

#include "wavemap/config/config_base.h"
#include "wavemap/integrator/projection_model/circular_projector.h"
#include "wavemap/integrator/projection_model/projector_base.h"
#include "wavemap/utils/approximate_trigonometry.h"

namespace wavemap {
struct OusterProjectorConfig : ConfigBase<OusterProjectorConfig> {
  CircularProjectorConfig elevation;
  CircularProjectorConfig azimuth;
  FloatingPoint lidar_origin_to_beam_origin = 0.02767f;
  FloatingPoint lidar_origin_to_sensor_origin_z_offset = 0.03618f;

  // Constructors
  OusterProjectorConfig() = default;
  OusterProjectorConfig(CircularProjectorConfig elevation,
                        CircularProjectorConfig azimuth,
                        FloatingPoint lidar_origin_to_beam_origin,
                        FloatingPoint lidar_origin_to_sensor_origin_z_offset)
      : elevation(std::move(elevation)),
        azimuth(std::move(azimuth)),
        lidar_origin_to_beam_origin(lidar_origin_to_beam_origin),
        lidar_origin_to_sensor_origin_z_offset(
            lidar_origin_to_sensor_origin_z_offset) {}

  bool isValid(bool verbose) const override;
  static OusterProjectorConfig from(const param::Map& params);
};

class OusterProjector : public ProjectorBase {
 public:
  using Config = OusterProjectorConfig;

  explicit OusterProjector(const Config& config)
      : ProjectorBase(
            Vector2D(config.elevation.max_angle - config.elevation.min_angle,
                     config.azimuth.max_angle - config.azimuth.min_angle)
                .cwiseQuotient(Index2D(config.elevation.num_cells - 1,
                                       config.azimuth.num_cells - 1)
                                   .cast<FloatingPoint>()),
            Vector2D(config.elevation.min_angle, config.azimuth.min_angle)),
        config_(config.checkValid()) {}

  IndexElement getNumRows() const final { return config_.elevation.num_cells; }
  IndexElement getNumColumns() const final { return config_.azimuth.num_cells; }
  Vector2D getMinImageCoordinates() const final {
    return {config_.elevation.min_angle, config_.azimuth.min_angle};
  }
  Vector2D getMaxImageCoordinates() const final {
    return {config_.elevation.max_angle, config_.azimuth.max_angle};
  }
  Eigen::Matrix<bool, 3, 1> sensorAxisIsPeriodic() const final;
  Eigen::Matrix<bool, 3, 1> sensorAxisCouldBePeriodic() const final {
    return {true, true, false};
  }
  SiUnit getImageCoordinatesUnit() const final { return SiUnit::kRadians; }

  // Coordinate transforms between Cartesian and sensor space
  Vector3D cartesianToSensor(const Point3D& C_point) const final {
    // Project the beam's endpoint into the 2D plane B whose origin lies at the
    // beam's start point, X-axis is parallel to the projection of the beam onto
    // frame C's XY-plane and Y-axis is parallel to frame C's Z-axis
    const Point2D B_point{
        C_point.head<2>().norm() - config_.lidar_origin_to_beam_origin,
        C_point.z() - config_.lidar_origin_to_sensor_origin_z_offset};
    const FloatingPoint elevation_angle = std::atan2(B_point.y(), B_point.x());
    const FloatingPoint azimuth_angle = std::atan2(C_point.y(), C_point.x());
    const FloatingPoint range = B_point.norm();
    return {elevation_angle, azimuth_angle, range};
  }
  Point3D sensorToCartesian(const Vector3D& coordinates) const final {
    const FloatingPoint elevation_angle = coordinates[0];
    const FloatingPoint azimuth_angle = coordinates[1];
    const FloatingPoint range = coordinates[2];
    const Point2D B_point =
        range * Vector2D(std::cos(elevation_angle), std::sin(elevation_angle)) +
        Vector2D(config_.lidar_origin_to_beam_origin,
                 config_.lidar_origin_to_sensor_origin_z_offset);
    Point3D C_point{B_point.x() * std::cos(azimuth_angle),
                    B_point.x() * std::sin(azimuth_angle), B_point.y()};
    return C_point;
  }
  Point3D sensorToCartesian(const Vector2D& image_coordinates,
                            FloatingPoint range) const final {
    return sensorToCartesian(
        {image_coordinates.x(), image_coordinates.y(), range});
  }
  FloatingPoint imageOffsetToErrorNorm(const Vector2D& linearization_point,
                                       Vector2D offset) const final {
    // Scale the azimuth offset by the cosine of the elevation angle to account
    // for the change in density along the azimuth axis in function of elevation
    const FloatingPoint cos_elevation_angle = std::cos(linearization_point[0]);
    offset[1] *= cos_elevation_angle;
    return offset.norm();
  }

  // Projection from Cartesian space onto the sensor's image surface
  Vector2D cartesianToImage(const Point3D& C_point) const final {
    // Project the beam's endpoint into the 2D plane B whose origin lies at the
    // beam's start point, X-axis is parallel to the projection of the beam onto
    // frame C's XY-plane and Y-axis is parallel to frame C's Z-axis
    const Vector2D B_point{
        C_point.head<2>().norm() - config_.lidar_origin_to_beam_origin,
        C_point.z() - config_.lidar_origin_to_sensor_origin_z_offset};
    const FloatingPoint elevation_angle = std::atan2(B_point.y(), B_point.x());
    const FloatingPoint azimuth_angle = std::atan2(C_point.y(), C_point.x());
    return {elevation_angle, azimuth_angle};
  }
  FloatingPoint cartesianToSensorZ(const Point3D& C_point) const final {
    const Point2D B_point{
        C_point.head<2>().norm() - config_.lidar_origin_to_beam_origin,
        C_point.z() - config_.lidar_origin_to_sensor_origin_z_offset};
    return B_point.norm();
  }

  // NOTE: When the AABB is right behind the sensor, the angle range will wrap
  //       around at +-PI and a min_angle >= max_angle will be returned.
  AABB<Vector3D> cartesianToSensorAABB(
      const AABB<Point3D>& W_aabb,
      const Transformation3D::RotationMatrix& R_C_W,
      const Point3D& t_W_C) const final {
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
          cartesianToImageApprox(C_aabb_corners[corner_idx]);
    }

    for (const int axis : {0, 1}) {
      FloatingPoint& min_coordinate = sensor_coordinate_aabb.min[axis];
      FloatingPoint& max_coordinate = sensor_coordinate_aabb.max[axis];
      for (int corner_idx = 0; corner_idx < AABB<Point3D>::kNumCorners;
           ++corner_idx) {
        min_coordinate = std::min(min_coordinate,
                                  corner_sensor_coordinates[corner_idx][axis]);
        max_coordinate = std::max(max_coordinate,
                                  corner_sensor_coordinates[corner_idx][axis]);
      }

      const bool angle_interval_wraps_around =
          sensorAxisCouldBePeriodic()[axis] &&
          kPi < (max_coordinate - min_coordinate);
      if (angle_interval_wraps_around) {
        min_coordinate = AABB<Vector3D>::kInitialMin;
        max_coordinate = AABB<Vector3D>::kInitialMax;
        for (int corner_idx = 0; corner_idx < AABB<Point3D>::kNumCorners;
             ++corner_idx) {
          const FloatingPoint angle =
              corner_sensor_coordinates[corner_idx][axis];
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

 private:
  const OusterProjectorConfig config_;

  Vector2D cartesianToImageApprox(const Point3D& C_point) const {
    const Vector2D B_point{
        C_point.head<2>().norm() - config_.lidar_origin_to_beam_origin,
        C_point.z() - config_.lidar_origin_to_sensor_origin_z_offset};
    const FloatingPoint elevation_angle =
        approximate::atan2()(B_point.y(), B_point.x());
    const FloatingPoint azimuth_angle =
        approximate::atan2()(C_point.y(), C_point.x());
    return {elevation_angle, azimuth_angle};
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_INTEGRATOR_PROJECTION_MODEL_OUSTER_PROJECTOR_H_
