#ifndef WAVEMAP_COMMON_INTEGRATOR_PROJECTION_MODEL_IMAGE_2D_OUSTER_PROJECTOR_H_
#define WAVEMAP_COMMON_INTEGRATOR_PROJECTION_MODEL_IMAGE_2D_OUSTER_PROJECTOR_H_

#include <algorithm>
#include <utility>

#include "wavemap_common/integrator/projection_model/image_1d/circular_projector.h"
#include "wavemap_common/integrator/projection_model/image_2d/image_2d_projection_model.h"
#include "wavemap_common/utils/approximate_trigonometry.h"
#include "wavemap_common/utils/config_utils.h"

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

class OusterProjector : public Image2DProjectionModel {
 public:
  using Config = OusterProjectorConfig;

  explicit OusterProjector(const Config& config)
      : Image2DProjectionModel(
            Vector2D(config.elevation.max_angle - config.elevation.min_angle,
                     config.azimuth.max_angle - config.azimuth.min_angle)
                .cwiseQuotient(Index2D(config.elevation.num_cells - 1,
                                       config.azimuth.num_cells - 1)
                                   .cast<FloatingPoint>()),
            Vector2D(config.elevation.min_angle, config.azimuth.min_angle)),
        config_(config.checkValid()) {}

  IndexElement getNumRows() const final { return config_.elevation.num_cells; }
  IndexElement getNumColumns() const final { return config_.azimuth.num_cells; }
  ImageCoordinates getMinImageCoordinates() const final {
    return {config_.elevation.min_angle, config_.azimuth.min_angle};
  }
  ImageCoordinates getMaxImageCoordinates() const final {
    return {config_.elevation.max_angle, config_.azimuth.max_angle};
  }
  Eigen::Matrix<bool, 3, 1> sensorAxisIsPeriodic() const final;
  Eigen::Matrix<bool, 3, 1> sensorAxisCouldBePeriodic() const final {
    return {true, true, false};
  }

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
  Point3D sensorToCartesian(const ImageCoordinates& image_coordinates,
                            FloatingPoint range) const final {
    return sensorToCartesian(
        {image_coordinates.x(), image_coordinates.y(), range});
  }
  FloatingPoint imageOffsetToErrorNorm(
      const ImageCoordinates& linearization_point,
      ImageCoordinates offset) const final {
    // Scale the azimuth offset by the cosine of the elevation angle to account
    // for the change in density along the azimuth axis in function of elevation
    const FloatingPoint cos_elevation_angle = std::cos(linearization_point[0]);
    offset[1] *= cos_elevation_angle;
    return offset.norm();
  }

  // Projection from Cartesian space onto the sensor's image surface
  ImageCoordinates cartesianToImage(const Point3D& C_point) const final {
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
  FloatingPoint cartesianToSensorZ(
      const wavemap::Point3D& C_point) const final {
    const Point2D B_point{
        C_point.head<2>().norm() - config_.lidar_origin_to_beam_origin,
        C_point.z() - config_.lidar_origin_to_sensor_origin_z_offset};
    return B_point.norm();
  }

  // NOTE: When the AABB is right behind the sensor, the angle range will wrap
  //       around at +-PI and a min_angle >= max_angle will be returned.
  AABB<Vector3D> cartesianToSensorAABB(
      const AABB<wavemap::Point3D>& W_aabb,
      const wavemap::Transformation3D& T_W_C) const final {
    AABB<Vector3D> sensor_coordinate_aabb;

    sensor_coordinate_aabb.min.z() = W_aabb.minDistanceTo(T_W_C.getPosition());
    sensor_coordinate_aabb.max.z() = W_aabb.maxDistanceTo(T_W_C.getPosition());

    const Transformation3D T_C_W = T_W_C.inverse();
    Eigen::Matrix<FloatingPoint, 2, 8> corner_sensor_coordinates;
    for (int corner_idx = 0; corner_idx < AABB<Point3D>::kNumCorners;
         ++corner_idx) {
      const Point3D C_t_C_corner = T_C_W * W_aabb.corner_point(corner_idx);
      corner_sensor_coordinates.col(corner_idx) =
          cartesianToImageApprox(C_t_C_corner);
    }

    for (const int axis : {0, 1}) {
      auto& min_coordinate = sensor_coordinate_aabb.min[axis];
      auto& max_coordinate = sensor_coordinate_aabb.max[axis];

      min_coordinate = corner_sensor_coordinates.row(axis).minCoeff();
      max_coordinate = corner_sensor_coordinates.row(axis).maxCoeff();

      const bool angle_interval_wraps_around =
          sensorAxisCouldBePeriodic()[axis] &&
          kPi < (max_coordinate - min_coordinate);
      if (angle_interval_wraps_around) {
        min_coordinate = AABB<Vector3D>::kInitialMin;
        max_coordinate = AABB<Vector3D>::kInitialMax;
        for (int corner_idx = 0; corner_idx < AABB<Point3D>::kNumCorners;
             ++corner_idx) {
          const FloatingPoint angle =
              corner_sensor_coordinates(axis, corner_idx);
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

  ImageCoordinates cartesianToImageApprox(const Point3D& C_point) const {
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

#endif  // WAVEMAP_COMMON_INTEGRATOR_PROJECTION_MODEL_IMAGE_2D_OUSTER_PROJECTOR_H_
