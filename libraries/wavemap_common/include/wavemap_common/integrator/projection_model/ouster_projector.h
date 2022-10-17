#ifndef WAVEMAP_COMMON_INTEGRATOR_PROJECTION_MODEL_OUSTER_PROJECTOR_H_
#define WAVEMAP_COMMON_INTEGRATOR_PROJECTION_MODEL_OUSTER_PROJECTOR_H_

#include <utility>

#include "wavemap_common/integrator/projection_model/circular_projector.h"
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

class OusterProjector {
 public:
  explicit OusterProjector(const OusterProjectorConfig& config)
      : config_(config.checkValid()),
        elevation_projector_(config_.elevation),
        azimuth_projector_(config_.azimuth) {}

  IndexElement getNumRows() const { return elevation_projector_.getNumCells(); }
  IndexElement getNumColumns() const {
    return azimuth_projector_.getNumCells();
  }
  Index2D getDimensions() const { return {getNumRows(), getNumColumns()}; }

  // Coordinate transforms between Cartesian and sensor space
  Vector3D cartesianToSensor(const Point3D& C_point) const {
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
  Point3D sensorToCartesian(const Vector3D& coordinates) const {
    const FloatingPoint elevation_angle = coordinates[0];
    const FloatingPoint azimuth_angle = coordinates[1];
    const FloatingPoint range = coordinates[2];
    Point2D B_point =
        Vector2D{std::cos(elevation_angle), std::sin(elevation_angle)};
    B_point *= range;
    B_point += Vector2D{config_.lidar_origin_to_beam_origin,
                        config_.lidar_origin_to_sensor_origin_z_offset};
    Point3D C_point{B_point.x() * std::cos(azimuth_angle),
                    B_point.x() * std::sin(azimuth_angle), B_point.y()};
    return C_point;
  }
  Point3D sensorToCartesian(const Vector2D& image_coordinates,
                            FloatingPoint range) const {
    return sensorToCartesian(
        {image_coordinates.x(), image_coordinates.y(), range});
  }

  // Projection from Cartesian space onto the sensor's image surface
  Vector2D cartesianToImage(const Point3D& C_point) const {
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

  // Conversions between real (unscaled) coordinates on the sensor's image
  // surface and indices corresponding to sensor pixels/rays
  Index2D imageToIndex(const Vector2D& image_coordinates) const {
    const FloatingPoint elevation_angle = image_coordinates.x();
    const FloatingPoint azimuth_angle = image_coordinates.y();
    return {elevation_projector_.angleToNearestIndex(elevation_angle),
            azimuth_projector_.angleToNearestIndex(azimuth_angle)};
  }
  Vector2D indexToImage(const Index2D& index) const {
    return {elevation_projector_.indexToAngle(index.x()),
            azimuth_projector_.indexToAngle(index.y())};
  }

 private:
  const OusterProjectorConfig config_;
  CircularProjector elevation_projector_;
  CircularProjector azimuth_projector_;
};
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_INTEGRATOR_PROJECTION_MODEL_OUSTER_PROJECTOR_H_
