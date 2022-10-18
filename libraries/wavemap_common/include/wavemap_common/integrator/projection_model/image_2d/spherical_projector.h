#ifndef WAVEMAP_COMMON_INTEGRATOR_PROJECTION_MODEL_IMAGE_2D_SPHERICAL_PROJECTOR_H_
#define WAVEMAP_COMMON_INTEGRATOR_PROJECTION_MODEL_IMAGE_2D_SPHERICAL_PROJECTOR_H_

#include <utility>

#include "wavemap_common/integrator/projection_model/image_1d/circular_projector.h"
#include "wavemap_common/utils/approximate_trigonometry.h"
#include "wavemap_common/utils/config_utils.h"

namespace wavemap {
struct SphericalProjectorConfig : ConfigBase<SphericalProjectorConfig> {
  CircularProjectorConfig elevation;
  CircularProjectorConfig azimuth;

  // Constructors
  SphericalProjectorConfig() = default;
  SphericalProjectorConfig(CircularProjectorConfig elevation,
                           CircularProjectorConfig azimuth)
      : elevation(std::move(elevation)), azimuth(std::move(azimuth)) {}

  bool isValid(bool verbose) const override;
  static SphericalProjectorConfig from(const param::Map& params);
};

class SphericalProjector {
 public:
  using Config = SphericalProjectorConfig;

  explicit SphericalProjector(const Config& config)
      : elevation_projector_(config.elevation),
        azimuth_projector_(config.azimuth) {}

  IndexElement getNumRows() const { return elevation_projector_.getNumCells(); }
  IndexElement getNumColumns() const {
    return azimuth_projector_.getNumCells();
  }
  Index2D getDimensions() const { return {getNumRows(), getNumColumns()}; }

  // Coordinate transforms between Cartesian and sensor space
  static Vector3D cartesianToSensor(const Point3D& C_point) {
    const Vector2D image_coordinates = cartesianToImage(C_point);
    const FloatingPoint range = C_point.norm();
    return {image_coordinates.x(), image_coordinates.y(), range};
  }
  static Point3D sensorToCartesian(const Vector3D& coordinates) {
    const FloatingPoint elevation_angle = coordinates[0];
    const FloatingPoint azimuth_angle = coordinates[1];
    const FloatingPoint range = coordinates[2];
    const Vector3D bearing{std::cos(elevation_angle) * std::cos(azimuth_angle),
                           std::cos(elevation_angle) * std::sin(azimuth_angle),
                           std::sin(elevation_angle)};
    return range * bearing;
  }
  static Point3D sensorToCartesian(const Vector2D& image_coordinates,
                                   FloatingPoint range) {
    return sensorToCartesian(
        {image_coordinates.x(), image_coordinates.y(), range});
  }

  // Projection from Cartesian space onto the sensor's image surface
  static Vector2D cartesianToImage(const Point3D& C_point) {
    const FloatingPoint elevation_angle =
        std::atan2(C_point.z(), C_point.head<2>().norm());
    const FloatingPoint azimuth_angle = std::atan2(C_point.y(), C_point.x());
    return {elevation_angle, azimuth_angle};
  }
  // TODO(victorr): Move to base
  Index2D cartesianToIndex(const Point3D& C_point) const {
    return imageToIndex(cartesianToImage(C_point));
  }

  // Conversions between real (unscaled) coordinates on the sensor's image
  // surface and indices corresponding to sensor pixels/rays
  // TODO(victorr): Make image to scaled image and move rounding fns to base
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
  CircularProjector elevation_projector_;
  CircularProjector azimuth_projector_;
};
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_INTEGRATOR_PROJECTION_MODEL_IMAGE_2D_SPHERICAL_PROJECTOR_H_
