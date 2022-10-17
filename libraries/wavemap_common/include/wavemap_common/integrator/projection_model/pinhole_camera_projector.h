#ifndef WAVEMAP_COMMON_INTEGRATOR_PROJECTION_MODEL_PINHOLE_CAMERA_PROJECTOR_H_
#define WAVEMAP_COMMON_INTEGRATOR_PROJECTION_MODEL_PINHOLE_CAMERA_PROJECTOR_H_

#include "wavemap_common/utils/config_utils.h"

namespace wavemap {
struct PinholeCameraProjectorConfig : ConfigBase<PinholeCameraProjectorConfig> {
  IndexElement width = 0;
  IndexElement height = 0;
  FloatingPoint fx = 0.f;
  FloatingPoint fy = 0.f;
  FloatingPoint cx = 0.f;
  FloatingPoint cy = 0.f;

  // Constructors
  PinholeCameraProjectorConfig() = default;
  PinholeCameraProjectorConfig(FloatingPoint fx, FloatingPoint fy,
                               FloatingPoint cx, FloatingPoint cy,
                               IndexElement height, IndexElement width)
      : width(width), height(height), fx(fx), fy(fy), cx(cx), cy(cy) {}

  bool isValid(bool verbose) const override;
  static PinholeCameraProjectorConfig from(const param::Map& params);
};

class PinholeCameraProjector {
 public:
  explicit PinholeCameraProjector(const PinholeCameraProjectorConfig& config)
      : config_(config.checkValid()) {}

  IndexElement getNumRows() const { return config_.height; }
  IndexElement getNumColumns() const { return config_.width; }
  Index2D getDimensions() const { return {getNumRows(), getNumColumns()}; }

  // Coordinate transforms between Cartesian and sensor space
  Vector3D cartesianToSensor(const Point3D& C_point) const {
    const FloatingPoint u = config_.fx * C_point.x() + config_.cx * C_point.z();
    const FloatingPoint v = config_.fy * C_point.y() + config_.cy * C_point.z();
    const FloatingPoint w = C_point.z();
    return {u / w, v / w, w};
  }
  Point3D sensorToCartesian(const Vector3D& coordinates) const {
    const FloatingPoint w = coordinates[2];
    const FloatingPoint u = w * coordinates[0];
    const FloatingPoint v = w * coordinates[1];
    const FloatingPoint x = config_.fx * u;
    const FloatingPoint y = config_.fy * v;
    const FloatingPoint z = config_.cx * u + config_.cy * v + w;
    return {x, y, z};
  }
  Point3D sensorToCartesian(const Vector2D& image_coordinates,
                            FloatingPoint depth) const {
    return sensorToCartesian(
        {image_coordinates.x(), image_coordinates.y(), depth});
  }

  // Projection from Cartesian space onto the sensor's image surface
  Vector2D cartesianToImage(const Point3D& C_point) const {
    const FloatingPoint u = config_.fx * C_point.x() + config_.cx * C_point.z();
    const FloatingPoint v = config_.fy * C_point.y() + config_.cy * C_point.z();
    const FloatingPoint w = C_point.z();
    return {u / w, v / w};
  }

  // Conversions between real (unscaled) coordinates on the sensor's image
  // surface and indices corresponding to sensor pixels/rays
  static Index2D imageToIndex(const Vector2D& image_coordinates) {
    return {std::round(image_coordinates.x()),
            std::round(image_coordinates.y())};
  }
  static Vector2D indexToImage(const Index2D& index) {
    return index.cast<FloatingPoint>();
  }

 private:
  const PinholeCameraProjectorConfig config_;
};
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_INTEGRATOR_PROJECTION_MODEL_PINHOLE_CAMERA_PROJECTOR_H_
