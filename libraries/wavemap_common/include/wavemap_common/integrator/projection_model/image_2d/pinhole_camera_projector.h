#ifndef WAVEMAP_COMMON_INTEGRATOR_PROJECTION_MODEL_IMAGE_2D_PINHOLE_CAMERA_PROJECTOR_H_
#define WAVEMAP_COMMON_INTEGRATOR_PROJECTION_MODEL_IMAGE_2D_PINHOLE_CAMERA_PROJECTOR_H_

#include "wavemap_common/integrator/projection_model/image_2d/image_2d_projection_model.h"
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

class PinholeCameraProjector : public Image2DProjectionModel {
 public:
  using Config = PinholeCameraProjectorConfig;

  explicit PinholeCameraProjector(const Config& config)
      : Image2DProjectionModel(Vector2D::Ones(), Vector2D::Zero()),
        config_(config.checkValid()) {}

  IndexElement getNumRows() const final { return config_.height; }
  IndexElement getNumColumns() const final { return config_.width; }
  Vector2D getMinImageCoordinates() const final {
    return indexToImage(Index2D::Zero());
  }
  Vector2D getMaxImageCoordinates() const final {
    return {indexToImage({config_.width, config_.height})};
  }

  // Coordinate transforms between Cartesian and sensor space
  Vector3D cartesianToSensor(const Point3D& C_point) const final {
    const FloatingPoint u = config_.fx * C_point.x() + config_.cx * C_point.z();
    const FloatingPoint v = config_.fy * C_point.y() + config_.cy * C_point.z();
    const FloatingPoint w = C_point.z();
    return {u / w, v / w, w};
  }
  Point3D sensorToCartesian(const Vector3D& coordinates) const final {
    const FloatingPoint u_scaled = coordinates[0];
    const FloatingPoint v_scaled = coordinates[1];
    const FloatingPoint w = coordinates[2];
    Point3D C_point = w * fxfy_inv_ *
                      Point3D{config_.fy * u_scaled - cxfy_,
                              config_.fx * v_scaled - cyfx_, fxfy_};
    return C_point;
  }
  Point3D sensorToCartesian(const Vector2D& image_coordinates,
                            FloatingPoint depth) const final {
    return sensorToCartesian(
        {image_coordinates.x(), image_coordinates.y(), depth});
  }

  // Projection from Cartesian space onto the sensor's image surface
  Vector2D cartesianToImage(const Point3D& C_point) const final {
    return cartesianToSensor(C_point).head<2>();
  }

 private:
  const PinholeCameraProjectorConfig config_;

  const FloatingPoint fxfy_ = config_.fx * config_.fy;
  const FloatingPoint fxfy_inv_ = 1.f / fxfy_;
  const FloatingPoint cxfy_ = config_.cx * config_.fy;
  const FloatingPoint cyfx_ = config_.cy * config_.fx;
};
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_INTEGRATOR_PROJECTION_MODEL_IMAGE_2D_PINHOLE_CAMERA_PROJECTOR_H_
