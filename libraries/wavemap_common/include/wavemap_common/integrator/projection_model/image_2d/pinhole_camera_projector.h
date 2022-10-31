#ifndef WAVEMAP_COMMON_INTEGRATOR_PROJECTION_MODEL_IMAGE_2D_PINHOLE_CAMERA_PROJECTOR_H_
#define WAVEMAP_COMMON_INTEGRATOR_PROJECTION_MODEL_IMAGE_2D_PINHOLE_CAMERA_PROJECTOR_H_

#include <algorithm>

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

  IndexElement getNumRows() const final { return config_.width; }
  IndexElement getNumColumns() const final { return config_.height; }
  ImageCoordinates getMinImageCoordinates() const final {
    return indexToImage(Index2D::Zero());
  }
  ImageCoordinates getMaxImageCoordinates() const final {
    return {indexToImage({config_.width - 1, config_.height - 1})};
  }
  Eigen::Matrix<bool, 3, 1> sensorAxisIsPeriodic() const final {
    return {false, false, false};
  }
  Eigen::Matrix<bool, 3, 1> sensorAxisCouldBePeriodic() const final {
    return {false, false, false};
  }

  // Coordinate transforms between Cartesian and sensor space
  Vector3D cartesianToSensor(const Point3D& C_point) const final {
    const FloatingPoint w = C_point.z();
    const FloatingPoint uw = config_.fx * C_point.x() / w + config_.cx;
    const FloatingPoint vw = config_.fy * C_point.y() / w + config_.cy;
    return {uw, vw, w};
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
  Point3D sensorToCartesian(const ImageCoordinates& image_coordinates,
                            FloatingPoint depth) const final {
    return sensorToCartesian(
        {image_coordinates.x(), image_coordinates.y(), depth});
  }
  FloatingPoint imageOffsetToErrorNorm(
      const ImageCoordinates& /*linearization_point*/,
      ImageCoordinates offset) const final {
    return offset.norm();
  }

  // Projection from Cartesian space onto the sensor's image surface
  ImageCoordinates cartesianToImage(const Point3D& C_point) const final {
    return cartesianToSensor(C_point).head<2>();
  }
  FloatingPoint cartesianToSensorZ(
      const wavemap::Point3D& C_point) const final {
    return C_point.z();
  }

  AABB<Vector3D> cartesianToSensorAABB(
      const AABB<wavemap::Point3D>& W_aabb,
      const wavemap::Transformation3D& T_W_C) const final {
    AABB<Vector3D> sensor_coordinate_aabb;

    const Transformation3D T_C_W = T_W_C.inverse();
    for (int corner_idx = 0; corner_idx < AABB<Point3D>::kNumCorners;
         ++corner_idx) {
      const Point3D C_t_C_corner = T_C_W * W_aabb.corner_point(corner_idx);
      const Vector3D corner_sensor_coordinates =
          cartesianToSensorClamped(C_t_C_corner);
      sensor_coordinate_aabb.includePoint(corner_sensor_coordinates);
    }

    return sensor_coordinate_aabb;
  }

 private:
  const PinholeCameraProjectorConfig config_;

  const FloatingPoint fxfy_ = config_.fx * config_.fy;
  const FloatingPoint fxfy_inv_ = 1.f / fxfy_;
  const FloatingPoint cxfy_ = config_.cx * config_.fy;
  const FloatingPoint cyfx_ = config_.cy * config_.fx;

  Vector3D cartesianToSensorClamped(const Point3D& C_point) const {
    const FloatingPoint w_clamped = std::max(C_point.z(), kEpsilon);
    const FloatingPoint uw = config_.fx * C_point.x() / w_clamped + config_.cx;
    const FloatingPoint vw = config_.fy * C_point.y() / w_clamped + config_.cy;
    return {uw, vw, C_point.z()};
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_INTEGRATOR_PROJECTION_MODEL_IMAGE_2D_PINHOLE_CAMERA_PROJECTOR_H_
