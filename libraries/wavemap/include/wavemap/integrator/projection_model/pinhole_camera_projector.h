#ifndef WAVEMAP_INTEGRATOR_PROJECTION_MODEL_PINHOLE_CAMERA_PROJECTOR_H_
#define WAVEMAP_INTEGRATOR_PROJECTION_MODEL_PINHOLE_CAMERA_PROJECTOR_H_

#include <algorithm>

#include "wavemap/config/config_base.h"
#include "wavemap/integrator/projection_model/projector_base.h"

namespace wavemap {
/**
 * Config struct for the pinhole camera projection model.
 */
struct PinholeCameraProjectorConfig
    : ConfigBase<PinholeCameraProjectorConfig, 6> {
  //! The image's width in pixels.
  IndexElement width = 0;
  //! The image's height in pixels.
  IndexElement height = 0;
  //! Fx according to ROS' CameraInfo convention:
  //! http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html.
  FloatingPoint fx = 0.f;
  //! Fy according to ROS' CameraInfo convention:
  //! http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html.
  FloatingPoint fy = 0.f;
  //! Cx according to ROS' CameraInfo convention:
  //! http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html.
  FloatingPoint cx = 0.f;
  //! Cy according to ROS' CameraInfo convention:
  //! http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html.
  FloatingPoint cy = 0.f;

  static MemberMap memberMap;

  // Constructors
  PinholeCameraProjectorConfig() = default;
  PinholeCameraProjectorConfig(FloatingPoint fx, FloatingPoint fy,
                               FloatingPoint cx, FloatingPoint cy,
                               IndexElement height, IndexElement width)
      : width(width), height(height), fx(fx), fy(fy), cx(cx), cy(cy) {}

  bool isValid(bool verbose) const override;
};

class PinholeCameraProjector : public ProjectorBase {
 public:
  using Config = PinholeCameraProjectorConfig;

  explicit PinholeCameraProjector(const Config& config);

  Eigen::Matrix<bool, 3, 1> sensorAxisIsPeriodic() const final {
    return {false, false, false};
  }
  Eigen::Matrix<bool, 3, 1> sensorAxisCouldBePeriodic() const final {
    return {false, false, false};
  }
  SiUnit getImageCoordinatesUnit() const final { return SiUnit::kPixels; }

  // Coordinate transforms between Cartesian and sensor space
  SensorCoordinates cartesianToSensor(const Point3D& C_point) const final;
  Point3D sensorToCartesian(const SensorCoordinates& coordinates) const final;
  FloatingPoint imageOffsetToErrorNorm(
      const ImageCoordinates& /*linearization_point*/,
      const Vector2D& offset) const final;
  std::array<FloatingPoint, 4> imageOffsetsToErrorNorms(
      const ImageCoordinates& /*linearization_point*/,
      const CellToBeamOffsetArray& offsets) const final;

  // Projection from Cartesian space onto the sensor's image surface
  ImageCoordinates cartesianToImage(const Point3D& C_point) const final {
    return cartesianToSensor(C_point).image;
  }
  FloatingPoint cartesianToSensorZ(const Point3D& C_point) const final {
    return C_point.z();
  }

  AABB<Vector3D> cartesianToSensorAABB(
      const AABB<Point3D>& W_aabb,
      const Transformation3D::RotationMatrix& R_C_W,
      const Point3D& t_W_C) const final;

 private:
  const PinholeCameraProjectorConfig config_;

  const FloatingPoint fxfy_ = config_.fx * config_.fy;
  const FloatingPoint fxfy_inv_ = 1.f / fxfy_;
  const FloatingPoint cxfy_ = config_.cx * config_.fy;
  const FloatingPoint cyfx_ = config_.cy * config_.fx;
};
}  // namespace wavemap

#include "wavemap/integrator/projection_model/impl/pinhole_camera_projector_inl.h"

#endif  // WAVEMAP_INTEGRATOR_PROJECTION_MODEL_PINHOLE_CAMERA_PROJECTOR_H_
