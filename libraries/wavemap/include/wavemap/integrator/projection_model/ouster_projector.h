#ifndef WAVEMAP_INTEGRATOR_PROJECTION_MODEL_OUSTER_PROJECTOR_H_
#define WAVEMAP_INTEGRATOR_PROJECTION_MODEL_OUSTER_PROJECTOR_H_

#include <algorithm>
#include <utility>

#include "wavemap/config/config_base.h"
#include "wavemap/integrator/projection_model/circular_projector.h"
#include "wavemap/integrator/projection_model/projector_base.h"
#include "wavemap/utils/approximate_trigonometry.h"

namespace wavemap {
/**
 * Config struct for the ouster LiDAR projection model.
 */
struct OusterProjectorConfig
    : ConfigBase<OusterProjectorConfig, 4, CircularProjectorConfig> {
  //! Properties of the projection model along the elevation axis.
  CircularProjectorConfig elevation;
  //! Properties of the projection model along the azimuth axis.
  CircularProjectorConfig azimuth;
  //! Offset between the Ouster LiDAR frame's origin and the laser beam's start
  //! point (radial direction). For illustrations and additional information,
  //! please see the Ouster sensor's manual.
  Meters<FloatingPoint> lidar_origin_to_beam_origin = 0.02767f;
  //! Offset between the Ouster sensor and LiDAR frame's origins (z-direction).
  //! For illustrations and additional information, please see the Ouster
  //! sensor's manual.
  Meters<FloatingPoint> lidar_origin_to_sensor_origin_z_offset = 0.03618f;

  static MemberMap memberMap;

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
};

class OusterProjector : public ProjectorBase {
 public:
  using Config = OusterProjectorConfig;

  explicit OusterProjector(const Config& config);

  Eigen::Matrix<bool, 3, 1> sensorAxisIsPeriodic() const final;
  Eigen::Matrix<bool, 3, 1> sensorAxisCouldBePeriodic() const final {
    return {true, true, false};
  }
  SiUnit getImageCoordinatesUnit() const final { return SiUnit::kRadians; }

  // Coordinate transforms between Cartesian and sensor space
  SensorCoordinates cartesianToSensor(const Point3D& C_point) const final;
  Point3D sensorToCartesian(const SensorCoordinates& coordinates) const final;
  FloatingPoint imageOffsetToErrorSquaredNorm(
      const ImageCoordinates& linearization_point,
      const Vector2D& offset) const final;
  std::array<FloatingPoint, 4> imageOffsetsToErrorSquaredNorms(
      const ImageCoordinates& linearization_point,
      const CellToBeamOffsetArray& offsets) const final;

  // Projection from Cartesian space onto the sensor's image surface
  ImageCoordinates cartesianToImage(const Point3D& C_point) const final;
  FloatingPoint cartesianToSensorZ(const Point3D& C_point) const final;

  // NOTE: When the AABB is right behind the sensor, the angle range will wrap
  //       around at +-PI and a min_angle >= max_angle will be returned.
  AABB<Vector3D> cartesianToSensorAABB(
      const AABB<Point3D>& W_aabb,
      const Transformation3D::RotationMatrix& R_C_W,
      const Point3D& t_W_C) const final;

 private:
  const OusterProjectorConfig config_;
};
}  // namespace wavemap

#include "wavemap/integrator/projection_model/impl/ouster_projector_inl.h"

#endif  // WAVEMAP_INTEGRATOR_PROJECTION_MODEL_OUSTER_PROJECTOR_H_
