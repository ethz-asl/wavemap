#ifndef WAVEMAP_INTEGRATOR_PROJECTION_MODEL_SPHERICAL_PROJECTOR_H_
#define WAVEMAP_INTEGRATOR_PROJECTION_MODEL_SPHERICAL_PROJECTOR_H_

#include <algorithm>
#include <utility>

#include "wavemap/config/config_base.h"
#include "wavemap/integrator/projection_model/circular_projector.h"
#include "wavemap/integrator/projection_model/projector_base.h"
#include "wavemap/utils/approximate_trigonometry.h"

namespace wavemap {
/**
 * Config struct for the spherical projection model.
 */
struct SphericalProjectorConfig
    : ConfigBase<SphericalProjectorConfig, 2, CircularProjectorConfig> {
  //! Properties of the projection model along the elevation axis.
  CircularProjectorConfig elevation;
  //! Properties of the projection model along the azimuth axis.
  CircularProjectorConfig azimuth;

  static MemberMap memberMap;

  // Constructors
  SphericalProjectorConfig() = default;
  SphericalProjectorConfig(CircularProjectorConfig elevation,
                           CircularProjectorConfig azimuth)
      : elevation(std::move(elevation)), azimuth(std::move(azimuth)) {}

  bool isValid(bool verbose) const override;
};

class SphericalProjector : public ProjectorBase {
 public:
  using Config = SphericalProjectorConfig;

  explicit SphericalProjector(const Config& config);

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
  const Config config_;
};
}  // namespace wavemap

#include "wavemap/integrator/projection_model/impl/spherical_projector_inl.h"

#endif  // WAVEMAP_INTEGRATOR_PROJECTION_MODEL_SPHERICAL_PROJECTOR_H_
