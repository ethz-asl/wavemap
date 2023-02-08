#ifndef WAVEMAP_COMMON_INTEGRATOR_PROJECTION_MODEL_IMAGE_2D_SPHERICAL_PROJECTOR_H_
#define WAVEMAP_COMMON_INTEGRATOR_PROJECTION_MODEL_IMAGE_2D_SPHERICAL_PROJECTOR_H_

#include <algorithm>
#include <utility>

#include "wavemap_common/integrator/projection_model/image_1d/circular_projector.h"
#include "wavemap_common/integrator/projection_model/image_2d/image_2d_projection_model.h"
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

class SphericalProjector : public Image2DProjectionModel {
 public:
  using Config = SphericalProjectorConfig;

  explicit SphericalProjector(const Config& config)
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
    const ImageCoordinates image_coordinates = cartesianToImage(C_point);
    const FloatingPoint range = C_point.norm();
    return {image_coordinates.x(), image_coordinates.y(), range};
  }
  Point3D sensorToCartesian(const Vector3D& coordinates) const final {
    const FloatingPoint elevation_angle = coordinates[0];
    const FloatingPoint azimuth_angle = coordinates[1];
    const FloatingPoint range = coordinates[2];
    const Vector3D bearing{std::cos(elevation_angle) * std::cos(azimuth_angle),
                           std::cos(elevation_angle) * std::sin(azimuth_angle),
                           std::sin(elevation_angle)};
    return range * bearing;
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
    const FloatingPoint elevation_angle =
        std::atan2(C_point.z(), C_point.head<2>().norm());
    const FloatingPoint azimuth_angle = std::atan2(C_point.y(), C_point.x());
    return {elevation_angle, azimuth_angle};
  }
  FloatingPoint cartesianToSensorZ(
      const wavemap::Point3D& C_point) const final {
    return C_point.norm();
  }

  // NOTE: When the AABB is right behind the sensor, the angle range will wrap
  //       around at +-PI and a min_angle >= max_angle will be returned.
  AABB<Vector3D> cartesianToSensorAABB(
      const AABB<wavemap::Point3D>& W_aabb,
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
  const Config config_;

  ImageCoordinates cartesianToImageApprox(const Point3D& C_point) const {
    const FloatingPoint elevation_angle =
        approximate::atan2()(C_point.z(), C_point.head<2>().norm());
    const FloatingPoint azimuth_angle =
        approximate::atan2()(C_point.y(), C_point.x());
    return {elevation_angle, azimuth_angle};
  }
};
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_INTEGRATOR_PROJECTION_MODEL_IMAGE_2D_SPHERICAL_PROJECTOR_H_
