#ifndef WAVEMAP_COMMON_INTEGRATOR_SPHERICAL_PROJECTOR_H_
#define WAVEMAP_COMMON_INTEGRATOR_SPHERICAL_PROJECTOR_H_

#include "wavemap_common/integrator/circular_projector.h"

namespace wavemap {
class SphericalProjector {
 public:
  SphericalProjector(FloatingPoint min_elevation_angle,
                     FloatingPoint max_elevation_angle, IndexElement num_rows,
                     FloatingPoint min_azimuth_angle,
                     FloatingPoint max_azimuth_angle, IndexElement num_columns)
      : elevation_projector_(min_elevation_angle, max_elevation_angle,
                             num_rows),
        azimuth_projector_(min_azimuth_angle, max_azimuth_angle, num_columns) {}

  FloatingPoint getMinElevationAngle() const {
    return elevation_projector_.getMinAngle();
  }
  FloatingPoint getMaxElevationAngle() const {
    return elevation_projector_.getMaxAngle();
  }
  IndexElement getNumRows() const { return elevation_projector_.getNumCells(); }

  FloatingPoint getMinAzimuthAngle() const {
    return azimuth_projector_.getMinAngle();
  }
  FloatingPoint getMaxAzimuthAngle() const {
    return azimuth_projector_.getMaxAngle();
  }
  IndexElement getNumColumns() const {
    return azimuth_projector_.getNumCells();
  }

  static Vector2D bearingToSpherical(const Vector3D& bearing) {
    return {std::atan(bearing.z() / bearing.head<2>().norm()),
            std::atan2(bearing.y(), bearing.x())};
  }
  static Vector3D sphericalToBearing(Vector2D spherical_coordinates) {
    const FloatingPoint elevation_angle = spherical_coordinates.x();
    const FloatingPoint azimuth_angle = spherical_coordinates.y();
    Vector3D bearing;
    bearing.x() = std::cos(elevation_angle) * std::cos(azimuth_angle);
    bearing.y() = std::cos(elevation_angle) * std::sin(azimuth_angle);
    bearing.z() = std::sin(elevation_angle);
    return bearing;
  }

 private:
  CircularProjector elevation_projector_;
  CircularProjector azimuth_projector_;
};
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_INTEGRATOR_SPHERICAL_PROJECTOR_H_
