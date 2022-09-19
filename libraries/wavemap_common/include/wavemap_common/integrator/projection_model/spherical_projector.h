#ifndef WAVEMAP_COMMON_INTEGRATOR_PROJECTION_MODEL_SPHERICAL_PROJECTOR_H_
#define WAVEMAP_COMMON_INTEGRATOR_PROJECTION_MODEL_SPHERICAL_PROJECTOR_H_

#include "wavemap_common/integrator/projection_model/circular_projector.h"

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

  Vector2D getMinAngles() const {
    return {getMinElevationAngle(), getMinAzimuthAngle()};
  }
  Vector2D getMaxAngles() const {
    return {getMaxElevationAngle(), getMaxAzimuthAngle()};
  }
  Index2D getDimensions() const { return {getNumRows(), getNumColumns()}; }

  static Vector2D bearingToSpherical(const Vector3D& bearing) {
    return {std::atan(bearing.z() / bearing.head<2>().norm()),
            std::atan2(bearing.y(), bearing.x())};
  }
  static Vector3D sphericalToBearing(Vector2D spherical_coordinates) {
    const FloatingPoint elevation_angle = spherical_coordinates[0];
    const FloatingPoint azimuth_angle = spherical_coordinates[1];
    Vector3D bearing;
    bearing.x() = std::cos(elevation_angle) * std::cos(azimuth_angle);
    bearing.y() = std::cos(elevation_angle) * std::sin(azimuth_angle);
    bearing.z() = std::sin(elevation_angle);
    return bearing;
  }

  Index2D sphericalToNearestIndex(Vector2D spherical_coordinates) const {
    return {elevation_projector_.angleToNearestIndex(spherical_coordinates[0]),
            azimuth_projector_.angleToNearestIndex(spherical_coordinates[1])};
  }
  Index2D sphericalToNearestIndex(Vector2D spherical_coordinates,
                                  Vector2D& spherical_remainders) const {
    return {elevation_projector_.angleToNearestIndex(spherical_coordinates[0],
                                                     spherical_remainders[0]),
            azimuth_projector_.angleToNearestIndex(spherical_coordinates[1],
                                                   spherical_remainders[1])};
  }
  Index2D sphericalToFloorIndex(Vector2D spherical_coordinates) const {
    return {elevation_projector_.angleToFloorIndex(spherical_coordinates[0]),
            azimuth_projector_.angleToFloorIndex(spherical_coordinates[1])};
  }
  Index2D sphericalToCeilIndex(Vector2D spherical_coordinates) const {
    return {elevation_projector_.angleToCeilIndex(spherical_coordinates[0]),
            azimuth_projector_.angleToCeilIndex(spherical_coordinates[1])};
  }
  Vector2D indexToSpherical(Index2D index) const {
    return {elevation_projector_.indexToAngle(index[0]),
            azimuth_projector_.indexToAngle(index[1])};
  }

  Index2D bearingToNearestIndex(const Vector3D& bearing) const {
    const Vector2D spherical_coordinates = bearingToSpherical(bearing);
    auto range_image_index = sphericalToNearestIndex(spherical_coordinates);
    return range_image_index;
  }
  Vector3D indexToBearing(const Index2D& index) const {
    const Vector2D spherical_coordinates = indexToSpherical(index);
    return sphericalToBearing(spherical_coordinates);
  }

 private:
  CircularProjector elevation_projector_;
  CircularProjector azimuth_projector_;
};
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_INTEGRATOR_PROJECTION_MODEL_SPHERICAL_PROJECTOR_H_
