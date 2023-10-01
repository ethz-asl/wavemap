#ifndef WAVEMAP_INTEGRATOR_PROJECTION_MODEL_IMPL_SPHERICAL_PROJECTOR_INL_H_
#define WAVEMAP_INTEGRATOR_PROJECTION_MODEL_IMPL_SPHERICAL_PROJECTOR_INL_H_

#include <utility>

namespace wavemap {
inline SensorCoordinates SphericalProjector::cartesianToSensor(
    const Point3D& C_point) const {
  ImageCoordinates image_coordinates = cartesianToImage(C_point);
  const FloatingPoint range = C_point.norm();
  return {std::move(image_coordinates), range};
}

inline Point3D SphericalProjector::sensorToCartesian(
    const SensorCoordinates& coordinates) const {
  const FloatingPoint elevation_angle = coordinates.image[0];
  const FloatingPoint azimuth_angle = coordinates.image[1];
  const FloatingPoint range = coordinates.normal;
  const Vector3D bearing{std::cos(elevation_angle) * std::cos(azimuth_angle),
                         std::cos(elevation_angle) * std::sin(azimuth_angle),
                         std::sin(elevation_angle)};
  return range * bearing;
}

inline FloatingPoint SphericalProjector::imageOffsetToErrorSquaredNorm(
    const ImageCoordinates& linearization_point, const Vector2D& offset) const {
  // Scale the azimuth offset by the cosine of the elevation angle to account
  // for the change in density along the azimuth axis in function of elevation
  const FloatingPoint cos_elevation_angle = std::cos(linearization_point[0]);
  return offset[0] * offset[0] +
         (cos_elevation_angle * cos_elevation_angle) * (offset[1] * offset[1]);
}

inline std::array<FloatingPoint, 4>
SphericalProjector::imageOffsetsToErrorSquaredNorms(
    const ImageCoordinates& linearization_point,
    const CellToBeamOffsetArray& offsets) const {
  const FloatingPoint cos_elevation_angle = std::cos(linearization_point[0]);
  const FloatingPoint cos_elevation_angle_sq =
      cos_elevation_angle * cos_elevation_angle;
  std::array<FloatingPoint, 4> error_norms{};
  for (int offset_idx = 0; offset_idx < 4; ++offset_idx) {
    error_norms[offset_idx] =
        (offsets[offset_idx][0] * offsets[offset_idx][0]) +
        cos_elevation_angle_sq *
            (offsets[offset_idx][1] * offsets[offset_idx][1]);
  }
  return error_norms;
}

inline ImageCoordinates SphericalProjector::cartesianToImage(
    const Point3D& C_point) const {
  const FloatingPoint elevation_angle =
      approximate::atan2()(C_point.z(), C_point.head<2>().norm());
  const FloatingPoint azimuth_angle =
      approximate::atan2()(C_point.y(), C_point.x());
  return {elevation_angle, azimuth_angle};
}

inline FloatingPoint SphericalProjector::cartesianToSensorZ(
    const Point3D& C_point) const {
  return C_point.norm();
}
}  // namespace wavemap

#endif  // WAVEMAP_INTEGRATOR_PROJECTION_MODEL_IMPL_SPHERICAL_PROJECTOR_INL_H_
