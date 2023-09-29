#ifndef WAVEMAP_INTEGRATOR_PROJECTION_MODEL_IMPL_SPHERICAL_PROJECTOR_INL_H_
#define WAVEMAP_INTEGRATOR_PROJECTION_MODEL_IMPL_SPHERICAL_PROJECTOR_INL_H_

namespace wavemap {
inline Vector3D SphericalProjector::cartesianToSensor(
    const Point3D& C_point) const {
  const Vector2D image_coordinates = cartesianToImage(C_point);
  const FloatingPoint range = C_point.norm();
  return {image_coordinates.x(), image_coordinates.y(), range};
}

inline Point3D SphericalProjector::sensorToCartesian(
    const Vector3D& coordinates) const {
  const FloatingPoint elevation_angle = coordinates[0];
  const FloatingPoint azimuth_angle = coordinates[1];
  const FloatingPoint range = coordinates[2];
  const Vector3D bearing{std::cos(elevation_angle) * std::cos(azimuth_angle),
                         std::cos(elevation_angle) * std::sin(azimuth_angle),
                         std::sin(elevation_angle)};
  return range * bearing;
}

inline FloatingPoint SphericalProjector::imageOffsetToErrorNorm(
    const Vector2D& linearization_point, const Vector2D& offset) const {
  // Scale the azimuth offset by the cosine of the elevation angle to account
  // for the change in density along the azimuth axis in function of elevation
  const FloatingPoint cos_elevation_angle = std::cos(linearization_point[0]);
  return std::sqrt(offset[0] * offset[0] +
                   (cos_elevation_angle * cos_elevation_angle) *
                       (offset[1] * offset[1]));
}

inline std::array<FloatingPoint, 4>
SphericalProjector::imageOffsetsToErrorNorms(
    const Vector2D& linearization_point,
    const ProjectorBase::CellToBeamOffsetArray& offsets) const {
  const FloatingPoint cos_elevation_angle = std::cos(linearization_point[0]);
  std::array<FloatingPoint, 4> error_norms{};
  for (int offset_idx = 0; offset_idx < 4; ++offset_idx) {
    error_norms[offset_idx] =
        std::sqrt((offsets[offset_idx][0] * offsets[offset_idx][0]) +
                  (cos_elevation_angle * cos_elevation_angle) *
                      (offsets[offset_idx][1] * offsets[offset_idx][1]));
  }
  return error_norms;
}

inline Vector2D SphericalProjector::cartesianToImage(
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

inline Vector2D SphericalProjector::cartesianToImageApprox(
    const Point3D& C_point) const {
  const FloatingPoint elevation_angle =
      approximate::atan2()(C_point.z(), C_point.head<2>().norm());
  const FloatingPoint azimuth_angle =
      approximate::atan2()(C_point.y(), C_point.x());
  return {elevation_angle, azimuth_angle};
}
}  // namespace wavemap

#endif  // WAVEMAP_INTEGRATOR_PROJECTION_MODEL_IMPL_SPHERICAL_PROJECTOR_INL_H_
