#ifndef WAVEMAP_INTEGRATOR_PROJECTION_MODEL_IMPL_OUSTER_PROJECTOR_INL_H_
#define WAVEMAP_INTEGRATOR_PROJECTION_MODEL_IMPL_OUSTER_PROJECTOR_INL_H_

namespace wavemap {
inline Vector3D OusterProjector::cartesianToSensor(
    const Point3D& C_point) const {
  // Project the beam's endpoint into the 2D plane B whose origin lies at the
  // beam's start point, X-axis is parallel to the projection of the beam onto
  // frame C's XY-plane and Y-axis is parallel to frame C's Z-axis
  const Point2D B_point{
      C_point.head<2>().norm() - config_.lidar_origin_to_beam_origin,
      C_point.z() - config_.lidar_origin_to_sensor_origin_z_offset};
  const FloatingPoint elevation_angle =
      approximate::atan2()(B_point.y(), B_point.x());
  const FloatingPoint azimuth_angle =
      approximate::atan2()(C_point.y(), C_point.x());
  const FloatingPoint range = B_point.norm();
  return {elevation_angle, azimuth_angle, range};
}

inline Point3D OusterProjector::sensorToCartesian(
    const Vector3D& coordinates) const {
  const FloatingPoint elevation_angle = coordinates[0];
  const FloatingPoint azimuth_angle = coordinates[1];
  const FloatingPoint range = coordinates[2];
  const Point2D B_point =
      range * Vector2D(std::cos(elevation_angle), std::sin(elevation_angle)) +
      Vector2D(config_.lidar_origin_to_beam_origin,
               config_.lidar_origin_to_sensor_origin_z_offset);
  Point3D C_point{B_point.x() * std::cos(azimuth_angle),
                  B_point.x() * std::sin(azimuth_angle), B_point.y()};
  return C_point;
}

inline FloatingPoint OusterProjector::imageOffsetToErrorNorm(
    const Vector2D& linearization_point, Vector2D offset) const {
  // Scale the azimuth offset by the cosine of the elevation angle to account
  // for the change in density along the azimuth axis in function of elevation
  const FloatingPoint cos_elevation_angle = std::cos(linearization_point[0]);
  offset[1] *= cos_elevation_angle;
  return offset.norm();
}

inline std::array<FloatingPoint, 4> OusterProjector::imageOffsetsToErrorNorms(
    const Vector2D& linearization_point,
    ProjectorBase::CellToBeamOffsetArray offsets) const {
  const FloatingPoint cos_elevation_angle = std::cos(linearization_point[0]);
  for (int offset_idx = 0; offset_idx < 4; ++offset_idx) {
    offsets[offset_idx][1] *= cos_elevation_angle;
  }
  std::array<FloatingPoint, 4> error_norms{};
  for (int offset_idx = 0; offset_idx < 4; ++offset_idx) {
    error_norms[offset_idx] = offsets[offset_idx].norm();
  }
  return error_norms;
}

inline Vector2D OusterProjector::cartesianToImage(
    const Point3D& C_point) const {
  // Project the beam's endpoint into the 2D plane B whose origin lies at the
  // beam's start point, X-axis is parallel to the projection of the beam onto
  // frame C's XY-plane and Y-axis is parallel to frame C's Z-axis
  const Vector2D B_point{
      C_point.head<2>().norm() - config_.lidar_origin_to_beam_origin,
      C_point.z() - config_.lidar_origin_to_sensor_origin_z_offset};
  const FloatingPoint elevation_angle =
      approximate::atan2()(B_point.y(), B_point.x());
  const FloatingPoint azimuth_angle =
      approximate::atan2()(C_point.y(), C_point.x());
  return {elevation_angle, azimuth_angle};
}

inline FloatingPoint OusterProjector::cartesianToSensorZ(
    const Point3D& C_point) const {
  const Point2D B_point{
      C_point.head<2>().norm() - config_.lidar_origin_to_beam_origin,
      C_point.z() - config_.lidar_origin_to_sensor_origin_z_offset};
  return B_point.norm();
}

inline Vector2D OusterProjector::cartesianToImageApprox(
    const Point3D& C_point) const {
  const Vector2D B_point{
      C_point.head<2>().norm() - config_.lidar_origin_to_beam_origin,
      C_point.z() - config_.lidar_origin_to_sensor_origin_z_offset};
  const FloatingPoint elevation_angle =
      approximate::atan2()(B_point.y(), B_point.x());
  const FloatingPoint azimuth_angle =
      approximate::atan2()(C_point.y(), C_point.x());
  return {elevation_angle, azimuth_angle};
}
}  // namespace wavemap

#endif  // WAVEMAP_INTEGRATOR_PROJECTION_MODEL_IMPL_OUSTER_PROJECTOR_INL_H_
