#ifndef WAVEMAP_INTEGRATOR_PROJECTION_MODEL_IMPL_PINHOLE_CAMERA_PROJECTOR_INL_H_
#define WAVEMAP_INTEGRATOR_PROJECTION_MODEL_IMPL_PINHOLE_CAMERA_PROJECTOR_INL_H_

#include <algorithm>

namespace wavemap {
inline SensorCoordinates PinholeCameraProjector::cartesianToSensor(
    const Point3D& C_point) const {
  const FloatingPoint w_clamped = std::max(C_point.z(), kEpsilon);
  const FloatingPoint uw = config_.fx * C_point.x() / w_clamped + config_.cx;
  const FloatingPoint vw = config_.fy * C_point.y() / w_clamped + config_.cy;
  return {{uw, vw}, C_point.z()};
}

inline Point3D PinholeCameraProjector::sensorToCartesian(
    const SensorCoordinates& coordinates) const {
  const FloatingPoint u_scaled = coordinates.image[0];
  const FloatingPoint v_scaled = coordinates.image[1];
  const FloatingPoint w = coordinates.normal;
  Point3D C_point = w * fxfy_inv_ *
                    Point3D{config_.fy * u_scaled - cxfy_,
                            config_.fx * v_scaled - cyfx_, fxfy_};
  return C_point;
}

inline FloatingPoint PinholeCameraProjector::imageOffsetToErrorNorm(
    const ImageCoordinates& /*linearization_point*/,
    const Vector2D& offset) const {
  return offset.norm();
}

inline std::array<FloatingPoint, 4>
PinholeCameraProjector::imageOffsetsToErrorNorms(
    const ImageCoordinates& /*linearization_point*/,
    const ProjectorBase::CellToBeamOffsetArray& offsets) const {
  std::array<FloatingPoint, 4> error_norms{};
  for (int offset_idx = 0; offset_idx < 4; ++offset_idx) {
    error_norms[offset_idx] =
        std::sqrt(offsets[offset_idx][0] * offsets[offset_idx][0] +
                  offsets[offset_idx][1] * offsets[offset_idx][1]);
  }
  return error_norms;
}
}  // namespace wavemap

#endif  // WAVEMAP_INTEGRATOR_PROJECTION_MODEL_IMPL_PINHOLE_CAMERA_PROJECTOR_INL_H_
