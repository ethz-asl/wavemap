#ifndef WAVEMAP_INTEGRATOR_IMPL_INTEGRATOR_BASE_INL_H_
#define WAVEMAP_INTEGRATOR_IMPL_INTEGRATOR_BASE_INL_H_

namespace wavemap {
inline bool IntegratorBase::isMeasurementValid(const Point3D& C_end_point) {
  // Reject points with NaN coordinates
  if (C_end_point.hasNaN()) {
    return false;
  }

  // Reject points with suspicious range values
  constexpr FloatingPoint kValidRangeLowerThresholdSquared = 1e-3f * 1e-3f;
  constexpr FloatingPoint kValidRangeUpperThresholdSquared = 1e3f * 1e3f;
  const FloatingPoint measured_distance_squared = C_end_point.squaredNorm();
  if (measured_distance_squared < kValidRangeLowerThresholdSquared) {
    return false;
  }
  if (kValidRangeUpperThresholdSquared < measured_distance_squared) {
    LOG(INFO) << "Skipping point with suspicious range value: "
              << std::sqrt(measured_distance_squared);
    return false;
  }

  return true;
}

inline Point3D IntegratorBase::getEndPointOrMaxRange(
    const Point3D& W_start_point, const Point3D& W_end_point,
    FloatingPoint measured_distance, FloatingPoint max_range) {
  if (max_range < measured_distance) {
    return W_start_point +
           max_range / measured_distance * (W_end_point - W_start_point);
  } else {
    return W_end_point;
  }
}
}  // namespace wavemap

#endif  // WAVEMAP_INTEGRATOR_IMPL_INTEGRATOR_BASE_INL_H_
