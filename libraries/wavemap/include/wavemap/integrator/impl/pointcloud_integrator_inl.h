#ifndef WAVEMAP_INTEGRATOR_IMPL_POINTCLOUD_INTEGRATOR_INL_H_
#define WAVEMAP_INTEGRATOR_IMPL_POINTCLOUD_INTEGRATOR_INL_H_

namespace wavemap {
inline bool PointcloudIntegrator::isMeasurementValid(
    const Point3D& C_end_point) {
  if (C_end_point.hasNaN()) {
    return false;
  }
  const FloatingPoint measured_distance = C_end_point.norm();
  if (measured_distance < 1e-3f) {
    return false;
  }
  if (1e3 < measured_distance) {
    LOG(INFO) << "Skipping measurement with suspicious length: "
              << measured_distance;
    return false;
  }
  return true;
}

inline Point3D PointcloudIntegrator::getEndPointOrMaxRange(
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

#endif  // WAVEMAP_INTEGRATOR_IMPL_POINTCLOUD_INTEGRATOR_INL_H_
