#ifndef WAVEMAP_COMMON_INTEGRATOR_IMPL_POINTCLOUD_INTEGRATOR_INL_H_
#define WAVEMAP_COMMON_INTEGRATOR_IMPL_POINTCLOUD_INTEGRATOR_INL_H_

namespace wavemap {
template <int dim>
bool PointcloudIntegrator<dim>::isPointcloudValid(
    const PosedPointcloud<Point<dim>>& pointcloud) {
  if (const Point<dim>& origin = pointcloud.getOrigin(); origin.hasNaN()) {
    LOG(WARNING) << "Ignoring request to integrate pointcloud whose origin "
                    "contains NaNs:\n"
                 << origin;
    return false;
  }

  return true;
}

template <int dim>
bool PointcloudIntegrator<dim>::isMeasurementValid(
    const Point<dim>& C_end_point) {
  if (C_end_point.hasNaN()) {
    LOG(WARNING) << "Skipping measurement whose endpoint contains NaNs:\n"
                 << C_end_point;
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

template <int dim>
Point<dim> PointcloudIntegrator<dim>::getEndPointOrMaxRange(
    const Point<dim>& W_start_point, const Point<dim>& W_end_point,
    FloatingPoint measured_distance, FloatingPoint max_range) {
  if (max_range < measured_distance) {
    return W_start_point +
           max_range / measured_distance * (W_end_point - W_start_point);
  } else {
    return W_end_point;
  }
}
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_INTEGRATOR_IMPL_POINTCLOUD_INTEGRATOR_INL_H_
