#ifndef WAVEMAP_COMMON_INTEGRATOR_MEASUREMENT_MODEL_IMPL_MEASUREMENT_MODEL_BASE_INL_H_
#define WAVEMAP_COMMON_INTEGRATOR_MEASUREMENT_MODEL_IMPL_MEASUREMENT_MODEL_BASE_INL_H_

namespace wavemap {
template <int dim>
bool MeasurementModelBase<dim>::isMeasurementValid() const {
  if (W_end_point_.hasNaN()) {
    LOG(WARNING) << "Skipping measurement whose endpoint contains NaNs:\n"
                 << W_end_point_;
    return false;
  }
  if (measured_distance_ < kEpsilon) {
    LOG(INFO) << "Skipping measurement with near-zero length: "
              << measured_distance_;
    return false;
  }
  if (1e3 < measured_distance_) {
    LOG(INFO) << "Skipping measurement with suspicious length: "
              << measured_distance_;
    return false;
  }
  return true;
}

template <int dim>
Point<dim> MeasurementModelBase<dim>::getEndPointOrMaxRange() const {
  if (kRangeMax < measured_distance_) {
    return W_start_point_ +
           kRangeMax / measured_distance_ * (W_end_point_ - W_start_point_);
  } else {
    return W_end_point_;
  }
}
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_INTEGRATOR_MEASUREMENT_MODEL_IMPL_MEASUREMENT_MODEL_BASE_INL_H_
