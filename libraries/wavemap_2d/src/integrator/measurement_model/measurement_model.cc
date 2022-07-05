#include "wavemap_2d/integrator/measurement_model/measurement_model.h"

namespace wavemap {
bool MeasurementModel::isMeasurementValid() const {
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
  if (!kUseClearing && exceedsMaxRange()) {
    return false;
  }
  return true;
}
}  // namespace wavemap
