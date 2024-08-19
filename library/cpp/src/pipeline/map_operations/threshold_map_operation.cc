#include "wavemap/pipeline/map_operations/threshold_map_operation.h"

namespace wavemap {
DECLARE_CONFIG_MEMBERS(ThresholdMapOperationConfig,
                      (once_every));

bool ThresholdMapOperationConfig::isValid(bool verbose) const {
  return IS_PARAM_GT(once_every, 0.f, verbose);
}

bool ThresholdMapOperation::shouldRun(const Timestamp& current_time) {
  return config_.once_every <
         time::to_seconds<FloatingPoint>(current_time - last_run_timestamp_);
}

void ThresholdMapOperation::run(bool force_run) {
  const Timestamp current_time = Time::now();
  if (force_run || shouldRun(current_time)) {
    occupancy_map_->threshold();
    last_run_timestamp_ = current_time;
  }
}
}  // namespace wavemap
