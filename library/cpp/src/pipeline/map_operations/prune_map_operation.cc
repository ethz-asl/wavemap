#include "wavemap/pipeline/map_operations/prune_map_operation.h"

namespace wavemap {
DECLARE_CONFIG_MEMBERS(PruneMapOperationConfig,
                      (once_every));

bool PruneMapOperationConfig::isValid(bool verbose) const {
  return IS_PARAM_GT(once_every, 0.f, verbose);
}

bool PruneMapOperation::shouldRun(const Timestamp& current_time) {
  return config_.once_every <
         time::to_seconds<FloatingPoint>(current_time - last_run_timestamp_);
}

void PruneMapOperation::run(bool force_run) {
  const Timestamp current_time = Time::now();
  if (force_run || shouldRun(current_time)) {
    occupancy_map_->pruneSmart();
    last_run_timestamp_ = current_time;
  }
}
}  // namespace wavemap
