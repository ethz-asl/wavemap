#include "wavemap_ros/operations/threshold_map_operation.h"

namespace wavemap {
DECLARE_CONFIG_MEMBERS(ThresholdMapOperationConfig,
                      (once_every));

bool ThresholdMapOperationConfig::isValid(bool verbose) const {
  return IS_PARAM_GT(once_every, 0.f, verbose);
}
}  // namespace wavemap
