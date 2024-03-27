#include "wavemap_ros/operations/prune_map_operation.h"

namespace wavemap {
DECLARE_CONFIG_MEMBERS(PruneMapOperationConfig,
                      (once_every));

bool PruneMapOperationConfig::isValid(bool verbose) const {
  return IS_PARAM_GT(once_every, 0.f, verbose);
}
}  // namespace wavemap
