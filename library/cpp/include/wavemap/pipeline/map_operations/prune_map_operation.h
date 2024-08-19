#ifndef WAVEMAP_PIPELINE_MAP_OPERATIONS_PRUNE_MAP_OPERATION_H_
#define WAVEMAP_PIPELINE_MAP_OPERATIONS_PRUNE_MAP_OPERATION_H_

#include <utility>

#include "wavemap/core/config/config_base.h"
#include "wavemap/core/map/map_base.h"
#include "wavemap/core/utils/time/time.h"
#include "wavemap/pipeline/map_operations/map_operation_base.h"

namespace wavemap {
/**
 * Config struct for map pruning operations.
 */
struct PruneMapOperationConfig : public ConfigBase<PruneMapOperationConfig, 1> {
  //! Time period controlling how often the map is pruned.
  Seconds<FloatingPoint> once_every = 10.f;

  static MemberMap memberMap;

  bool isValid(bool verbose) const override;
};

class PruneMapOperation : public MapOperationBase {
 public:
  PruneMapOperation(const PruneMapOperationConfig& config,
                    MapBase::Ptr occupancy_map)
      : MapOperationBase(std::move(occupancy_map)),
        config_(config.checkValid()) {}

  bool shouldRun(const Timestamp& current_time);

  void run(bool force_run) override;

 private:
  const PruneMapOperationConfig config_;
  Timestamp last_run_timestamp_;
};
}  // namespace wavemap

#endif  // WAVEMAP_PIPELINE_MAP_OPERATIONS_PRUNE_MAP_OPERATION_H_
