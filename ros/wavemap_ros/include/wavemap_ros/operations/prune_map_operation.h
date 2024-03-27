#ifndef WAVEMAP_ROS_OPERATIONS_PRUNE_MAP_OPERATION_H_
#define WAVEMAP_ROS_OPERATIONS_PRUNE_MAP_OPERATION_H_

#include <utility>

#include <wavemap/config/config_base.h>
#include <wavemap/map/map_base.h>

#include "wavemap_ros/operations/operation_base.h"

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

class PruneMapOperation : public OperationBase {
 public:
  PruneMapOperation(const PruneMapOperationConfig& config,
                    MapBase::Ptr occupancy_map)
      : config_(config.checkValid()),
        occupancy_map_(std::move(occupancy_map)) {}

  OperationType getType() const override { return OperationType::kPruneMap; }

  bool shouldRun(const ros::Time& current_time) {
    return config_.once_every < (current_time - last_run_timestamp_).toSec();
  }

  void run(const ros::Time& current_time, bool force_run) override {
    if (force_run || shouldRun(current_time)) {
      occupancy_map_->pruneSmart();
      last_run_timestamp_ = current_time;
    }
  }

 private:
  const PruneMapOperationConfig config_;
  const MapBase::Ptr occupancy_map_;
  ros::Time last_run_timestamp_;
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_OPERATIONS_PRUNE_MAP_OPERATION_H_
