#ifndef WAVEMAP_ROS_MAP_OPERATIONS_DECAY_MAP_OPERATION_H_
#define WAVEMAP_ROS_MAP_OPERATIONS_DECAY_MAP_OPERATION_H_

#include <utility>

#include <ros/ros.h>
#include <wavemap/core/config/config_base.h>
#include <wavemap/core/map/map_base.h>
#include <wavemap/core/utils/time/stopwatch.h>
#include <wavemap/pipeline/map_operations/map_operation_base.h>

namespace wavemap {
/**
 * Config struct for map decaying operations.
 */
struct DecayMapOperationConfig : public ConfigBase<DecayMapOperationConfig, 2> {
  //! Time period controlling how often the map is decayed.
  Seconds<FloatingPoint> once_every = 2.f;

  //! Decay rate in the range (0, 1), applied to each map value as
  //! `map_value *= decay_rate` at each operation run.
  FloatingPoint decay_rate = 0.9f;

  static MemberMap memberMap;

  bool isValid(bool verbose) const override;
};

class DecayMapOperation : public MapOperationBase {
 public:
  DecayMapOperation(const DecayMapOperationConfig& config,
                    MapBase::Ptr occupancy_map)
      : MapOperationBase(std::move(occupancy_map)),
        config_(config.checkValid()) {}

  bool shouldRun(const ros::Time& current_time = ros::Time::now());

  void run(bool force_run) override;

 private:
  const DecayMapOperationConfig config_;
  ros::Time last_run_timestamp_;
  Stopwatch timer_;
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_MAP_OPERATIONS_DECAY_MAP_OPERATION_H_
