#ifndef WAVEMAP_ROS_OPERATIONS_THRESHOLD_MAP_OPERATION_H_
#define WAVEMAP_ROS_OPERATIONS_THRESHOLD_MAP_OPERATION_H_

#include <utility>

#include <wavemap/config/config_base.h>
#include <wavemap/data_structure/volumetric/volumetric_data_structure_base.h>

#include "wavemap_ros/operations/operation_base.h"

namespace wavemap {
/**
 * Config struct for map thresholding operations.
 */
struct ThresholdMapOperationConfig
    : public ConfigBase<ThresholdMapOperationConfig, 1> {
  //! Time period controlling how often the map is thresholded.
  Seconds<FloatingPoint> once_every = 2.f;

  static MemberMap memberMap;

  bool isValid(bool verbose) const override;
};

class ThresholdMapOperation : public OperationBase {
 public:
  ThresholdMapOperation(const ThresholdMapOperationConfig& config,
                        VolumetricDataStructureBase::Ptr occupancy_map)
      : config_(config.checkValid()),
        occupancy_map_(std::move(occupancy_map)) {}

  OperationType getType() const override {
    return OperationType::kThresholdMap;
  }

  bool shouldRun(const ros::Time& current_time) {
    return config_.once_every < (current_time - last_run_timestamp_).toSec();
  }

  void run(const ros::Time& current_time, bool force_run) override {
    if (force_run || shouldRun(current_time)) {
      occupancy_map_->threshold();
      last_run_timestamp_ = current_time;
    }
  }

 private:
  const ThresholdMapOperationConfig config_;
  const VolumetricDataStructureBase::Ptr occupancy_map_;
  ros::Time last_run_timestamp_;
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_OPERATIONS_THRESHOLD_MAP_OPERATION_H_
