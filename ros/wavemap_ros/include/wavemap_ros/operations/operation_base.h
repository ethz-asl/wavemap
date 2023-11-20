#ifndef WAVEMAP_ROS_OPERATIONS_OPERATION_BASE_H_
#define WAVEMAP_ROS_OPERATIONS_OPERATION_BASE_H_

#include <ros/ros.h>
#include <wavemap/config/type_selector.h>

namespace wavemap {
struct OperationType : public TypeSelector<OperationType> {
  using TypeSelector<OperationType>::TypeSelector;

  enum Id : TypeId { kThresholdMap, kPruneMap, kPublishMap };

  static constexpr std::array names = {"threshold_map", "prune_map",
                                       "publish_map"};
};

class OperationBase {
 public:
  OperationBase() = default;
  virtual ~OperationBase() = default;

  virtual OperationType getType() const = 0;

  virtual void run(const ros::Time& current_time, bool force_run) = 0;
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_OPERATIONS_OPERATION_BASE_H_
