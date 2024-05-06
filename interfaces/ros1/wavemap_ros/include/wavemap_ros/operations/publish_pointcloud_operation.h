#ifndef WAVEMAP_ROS_OPERATIONS_PUBLISH_POINTCLOUD_OPERATION_H_
#define WAVEMAP_ROS_OPERATIONS_PUBLISH_POINTCLOUD_OPERATION_H_

#include <memory>
#include <string>

#include <geometry_msgs/Point32.h>
#include <ros/ros.h>
#include <wavemap/core/config/config_base.h>
#include <wavemap/core/map/map_base.h>
#include <wavemap/core/utils/time/time.h>

#include "wavemap_ros/operations/operation_base.h"

namespace wavemap {
/**
 * Config struct for obstacle pointcloud publishing operations.
 */
struct PublishPointcloudOperationConfig
    : public ConfigBase<PublishPointcloudOperationConfig, 4> {
  //! Time period controlling how often the pointcloud is published.
  Seconds<FloatingPoint> once_every = 1.f;

  //! Threshold in log odds above which cells are classified as occupied.
  FloatingPoint occupancy_threshold_log_odds = 1e-3f;

  bool only_publish_changed_blocks = true;

  //! Name of the topic the pointcloud will be published on.
  std::string topic = "obstacle_pointcloud";

  static MemberMap memberMap;

  bool isValid(bool verbose) const override;
};

class PublishPointcloudOperation : public OperationBase {
 public:
  PublishPointcloudOperation(const PublishPointcloudOperationConfig& config,
                             std::string world_frame,
                             MapBase::Ptr occupancy_map,
                             ros::NodeHandle nh_private);

  OperationType getType() const override {
    return OperationType::kPublishPointcloud;
  }

  bool shouldRun(const ros::Time& current_time) {
    return config_.once_every < (current_time - last_run_timestamp_).toSec();
  }

  void run(const ros::Time& current_time, bool force_run) override {
    if (force_run || shouldRun(current_time)) {
      publishPointcloud(current_time);
      last_run_timestamp_ = current_time;
    }
  }

 private:
  const PublishPointcloudOperationConfig config_;
  const std::string world_frame_;
  const MapBase::Ptr occupancy_map_;
  ros::Time last_run_timestamp_;

  // Pointcloud publishing
  ros::Publisher pointcloud_pub_;
  Timestamp last_run_timestamp_internal_;
  void publishPointcloud(const ros::Time& current_time);
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_OPERATIONS_PUBLISH_POINTCLOUD_OPERATION_H_
