#ifndef WAVEMAP_ROS_OPERATIONS_PUBLISH_POINTCLOUD_OPERATION_H_
#define WAVEMAP_ROS_OPERATIONS_PUBLISH_POINTCLOUD_OPERATION_H_

#include <memory>
#include <string>

#include <ros/ros.h>
#include <wavemap/config/config_base.h>
#include <wavemap/data_structure/volumetric/volumetric_data_structure_base.h>
#include <wavemap/utils/time/time.h>

#include "wavemap_ros/operations/operation_base.h"

namespace wavemap {
/**
 * Config struct for obstacle pointcloud publishing operations.
 */
struct PublishPointcloudOperationConfig
    : public ConfigBase<PublishPointcloudOperationConfig, 3> {
  //! Time period controlling how often the pointcloud is published.
  Seconds<FloatingPoint> once_every = 1.f;

  //! Threshold in log odds above which cells are classified as occupied.
  FloatingPoint occupancy_threshold_log_odds = 1e-3f;

  //! Name of the topic the pointcloud will be published on.
  std::string topic = "obstacle_pointcloud";

  static MemberMap memberMap;

  bool isValid(bool verbose) const override;
};

class PublishPointcloudOperation : public OperationBase {
 public:
  PublishPointcloudOperation(const PublishPointcloudOperationConfig& config,
                             std::string world_frame,
                             VolumetricDataStructureBase::Ptr occupancy_map,
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
  const VolumetricDataStructureBase::Ptr occupancy_map_;
  ros::Time last_run_timestamp_;

  // Pointcloud publishing
  ros::Publisher pointcloud_pub_;
  Timestamp last_run_timestamp_internal_;
  void publishPointcloud(const ros::Time& cell_index);
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_OPERATIONS_PUBLISH_POINTCLOUD_OPERATION_H_
