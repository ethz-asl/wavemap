#ifndef WAVEMAP_ROS_INPUT_HANDLER_POINTCLOUD_INPUT_HANDLER_H_
#define WAVEMAP_ROS_INPUT_HANDLER_POINTCLOUD_INPUT_HANDLER_H_

#include <memory>
#include <queue>
#include <string>

#include <sensor_msgs/PointCloud2.h>

#include "wavemap_ros/input_handler/input_handler.h"
#include "wavemap_ros/input_handler/pointcloud_undistorter.h"

#ifdef LIVOX_AVAILABLE
#include <livox_ros_driver2/CustomMsg.h>
#endif

namespace wavemap {
struct PointcloudType : public TypeSelector<PointcloudType> {
  using TypeSelector<PointcloudType>::TypeSelector;

  enum Id : TypeId { kPointCloud2, kLivox };

  static constexpr std::array names = {"PointCloud2", "Livox"};
};

struct PointcloudInputHandlerConfig
    : public ConfigBase<PointcloudInputHandlerConfig, 10, PointcloudType> {
  std::string topic_name = "scan";
  int topic_queue_length = 10;

  // TODO(victorr): Add support for Ouster
  PointcloudType pointcloud_type = PointcloudType::kPointCloud2;

  FloatingPoint processing_retry_period = 0.05f;
  FloatingPoint max_wait_for_pose = 1.f;

  std::string sensor_frame_id;  // Leave blank to use frame_id from msg header
  FloatingPoint time_offset = 0.f;  // TODO(victorr): Implement this option
  bool undistort_motion = true;     // TODO(victorr): Implement this option

  std::string reprojected_pointcloud_topic_name;  // Leave blank to disable
  std::string projected_range_image_topic_name;   // Leave blank to disable

  static MemberMap memberMap;

  // Conversion to InputHandler base config
  operator InputHandlerConfig() const {  // NOLINT
    return {topic_name, topic_queue_length, processing_retry_period,
            reprojected_pointcloud_topic_name,
            projected_range_image_topic_name};
  }

  bool isValid(bool verbose) const override;
};

class PointcloudInputHandler : public InputHandler {
 public:
  PointcloudInputHandler(const PointcloudInputHandlerConfig& config,
                         const param::Map& params, std::string world_frame,
                         VolumetricDataStructureBase::Ptr occupancy_map,
                         std::shared_ptr<TfTransformer> transformer,
                         ros::NodeHandle nh, ros::NodeHandle nh_private);

  InputHandlerType getType() const override {
    return InputHandlerType::kPointcloud;
  }

  void pointcloudCallback(const sensor_msgs::PointCloud2& pointcloud_msg);
#ifdef LIVOX_AVAILABLE
  void pointcloudCallback(const livox_ros_driver2::CustomMsg& pointcloud_msg);
#endif

 private:
  const PointcloudInputHandlerConfig config_;

  PointcloudUndistorter pointcloud_undistorter_;
  ros::Subscriber pointcloud_sub_;
  std::queue<StampedPointcloud> pointcloud_queue_;
  void processQueue() override;
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_INPUT_HANDLER_POINTCLOUD_INPUT_HANDLER_H_
