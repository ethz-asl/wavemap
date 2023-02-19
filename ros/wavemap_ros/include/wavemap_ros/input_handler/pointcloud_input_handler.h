#ifndef WAVEMAP_ROS_INPUT_HANDLER_POINTCLOUD_INPUT_HANDLER_H_
#define WAVEMAP_ROS_INPUT_HANDLER_POINTCLOUD_INPUT_HANDLER_H_

#include <memory>
#include <queue>
#include <string>

#include <sensor_msgs/PointCloud2.h>

#include "wavemap_ros/input_handler/input_handler.h"

namespace wavemap {
class PointcloudInputHandler : public InputHandler {
 public:
  PointcloudInputHandler(const Config& config, const param::Map& params,
                         std::string world_frame,
                         VolumetricDataStructureBase::Ptr occupancy_map,
                         std::shared_ptr<TfTransformer> transformer,
                         ros::NodeHandle nh, ros::NodeHandle nh_private);

  InputHandlerType getType() const override {
    return InputHandlerType::kPointcloud;
  }

  void pointcloudCallback(const sensor_msgs::PointCloud2& pointcloud_msg) {
    pointcloud_queue_.emplace(pointcloud_msg);
  }

 private:
  ros::Subscriber pointcloud_sub_;
  std::queue<sensor_msgs::PointCloud2> pointcloud_queue_;
  void processQueue() override;
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_INPUT_HANDLER_POINTCLOUD_INPUT_HANDLER_H_
