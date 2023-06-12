#ifndef WAVEMAP_ROS_INPUT_HANDLER_LIVOX_INPUT_HANDLER_H_
#define WAVEMAP_ROS_INPUT_HANDLER_LIVOX_INPUT_HANDLER_H_
#ifdef LIVOX_AVAILABLE

#include <memory>
#include <queue>
#include <string>

#include <livox_ros_driver2/CustomMsg.h>

#include "wavemap_ros/input_handler/input_handler.h"
#include "wavemap_ros/input_handler/pointcloud_undistorter.h"

namespace wavemap {
class LivoxInputHandler : public InputHandler {
 public:
  LivoxInputHandler(const InputHandlerConfig& config, const param::Map& params,
                    std::string world_frame,
                    VolumetricDataStructureBase::Ptr occupancy_map,
                    std::shared_ptr<TfTransformer> transformer,
                    ros::NodeHandle nh, ros::NodeHandle nh_private);

  InputHandlerType getType() const override { return InputHandlerType::kLivox; }

  void pointcloudCallback(const livox_ros_driver2::CustomMsg& pointcloud_msg);

 private:
  ros::Subscriber pointcloud_sub_;
  PointcloudUndistorter pointcloud_undistorter_;
  std::queue<StampedPointcloud> pointcloud_queue_;
  void processQueue() override;
};
}  // namespace wavemap

#endif
#endif  // WAVEMAP_ROS_INPUT_HANDLER_LIVOX_INPUT_HANDLER_H_
