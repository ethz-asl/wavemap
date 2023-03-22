#ifndef WAVEMAP_ROS_INPUT_HANDLER_DEPTH_IMAGE_INPUT_HANDLER_H_
#define WAVEMAP_ROS_INPUT_HANDLER_DEPTH_IMAGE_INPUT_HANDLER_H_

#include <memory>
#include <queue>
#include <string>
#include <vector>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <wavemap/data_structure/image.h>
#include <wavemap/integrator/projective/projective_integrator.h>

#include "wavemap_ros/input_handler/input_handler.h"

namespace wavemap {
class DepthImageInputHandler : public InputHandler {
 public:
  DepthImageInputHandler(const Config& config, const param::Map& params,
                         std::string world_frame,
                         VolumetricDataStructureBase::Ptr occupancy_map,
                         std::shared_ptr<TfTransformer> transformer,
                         ros::NodeHandle nh, ros::NodeHandle nh_private);

  InputHandlerType getType() const override {
    return InputHandlerType::kDepthImage;
  }

  void depthImageCallback(const sensor_msgs::ImageConstPtr& depth_image_msg) {
    depthImageCallback(*depth_image_msg);
  }
  void depthImageCallback(const sensor_msgs::Image& depth_image_msg) {
    depth_image_queue_.emplace(depth_image_msg);
  }

 private:
  std::vector<ProjectiveIntegrator::Ptr> scanwise_integrators_;

  image_transport::Subscriber depth_image_sub_;
  std::queue<sensor_msgs::Image> depth_image_queue_;
  void processQueue() override;

  PosedPointcloud<> reproject(const PosedImage<>& posed_range_image);
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_INPUT_HANDLER_DEPTH_IMAGE_INPUT_HANDLER_H_
