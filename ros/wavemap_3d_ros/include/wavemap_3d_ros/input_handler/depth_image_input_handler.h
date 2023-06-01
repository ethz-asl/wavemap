#ifndef WAVEMAP_3D_ROS_INPUT_HANDLER_DEPTH_IMAGE_INPUT_HANDLER_H_
#define WAVEMAP_3D_ROS_INPUT_HANDLER_DEPTH_IMAGE_INPUT_HANDLER_H_

#include <memory>
#include <queue>
#include <string>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <wavemap_3d/integrator/projective/scanwise_integrator_3d.h>

#include "wavemap_3d_ros/input_handler/input_handler.h"

namespace wavemap {
class DepthImageInputHandler : public InputHandler {
 public:
  DepthImageInputHandler(const Config& config, const param::Map& params,
                         std::string world_frame,
                         VolumetricDataStructure3D::Ptr occupancy_map,
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
  ScanwiseIntegrator3D::Ptr scanwise_integrator_;

  image_transport::Subscriber depth_image_sub_;
  std::queue<sensor_msgs::Image> depth_image_queue_;
  void processQueue() override;

  PosedPointcloud<Point3D> reproject(
      const PosedRangeImage2D& posed_range_image);
};
}  // namespace wavemap

#endif  // WAVEMAP_3D_ROS_INPUT_HANDLER_DEPTH_IMAGE_INPUT_HANDLER_H_
