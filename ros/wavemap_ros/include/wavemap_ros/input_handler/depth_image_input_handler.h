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
struct DepthImageInputHandlerConfig
    : public ConfigBase<DepthImageInputHandlerConfig, 10> {
  std::string topic_name = "scan";
  int topic_queue_length = 10;

  FloatingPoint processing_retry_period = 0.05f;
  FloatingPoint max_wait_for_pose = 1.f;

  std::string sensor_frame_id;  // Leave blank to use frame_id from msg header
  std::string image_transport_hints = "raw";
  FloatingPoint depth_scale_factor = 1.f;
  FloatingPoint time_offset = 0.f;

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

class DepthImageInputHandler : public InputHandler {
 public:
  DepthImageInputHandler(const DepthImageInputHandlerConfig& config,
                         const param::Map& params, std::string world_frame,
                         VolumetricDataStructureBase::Ptr occupancy_map,
                         std::shared_ptr<TfTransformer> transformer,
                         ros::NodeHandle nh, ros::NodeHandle nh_private);

  InputHandlerType getType() const override {
    return InputHandlerType::kDepthImage;
  }

  void callback(const sensor_msgs::ImageConstPtr& depth_image_msg) {
    callback(*depth_image_msg);
  }
  void callback(const sensor_msgs::Image& depth_image_msg) {
    depth_image_queue_.emplace(depth_image_msg);
  }

 private:
  const DepthImageInputHandlerConfig config_;
  std::vector<ProjectiveIntegrator::Ptr> scanwise_integrators_;

  image_transport::Subscriber depth_image_sub_;
  std::queue<sensor_msgs::Image> depth_image_queue_;
  void processQueue() override;

  PosedPointcloud<> reproject(const PosedImage<>& posed_range_image);
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_INPUT_HANDLER_DEPTH_IMAGE_INPUT_HANDLER_H_
