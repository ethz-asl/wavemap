#ifndef WAVEMAP_ROS_INPUTS_DEPTH_IMAGE_INPUT_H_
#define WAVEMAP_ROS_INPUTS_DEPTH_IMAGE_INPUT_H_

#include <memory>
#include <queue>
#include <string>
#include <vector>

#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <wavemap/data_structure/image.h>
#include <wavemap/integrator/projective/projective_integrator.h>

#include "wavemap_ros/inputs/input_base.h"

namespace wavemap {
/**
 * Config struct for the depth image input handler.
 */
struct DepthImageInputConfig : public ConfigBase<DepthImageInputConfig, 10> {
  //! Name of the ROS topic to subscribe to.
  std::string topic_name = "scan";
  //! Queue length to use when subscribing to the ROS topic.
  int topic_queue_length = 10;

  //! Time period used to control the rate at which to retry getting the sensor
  //! pose when ROS TF lookups fail.
  Seconds<FloatingPoint> processing_retry_period = 0.05f;
  //! Maximum amount of time to wait for the sensor pose to become available
  //! when ROS TF lookups fail.
  Seconds<FloatingPoint> max_wait_for_pose = 1.f;

  //! The frame_id to use to look up the sensor pose using ROS TFs.
  //! Note that setting this is optional, when left blank the header.frame_id of
  //! the measurement's msg is used.
  std::string sensor_frame_id;
  //! Custom image_transport::TransportHints to use when subscribing to the
  //! depth image topic. Defaults to 'raw'.
  //! For more info, see http://wiki.ros.org/image_transport.
  std::string image_transport_hints = "raw";
  //! Scale factor used to convert the depth image's values to meters.
  FloatingPoint depth_scale_factor = 1.f;
  //! Time offset to apply to the header.stamp of the measurement's msg when
  //! looking up its pose using ROS TFs. Can be used when the time offset is
  //! known (e.g. through calibration) but not corrected by the sensor's driver.
  Seconds<FloatingPoint> time_offset = 0.f;

  //! Name of the topic on which to republish the depth images as pointclouds.
  //! Useful to share the pointclouds with other ROS nodes and for debugging.
  //! Disabled if not set.
  std::string reprojected_pointcloud_topic_name;
  //! Name of the topic on which to republish the range image computed from
  //! the pointclouds. Useful for debugging. Disabled if not set.
  std::string projected_range_image_topic_name;

  static MemberMap memberMap;

  // Conversion to InputHandler base config
  operator InputBaseConfig() const {  // NOLINT
    return {topic_name, topic_queue_length, processing_retry_period,
            reprojected_pointcloud_topic_name,
            projected_range_image_topic_name};
  }

  bool isValid(bool verbose) const override;
};

class DepthImageInput : public InputBase {
 public:
  DepthImageInput(
      const DepthImageInputConfig& config, const param::Value& params,
      std::string world_frame, VolumetricDataStructureBase::Ptr occupancy_map,
      std::shared_ptr<TfTransformer> transformer,
      std::shared_ptr<ThreadPool> thread_pool, ros::NodeHandle nh,
      ros::NodeHandle nh_private,
      std::function<void(const ros::Time&)> map_update_callback = {});

  InputType getType() const override { return InputType::kDepthImage; }

  void callback(const sensor_msgs::ImageConstPtr& depth_image_msg) {
    callback(*depth_image_msg);
  }
  void callback(const sensor_msgs::Image& depth_image_msg) {
    depth_image_queue_.emplace(depth_image_msg);
  }

 private:
  const DepthImageInputConfig config_;
  std::vector<ProjectiveIntegrator::Ptr> scanwise_integrators_;

  image_transport::Subscriber depth_image_sub_;
  std::queue<sensor_msgs::Image> depth_image_queue_;
  void processQueue() override;

  PosedPointcloud<> reproject(const PosedImage<>& posed_range_image);
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_INPUTS_DEPTH_IMAGE_INPUT_H_
