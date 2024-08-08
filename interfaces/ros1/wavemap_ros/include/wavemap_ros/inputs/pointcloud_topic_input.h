#ifndef WAVEMAP_ROS_INPUTS_POINTCLOUD_TOPIC_INPUT_H_
#define WAVEMAP_ROS_INPUTS_POINTCLOUD_TOPIC_INPUT_H_

#include <memory>
#include <queue>
#include <string>
#include <vector>

#include <image_transport/image_transport.h>
#include <sensor_msgs/PointCloud2.h>
#include <wavemap/core/config/string_list.h>
#include <wavemap/core/utils/time/stopwatch.h>

#include "wavemap_ros/inputs/ros_input_base.h"
#include "wavemap_ros/utils/pointcloud_undistorter.h"

#ifdef LIVOX_AVAILABLE
#include <livox_ros_driver2/CustomMsg.h>
#endif

namespace wavemap {
struct PointcloudTopicType : public TypeSelector<PointcloudTopicType> {
  using TypeSelector<PointcloudTopicType>::TypeSelector;

  enum Id : TypeId { kPointCloud2, kOuster, kLivox };

  static constexpr std::array names = {"PointCloud2", "ouster", "livox"};
};

/**
 * Config struct for the pointcloud input handler.
 */
struct PointcloudTopicInputConfig
    : public ConfigBase<PointcloudTopicInputConfig, 12, PointcloudTopicType,
                        StringList> {
  //! Name of the ROS topic to subscribe to.
  std::string topic_name;
  //! Message type of the ROS topic to subscribe to.
  PointcloudTopicType topic_type = PointcloudTopicType::kPointCloud2;
  //! Queue length to use when subscribing to the ROS topic.
  int topic_queue_length = 10;

  //! Name(s) of the measurement integrator(s) used to integrate the
  //! measurements into the map.
  StringList measurement_integrator_names;

  //! Time period used to control the rate at which to retry getting the sensor
  //! pose when ROS TF lookups fail.
  Seconds<FloatingPoint> processing_retry_period = 0.05f;
  //! Maximum amount of time to wait for the sensor pose to become available
  // when ROS TF lookups fail.
  Seconds<FloatingPoint> max_wait_for_pose = 1.f;

  //! The frame_id to use to look up the sensor pose using ROS TFs.
  //! Note that setting this is optional, when left blank the header.frame_id of
  //! the measurement's msg is used.
  std::string sensor_frame_id;
  //! Time offset to apply to the header.stamp of the measurement's msg when
  //! looking up its pose using ROS TFs. Can be used when the time offset is
  //! known (e.g. through calibration) but not corrected by the sensor's driver.
  Seconds<FloatingPoint> time_offset = 0.f;
  //! Whether to undistort each pointcloud based on the sensor's motion while it
  //! was captured. We strongly recommend turning this on, unless the robot's
  //! odometry is very jerky or the sensor's driver already performs motion
  //! undistortion.
  bool undistort_motion = false;
  //! Number of intermediate poses to sample from ROS TFs when performing motion
  //! undistortion.
  int num_undistortion_interpolation_intervals_per_cloud = 100;

  //! Name of the topic on which to publish the range image generated from the
  //! pointclouds. Useful for debugging, to see how well the projection model
  //! matches the LiDAR. Disabled if not set.
  std::string projected_range_image_topic_name;
  //! Name of the topic on which to republish the motion-undistorted
  //! pointclouds. Useful to share the undistorted pointclouds with other ROS
  //! nodes and for debugging. Disabled if not set.
  std::string undistorted_pointcloud_topic_name;

  static MemberMap memberMap;

  // Conversion to InputBase config
  operator RosInputBaseConfig() const {  // NOLINT
    return {topic_name, topic_queue_length, measurement_integrator_names,
            processing_retry_period};
  }

  bool isValid(bool verbose) const override;
};

class PointcloudTopicInput : public RosInputBase {
 public:
  PointcloudTopicInput(const PointcloudTopicInputConfig& config,
                       std::shared_ptr<Pipeline> pipeline,
                       std::shared_ptr<TfTransformer> transformer,
                       std::string world_frame, ros::NodeHandle nh,
                       ros::NodeHandle nh_private);

  RosInputType getType() const override {
    return RosInputType::kPointcloudTopic;
  }
  PointcloudTopicType getTopicType() const { return config_.topic_type; }

  void callback(const sensor_msgs::PointCloud2& pointcloud_msg);
#ifdef LIVOX_AVAILABLE
  void callback(const livox_ros_driver2::CustomMsg& pointcloud_msg);
#endif

  template <typename RegistrarT>
  static bool registerCallback(PointcloudTopicType type, RegistrarT registrar);

 private:
  const PointcloudTopicInputConfig config_;

  Stopwatch integration_timer_;

  PointcloudUndistorter pointcloud_undistorter_;

  ros::Subscriber pointcloud_sub_;
  std::queue<undistortion::StampedPointcloud> pointcloud_queue_;
  void processQueue() override;

  static bool hasField(const sensor_msgs::PointCloud2& msg,
                       const std::string& field_name);

  void publishProjectedRangeImageIfEnabled(
      const ros::Time& stamp, const PosedPointcloud<>& posed_pointcloud);
  image_transport::Publisher projected_range_image_pub_;

  void publishUndistortedPointcloudIfEnabled(
      const ros::Time& stamp, const PosedPointcloud<>& undistorted_pointcloud);
  ros::Publisher undistorted_pointcloud_pub_;
};
}  // namespace wavemap

#include "wavemap_ros/inputs/impl/pointcloud_topic_input_impl.h"

#endif  // WAVEMAP_ROS_INPUTS_POINTCLOUD_TOPIC_INPUT_H_
