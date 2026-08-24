#include <algorithm>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <boost/bind/bind.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <wavemap_msgs/LayeredMapUpdate.h>
#include <wavemap_ros/utils/tf_transformer.h>
#include <wavemap_ros_conversions/pointcloud_layer_observation_conversions.h>
#include <wavemap_ros_conversions/rgbd_layer_observation_conversions.h>

namespace wavemap {
class FieldLayerSensorBridge {
 public:
  FieldLayerSensorBridge()
      : nh_private_("~"),
        transformer_(std::make_shared<TfTransformer>()) {
    nh_private_.param("world_frame", world_frame_, std::string("odom"));
    nh_private_.param("reflectivity_topic", reflectivity_topic_,
                      std::string("/ouster/points"));
    nh_private_.param("color_topic", color_topic_,
                      std::string("/camera/color/image_raw"));
    nh_private_.param("aligned_depth_topic", aligned_depth_topic_,
                      std::string(
                          "/camera/aligned_depth_to_color/image_raw"));
    nh_private_.param("camera_info_topic", camera_info_topic_,
                      std::string("/camera/color/camera_info"));
    nh_private_.param("reflectivity_enabled", reflectivity_enabled_, true);
    nh_private_.param("color_enabled", color_enabled_, false);

    int point_stride = 1;
    int pixel_stride = 4;
    int queue_size = 10;
    double sync_tolerance = 0.05;
    nh_private_.param("reflectivity_point_stride", point_stride, point_stride);
    nh_private_.param("color_pixel_stride", pixel_stride, pixel_stride);
    nh_private_.param("sync_queue_size", queue_size, queue_size);
    nh_private_.param("sync_tolerance", sync_tolerance, sync_tolerance);
    reflectivity_config_.point_stride =
        static_cast<size_t>(std::max(1, point_stride));
    color_config_.pixel_stride =
        static_cast<size_t>(std::max(1, pixel_stride));

    update_pub_ = nh_.advertise<wavemap_msgs::LayeredMapUpdate>(
        "/wavemap/layered_map_updates", 1);

    if (reflectivity_enabled_) {
      reflectivity_sub_ = nh_.subscribe(
          reflectivity_topic_, queue_size,
          &FieldLayerSensorBridge::reflectivityCallback, this);
    }
    if (color_enabled_) {
      color_sub_.subscribe(nh_, color_topic_, queue_size);
      depth_sub_.subscribe(nh_, aligned_depth_topic_, queue_size);
      camera_info_sub_.subscribe(nh_, camera_info_topic_, queue_size);
      SyncPolicy policy(queue_size);
      policy.setMaxIntervalDuration(ros::Duration(sync_tolerance));
      rgbd_sync_ = std::make_unique<Synchronizer>(
          static_cast<const SyncPolicy&>(policy), color_sub_, depth_sub_,
          camera_info_sub_);
      rgbd_sync_->registerCallback(boost::bind(
          &FieldLayerSensorBridge::rgbdCallback, this,
          boost::placeholders::_1, boost::placeholders::_2,
          boost::placeholders::_3));
    }
  }

 private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
      sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::CameraInfo>;
  using Synchronizer = message_filters::Synchronizer<SyncPolicy>;

  template <typename ValueT>
  bool transformObservations(
      std::vector<layered::LayerObservation<ValueT>>& observations,
      const std::string& sensor_frame, const ros::Time& stamp) {
    const auto T_W_S =
        transformer_->lookupTransform(world_frame_, sensor_frame, stamp);
    if (!T_W_S) {
      ROS_WARN_STREAM_THROTTLE(
          2.0, "No transform from '" << sensor_frame << "' to '"
                                    << world_frame_
                                    << "'; skipping layer observations.");
      return false;
    }
    for (auto& observation : observations) {
      observation.position = *T_W_S * observation.position;
    }
    return true;
  }

  void reflectivityCallback(const sensor_msgs::PointCloud2& cloud) {
    auto batch = convert::pointcloudToReflectivityObservations(
        cloud, reflectivity_config_);
    if (!batch) {
      ROS_WARN_STREAM_THROTTLE(
          2.0, "Reflectivity conversion failed: " << batch.error);
      return;
    }
    if (!transformObservations(
            batch.observations, batch.header.frame_id, batch.header.stamp)) {
      return;
    }

    wavemap_msgs::LayeredMapUpdate update;
    update.header = batch.header;
    update.header.frame_id = world_frame_;
    update.positions.reserve(batch.observations.size());
    wavemap_msgs::Layer layer;
    layer.name = "reflectivity";
    layer.type = "float32";
    layer.float32_values.reserve(batch.observations.size());
    for (const auto& observation : batch.observations) {
      geometry_msgs::Point32 position;
      position.x = observation.position.x();
      position.y = observation.position.y();
      position.z = observation.position.z();
      update.positions.emplace_back(position);
      layer.float32_values.emplace_back(observation.value);
    }
    update.layers.emplace_back(std::move(layer));
    update_pub_.publish(update);
  }

  void rgbdCallback(
      const sensor_msgs::ImageConstPtr& color,
      const sensor_msgs::ImageConstPtr& depth,
      const sensor_msgs::CameraInfoConstPtr& camera_info) {
    auto batch = convert::rgbdToColorObservations(
        *color, *depth, *camera_info, color_config_);
    if (!batch) {
      ROS_WARN_STREAM_THROTTLE(2.0, "RGB-D conversion failed: " << batch.error);
      return;
    }
    if (!transformObservations(
            batch.observations, batch.header.frame_id, batch.header.stamp)) {
      return;
    }

    wavemap_msgs::LayeredMapUpdate update;
    update.header = batch.header;
    update.header.frame_id = world_frame_;
    update.positions.reserve(batch.observations.size());
    wavemap_msgs::Layer layer;
    layer.name = "color";
    layer.type = "float32_rgb";
    layer.float32_values.reserve(3u * batch.observations.size());
    for (const auto& observation : batch.observations) {
      geometry_msgs::Point32 position;
      position.x = observation.position.x();
      position.y = observation.position.y();
      position.z = observation.position.z();
      update.positions.emplace_back(position);
      layer.float32_values.emplace_back(observation.value.r);
      layer.float32_values.emplace_back(observation.value.g);
      layer.float32_values.emplace_back(observation.value.b);
    }
    update.layers.emplace_back(std::move(layer));
    update_pub_.publish(update);
  }

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;
  std::shared_ptr<TfTransformer> transformer_;
  std::string world_frame_;
  std::string reflectivity_topic_;
  std::string color_topic_;
  std::string aligned_depth_topic_;
  std::string camera_info_topic_;
  bool reflectivity_enabled_ = true;
  bool color_enabled_ = false;
  convert::ReflectivityObservationConfig reflectivity_config_;
  convert::RgbdColorObservationConfig color_config_;

  ros::Publisher update_pub_;
  ros::Subscriber reflectivity_sub_;
  message_filters::Subscriber<sensor_msgs::Image> color_sub_;
  message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> camera_info_sub_;
  std::unique_ptr<Synchronizer> rgbd_sync_;
};
}  // namespace wavemap

int main(int argc, char** argv) {
  ros::init(argc, argv, "field_layer_sensor_bridge");
  wavemap::FieldLayerSensorBridge bridge;
  ros::spin();
  return 0;
}
