#include "wavemap_ros/input_handler/input_handler.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/eigen.hpp>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tracy/Tracy.hpp>
#include <wavemap/integrator/integrator_factory.h>
#include <wavemap/integrator/projective/projective_integrator.h>

namespace wavemap {
DECLARE_CONFIG_MEMBERS(InputHandlerConfig,
                      (topic_name)
                      (topic_queue_length)
                      (processing_retry_period)
                      (reprojected_pointcloud_topic_name)
                      (projected_range_image_topic_name));

bool InputHandlerConfig::isValid(bool verbose) const {
  bool all_valid = true;

  all_valid &= IS_PARAM_NE(topic_name, std::string(""), verbose);
  all_valid &= IS_PARAM_GT(topic_queue_length, 0, verbose);
  all_valid &= IS_PARAM_GT(processing_retry_period, 0.f, verbose);

  return all_valid;
}

InputHandler::InputHandler(const InputHandlerConfig& config,
                           const param::Value& params, std::string world_frame,
                           MapBase::Ptr occupancy_map,
                           std::shared_ptr<TfTransformer> transformer,
                           std::shared_ptr<ThreadPool> thread_pool,
                           const ros::NodeHandle& nh,
                           ros::NodeHandle nh_private)
    : config_(config.checkValid()),
      world_frame_(std::move(world_frame)),
      transformer_(std::move(transformer)) {
  // Create the integrators
  const auto integrators_param = params.getChild("integrators");
  if (!integrators_param) {
    ROS_WARN(
        "Could not find key named \"integrators\" in input handler "
        "params. Input handler will be disabled.");
    return;
  }
  const auto integrators_array = integrators_param->as<param::Array>();
  if (!integrators_array) {
    ROS_WARN(
        "Key named \"integrators\" in input handler params is not "
        "of type Array (list). Input handler will be disabled.");
    return;
  }
  for (const auto& integrator_params : integrators_array.value()) {
    auto integrator =
        IntegratorFactory::create(integrator_params, occupancy_map, thread_pool,
                                  IntegratorType::kRayTracingIntegrator);
    CHECK_NOTNULL(integrator);
    integrators_.emplace_back(std::move(integrator));
  }

  // Start the queue processing retry timer
  queue_processing_retry_timer_ =
      nh.createTimer(ros::Duration(config_.processing_retry_period),
                     [this](const auto& /*event*/) { processQueue(); });

  // Advertise the reprojected pointcloud publisher if enabled
  if (!config_.reprojected_pointcloud_topic_name.empty()) {
    reprojected_pointcloud_pub_ =
        nh_private.advertise<sensor_msgs::PointCloud2>(
            config_.reprojected_pointcloud_topic_name,
            config_.topic_queue_length);
  }
  if (!config_.projected_range_image_topic_name.empty()) {
    image_transport::ImageTransport it_private(nh_private);
    projected_range_image_pub_ = it_private.advertise(
        config_.projected_range_image_topic_name, config_.topic_queue_length);
  }
}

void InputHandler::publishReprojectedPointcloud(
    const ros::Time& stamp, const PosedPointcloud<>& posed_pointcloud) {
  ZoneScoped;
  sensor_msgs::PointCloud pointcloud_msg;
  pointcloud_msg.header.stamp = stamp;
  pointcloud_msg.header.frame_id = world_frame_;

  pointcloud_msg.points.reserve(posed_pointcloud.size());
  for (const auto& point : posed_pointcloud.getPointsGlobal()) {
    auto& point_msg = pointcloud_msg.points.emplace_back();
    point_msg.x = point.x();
    point_msg.y = point.y();
    point_msg.z = point.z();
  }

  sensor_msgs::PointCloud2 pointcloud2_msg;
  sensor_msgs::convertPointCloudToPointCloud2(pointcloud_msg, pointcloud2_msg);
  reprojected_pointcloud_pub_.publish(pointcloud2_msg);
}

void InputHandler::publishProjectedRangeImage(const ros::Time& stamp,
                                              const Image<>& range_image) {
  ZoneScoped;
  cv_bridge::CvImage cv_image;
  cv_image.header.stamp = stamp;
  cv_image.encoding = "32FC1";
  cv::eigen2cv(range_image.getData(), cv_image.image);
  projected_range_image_pub_.publish(cv_image.toImageMsg());
}
}  // namespace wavemap
