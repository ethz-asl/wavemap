#include "wavemap_ros/input_handler/input_handler.h"

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <wavemap/integrator/projective/projective_integrator.h>

namespace wavemap {
InputHandler::InputHandler(const InputHandler::Config& config,
                           const param::Map& params, std::string world_frame,
                           VolumetricDataStructureBase::Ptr occupancy_map,
                           std::shared_ptr<TfTransformer> transformer,
                           const ros::NodeHandle& nh,
                           ros::NodeHandle nh_private)
    : config_(config.checkValid()),
      world_frame_(std::move(world_frame)),
      transformer_(std::move(transformer)) {
  // Create the integrator
  integrator_ = IntegratorFactory::create(
      params, std::move(occupancy_map), IntegratorType::kRayTracingIntegrator);
  CHECK_NOTNULL(integrator_);

  // Get the projection model (if the integrator uses one)
  auto scanwise_integrator =
      std::dynamic_pointer_cast<ProjectiveIntegrator>(integrator_);
  if (scanwise_integrator) {
    projection_model_ = scanwise_integrator->getProjectionModel();
  }

  // Start the queue processing retry timer
  queue_processing_retry_timer_ =
      nh.createTimer(ros::Duration(config_.processing_retry_period),
                     [this](const auto& /*event*/) { processQueue(); });

  // Advertise the reprojected pointcloud publisher if enabled
  if (!config_.reprojected_topic_name.empty()) {
    reprojection_pub_ = nh_private.advertise<sensor_msgs::PointCloud2>(
        config_.reprojected_topic_name, config_.topic_queue_length);
  }
}

void InputHandler::publishReprojected(
    const ros::Time& stamp, const PosedPointcloud<>& posed_pointcloud) {
  if (!isReprojectionEnabled()) {
    return;
  }

  sensor_msgs::PointCloud pointcloud_msg;
  pointcloud_msg.header.stamp = stamp;
  pointcloud_msg.header.frame_id = world_frame_;

  pointcloud_msg.points.reserve(posed_pointcloud.size());
  for (const auto& point : posed_pointcloud.getPointsGlobal()) {
    geometry_msgs::Point32 point_msg;
    point_msg.x = point.x();
    point_msg.y = point.y();
    point_msg.z = point.z();
    pointcloud_msg.points.emplace_back(point_msg);
  }

  sensor_msgs::PointCloud2 pointcloud2_msg;
  sensor_msgs::convertPointCloudToPointCloud2(pointcloud_msg, pointcloud2_msg);
  reprojection_pub_.publish(pointcloud2_msg);
}

InputHandler::Config InputHandler::Config::from(const param::Map& params) {
  Config config;

  config.topic_name = param::map::keyGetValue(params, NAMEOF(config.topic_name),
                                              config.topic_name);

  config.topic_queue_length = param::map::keyGetValue(
      params, NAMEOF(config.topic_queue_length), config.topic_queue_length);

  config.processing_retry_period =
      param::map::keyGetValue(params, NAMEOF(config.processing_retry_period),
                              config.processing_retry_period);

  config.max_wait_for_pose = param::convert::toSeconds(
      params, NAMEOF(config.max_wait_for_pose), config.max_wait_for_pose);

  config.sensor_frame_id = param::map::keyGetValue(
      params, NAMEOF(config.sensor_frame_id), config.sensor_frame_id);

  config.image_transport_hints =
      param::map::keyGetValue(params, NAMEOF(config.image_transport_hints),
                              config.image_transport_hints);
  config.depth_scale_factor = param::map::keyGetValue(
      params, NAMEOF(config.depth_scale_factor), config.depth_scale_factor);

  config.reprojected_topic_name =
      param::map::keyGetValue(params, NAMEOF(config.reprojected_topic_name),
                              config.reprojected_topic_name);

  return config;
}

bool InputHandler::Config::isValid(bool verbose) const {
  bool all_valid = true;

  all_valid &= IS_PARAM_NE(topic_name, std::string(""), verbose);
  all_valid &= IS_PARAM_GT(topic_queue_length, 0, verbose);
  all_valid &= IS_PARAM_GT(processing_retry_period, 0.f, verbose);
  all_valid &= IS_PARAM_GE(max_wait_for_pose, 0.f, verbose);

  return all_valid;
}
}  // namespace wavemap
