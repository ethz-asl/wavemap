#include "wavemap_ros/inputs/depth_image_input.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/eigen.hpp>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <wavemap/core/integrator/projective/projective_integrator.h>
#include <wavemap/core/utils/iterate/grid_iterator.h>
#include <wavemap/core/utils/print/eigen.h>
#include <wavemap/core/utils/profiler_interface.h>

namespace wavemap {
DECLARE_CONFIG_MEMBERS(DepthImageInputConfig,
                      (topic_name)
                      (topic_queue_length)
                      (measurement_integrator_names)
                      (processing_retry_period)
                      (max_wait_for_pose)
                      (sensor_frame_id)
                      (image_transport_hints)
                      (depth_scale_factor)
                      (time_offset)
                      (projected_pointcloud_topic_name));

bool DepthImageInputConfig::isValid(bool verbose) const {
  bool all_valid = true;

  all_valid &= IS_PARAM_NE(topic_name, std::string(""), verbose);
  all_valid &= IS_PARAM_GT(topic_queue_length, 0, verbose);
  all_valid &= IS_PARAM_GT(processing_retry_period, 0.f, verbose);
  all_valid &= IS_PARAM_GE(max_wait_for_pose, 0.f, verbose);

  return all_valid;
}

DepthImageInput::DepthImageInput(const DepthImageInputConfig& config,
                                 std::shared_ptr<Pipeline> pipeline,
                                 std::shared_ptr<TfTransformer> transformer,
                                 std::string world_frame, ros::NodeHandle nh,
                                 ros::NodeHandle nh_private)
    : InputBase(config, std::move(pipeline), std::move(transformer),
                std::move(world_frame), nh, nh_private),
      config_(config.checkValid()) {
  // Subscribe to the depth image input
  image_transport::ImageTransport it(nh);
  depth_image_sub_ = it.subscribe(
      config_.topic_name, config_.topic_queue_length,
      &DepthImageInput::callback, this,
      image_transport::TransportHints(config_.image_transport_hints));

  // Advertise the projected pointcloud publisher if enabled
  if (!config_.projected_pointcloud_topic_name.empty()) {
    projected_pointcloud_pub_ = nh_private.advertise<sensor_msgs::PointCloud2>(
        config_.projected_pointcloud_topic_name, config_.topic_queue_length);
  }
}

void DepthImageInput::processQueue() {
  ProfilerZoneScoped;
  while (!depth_image_queue_.empty()) {
    const sensor_msgs::Image& oldest_msg = depth_image_queue_.front();
    const std::string sensor_frame_id = config_.sensor_frame_id.empty()
                                            ? oldest_msg.header.frame_id
                                            : config_.sensor_frame_id;

    // Get the sensor pose in world frame
    Transformation3D T_W_C;
    const ros::Time stamp =
        oldest_msg.header.stamp + ros::Duration(config_.time_offset);
    if (!transformer_->lookupTransform(world_frame_, sensor_frame_id, stamp,
                                       T_W_C)) {
      const auto newest_msg = depth_image_queue_.back();
      if ((newest_msg.header.stamp - oldest_msg.header.stamp).toSec() <
          config_.max_wait_for_pose) {
        // Try to get this depth image's pose again at the next iteration
        return;
      } else {
        ROS_WARN_STREAM("Waited " << config_.max_wait_for_pose
                                  << "s but still could not look up pose for "
                                     "depth image with frame \""
                                  << sensor_frame_id << "\" in world frame \""
                                  << world_frame_ << "\" at timestamp " << stamp
                                  << "; skipping depth image.");
        depth_image_queue_.pop();
        continue;
      }
    }

    // Convert the depth image to our coordinate convention
    auto cv_image = cv_bridge::toCvCopy(oldest_msg);
    if (!cv_image) {
      return;
    }
    cv::transpose(cv_image->image, cv_image->image);
    cv_image->image.convertTo(cv_image->image, CV_32FC1,
                              config_.depth_scale_factor);

    // Create the posed depth image input
    PosedImage<> posed_depth_image(cv_image->image.rows, cv_image->image.cols);
    cv::cv2eigen<FloatingPoint>(cv_image->image, posed_depth_image.getData());
    posed_depth_image.setPose(T_W_C);

    // Integrate the depth image
    ROS_DEBUG_STREAM("Inserting depth image with "
                     << print::eigen::oneLine(posed_depth_image.getDimensions())
                     << " points. Remaining pointclouds in queue: "
                     << depth_image_queue_.size() - 1 << ".");
    integration_timer_.start();
    pipeline_->runPipeline(config_.measurement_integrator_names,
                           posed_depth_image);
    integration_timer_.stop();
    ROS_DEBUG_STREAM("Integrated new depth image in "
                     << integration_timer_.getLastEpisodeDuration()
                     << "s. Total integration time: "
                     << integration_timer_.getTotalDuration() << "s.");

    // Publish debugging visualizations
    publishProjectedPointcloudIfEnabled(stamp, posed_depth_image);
    ProfilerFrameMarkNamed("DepthImage");

    // Remove the depth image from the queue
    depth_image_queue_.pop();
  }
}

void DepthImageInput::publishProjectedPointcloudIfEnabled(
    const ros::Time& stamp,
    const PosedImage<FloatingPoint>& /*posed_depth_image*/) {
  ProfilerZoneScoped;
  if (config_.projected_pointcloud_topic_name.empty() ||
      projected_pointcloud_pub_.getNumSubscribers() <= 0) {
    return;
  }

  sensor_msgs::PointCloud pointcloud_msg;
  pointcloud_msg.header.stamp = stamp;
  pointcloud_msg.header.frame_id = world_frame_;

  // TODO(victorr): Reimplement this
  // auto projective_integrator =
  //     std::dynamic_pointer_cast<ProjectiveIntegrator>(integrators_.front());
  // if (!projective_integrator) {
  //   return {};
  // }
  // const auto& projection_model =
  //     projective_integrator->getProjectionModel();

  // const auto posed_pointcloud = project(posed_depth_image,
  // projection_model); pointcloud_msg.points.reserve(posed_pointcloud.size());
  // for (const auto& point : posed_pointcloud.getPointsGlobal()) {
  //   auto& point_msg = pointcloud_msg.points.emplace_back();
  //   point_msg.x = point.x();
  //   point_msg.y = point.y();
  //   point_msg.z = point.z();
  // }
  //
  // sensor_msgs::PointCloud2 pointcloud2_msg;
  // sensor_msgs::convertPointCloudToPointCloud2(pointcloud_msg,
  // pointcloud2_msg); projected_pointcloud_pub_.publish(pointcloud2_msg);
}

PosedPointcloud<Point3D> DepthImageInput::project(
    const PosedImage<>& posed_depth_image,
    const ProjectorBase& projection_model) {
  ProfilerZoneScoped;
  std::vector<Point3D> pointcloud;
  pointcloud.reserve(posed_depth_image.size());
  for (const Index2D& index :
       Grid<2>(Index2D::Zero(),
               posed_depth_image.getDimensions() - Index2D::Ones())) {
    const Vector2D image_xy = projection_model.indexToImage(index);
    const FloatingPoint image_z = posed_depth_image.at(index);
    const Point3D C_point =
        projection_model.sensorToCartesian(image_xy, image_z);
    pointcloud.emplace_back(C_point);
  }

  return PosedPointcloud<Point3D>{posed_depth_image.getPose(), pointcloud};
}
}  // namespace wavemap
