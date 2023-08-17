#include "wavemap_ros/input_handler/depth_image_input_handler.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/eigen.hpp>
#include <tracy/Tracy.hpp>
#include <wavemap/iterator/grid_iterator.h>
#include <wavemap/utils/eigen_format.h>

namespace wavemap {
DECLARE_CONFIG_MEMBERS(DepthImageInputHandlerConfig,
                      (topic_name)
                      (topic_queue_length)
                      (processing_retry_period, SiUnit::kSeconds)
                      (max_wait_for_pose, SiUnit::kSeconds)
                      (sensor_frame_id)
                      (image_transport_hints)
                      (depth_scale_factor)
                      (time_offset, SiUnit::kSeconds)
                      (reprojected_pointcloud_topic_name)
                      (projected_range_image_topic_name));

bool DepthImageInputHandlerConfig::isValid(bool verbose) const {
  bool all_valid = true;

  all_valid &= IS_PARAM_NE(topic_name, std::string(""), verbose);
  all_valid &= IS_PARAM_GT(topic_queue_length, 0, verbose);
  all_valid &= IS_PARAM_GT(processing_retry_period, 0.f, verbose);
  all_valid &= IS_PARAM_GE(max_wait_for_pose, 0.f, verbose);

  return all_valid;
}

DepthImageInputHandler::DepthImageInputHandler(
    const DepthImageInputHandlerConfig& config, const param::Map& params,
    std::string world_frame, VolumetricDataStructureBase::Ptr occupancy_map,
    std::shared_ptr<TfTransformer> transformer, ros::NodeHandle nh,
    ros::NodeHandle nh_private)
    : InputHandler(config, params, std::move(world_frame),
                   std::move(occupancy_map), std::move(transformer), nh,
                   nh_private),
      config_(config.checkValid()) {
  // Get pointers to the underlying scanwise integrators
  for (const auto& integrator : integrators_) {
    auto scanwise_integrator =
        std::dynamic_pointer_cast<ProjectiveIntegrator>(integrator);
    CHECK(scanwise_integrator)
        << "Depth image inputs are currently only supported in "
           "combination with projective integrators.";
    scanwise_integrators_.emplace_back(std::move(scanwise_integrator));
  }

  // Subscribe to the depth image input
  image_transport::ImageTransport it(nh);
  depth_image_sub_ = it.subscribe(
      config_.topic_name, config_.topic_queue_length,
      &DepthImageInputHandler::callback, this,
      image_transport::TransportHints(config_.image_transport_hints));
}

void DepthImageInputHandler::processQueue() {
  ZoneScoped;
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

    // Create the posed range image input
    PosedImage<> posed_range_image(cv_image->image.rows, cv_image->image.cols);
    cv::cv2eigen<FloatingPoint>(cv_image->image, posed_range_image.getData());
    posed_range_image.setPose(T_W_C);

    // Integrate the depth image
    ROS_INFO_STREAM("Inserting depth image with "
                    << EigenFormat::oneLine(posed_range_image.getDimensions())
                    << " points. Remaining pointclouds in queue: "
                    << depth_image_queue_.size() - 1 << ".");
    integration_timer_.start();
    for (const auto& integrator : scanwise_integrators_) {
      integrator->integrateRangeImage(posed_range_image);
    }
    integration_timer_.stop();
    ROS_INFO_STREAM("Integrated new depth image in "
                    << integration_timer_.getLastEpisodeDuration()
                    << "s. Total integration time: "
                    << integration_timer_.getTotalDuration() << "s.");

    // Publish debugging visualizations
    if (shouldPublishReprojectedPointcloud()) {
      const auto posed_pointcloud = reproject(posed_range_image);
      publishReprojectedPointcloud(stamp, posed_pointcloud);
    }
    if (shouldPublishProjectedRangeImage()) {
      auto projective_integrator =
          std::dynamic_pointer_cast<ProjectiveIntegrator>(integrators_.front());
      if (projective_integrator) {
        const auto& range_image = projective_integrator->getPosedRangeImage();
        if (range_image) {
          publishProjectedRangeImage(stamp, *range_image);
        }
      }
    }
    FrameMarkNamed("DepthImage");

    // Remove the depth image from the queue
    depth_image_queue_.pop();
  }
}

PosedPointcloud<> DepthImageInputHandler::reproject(
    const PosedImage<>& posed_range_image) {
  ZoneScoped;
  auto projective_integrator =
      std::dynamic_pointer_cast<ProjectiveIntegrator>(integrators_.front());
  if (!projective_integrator) {
    return {};
  }
  const auto& projection_model = projective_integrator->getProjectionModel();

  std::vector<Point3D> pointcloud;
  pointcloud.reserve(posed_range_image.size());
  for (const Index2D& index :
       Grid<2>(Index2D::Zero(),
               posed_range_image.getDimensions() - Index2D::Ones())) {
    const Vector2D image_xy = projection_model->indexToImage(index);
    const FloatingPoint image_z = posed_range_image.at(index);
    const Point3D C_point =
        projection_model->sensorToCartesian(image_xy, image_z);
    pointcloud.emplace_back(C_point);
  }

  return PosedPointcloud<>(posed_range_image.getPose(), pointcloud);
}
}  // namespace wavemap
