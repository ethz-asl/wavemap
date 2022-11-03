#include "wavemap_3d_ros/input_handler/depth_image_input_handler.h"

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/eigen.hpp>
#include <wavemap_3d/integrator/projective/range_image_2d.h>
#include <wavemap_common/utils/eigen_format.h>

namespace wavemap {
DepthImageInputHandler::DepthImageInputHandler(
    const Config& config, const param::Map& params, std::string world_frame,
    VolumetricDataStructure3D::Ptr occupancy_map,
    std::shared_ptr<TfTransformer> transformer, ros::NodeHandle nh)
    : InputHandler(config, params, std::move(world_frame),
                   std::move(occupancy_map), std::move(transformer), nh) {
  // Get a pointer to the underlying scanwise integrator
  scanwise_integrator_ =
      std::dynamic_pointer_cast<ScanwiseIntegrator3D>(integrator_);
  CHECK(integrator_) << "Depth image inputs are currently only supported in "
                        "combination with projective integrators.";

  // Subscribe to the depth image input
  depth_image_sub_ =
      nh.subscribe(config_.topic_name, config_.topic_queue_length,
                   &DepthImageInputHandler::depthImageCallback, this);
}

void DepthImageInputHandler::processQueue() {
  while (!depth_image_queue_.empty()) {
    const sensor_msgs::Image& oldest_msg = depth_image_queue_.front();

    // Get the sensor pose in world frame
    Transformation3D T_W_C;
    if (!transformer_->lookupTransform(world_frame_, oldest_msg.header.frame_id,
                                       oldest_msg.header.stamp, T_W_C)) {
      const auto newest_msg = depth_image_queue_.back();
      if ((newest_msg.header.stamp - oldest_msg.header.stamp).toSec() <
          config_.max_wait_for_pose) {
        // Try to get this depth image's pose again at the next iteration
        return;
      } else {
        ROS_WARN_STREAM(
            "Waited " << config_.max_wait_for_pose
                      << "s but still could not look up pose for "
                         "depth image with frame \""
                      << oldest_msg.header.frame_id << "\" in world frame \""
                      << world_frame_ << "\" at timestamp "
                      << oldest_msg.header.stamp << "; skipping depth image.");
        depth_image_queue_.pop();
        continue;
      }
    }

    // Convert the depth image to our coordinate convention
    auto cv_image = cv_bridge::toCvCopy(oldest_msg, oldest_msg.encoding);
    if (!cv_image) {
      return;
    }
    cv::transpose(cv_image->image, cv_image->image);
    cv_image->image.convertTo(cv_image->image, CV_32FC1);

    // Create the posed range image input
    PosedRangeImage2D posed_range_image(cv_image->image.rows,
                                        cv_image->image.cols);
    cv::cv2eigen<FloatingPoint>(cv_image->image, posed_range_image.getData());
    posed_range_image.setPose(T_W_C);

    // Integrate the depth image
    ROS_INFO_STREAM("Inserting depth image with "
                    << EigenFormat::oneLine(posed_range_image.getDimensions())
                    << " points. Remaining pointclouds in queue: "
                    << depth_image_queue_.size() - 1 << ".");
    integration_timer_.start();
    scanwise_integrator_->integrateRangeImage(posed_range_image);
    const double pointcloud_integration_time = integration_timer_.stop();
    const double total_pointcloud_integration_time =
        integration_timer_.getTotal();
    ROS_INFO_STREAM("Integrated new depth image in "
                    << pointcloud_integration_time
                    << "s. Total integration time: "
                    << total_pointcloud_integration_time << "s.");

    // Remove the depth image from the queue
    depth_image_queue_.pop();
  }
}
}  // namespace wavemap