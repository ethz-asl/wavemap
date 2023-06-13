#include "wavemap_ros/input_handler/pointcloud_input_handler.h"

#include <sensor_msgs/point_cloud2_iterator.h>
#include <wavemap/integrator/projective/projective_integrator.h>
#include <wavemap_ros_conversions/time_conversions.h>

namespace wavemap {
DECLARE_CONFIG_MEMBERS(PointcloudInputHandlerConfig,
                      (topic_name)
                      (topic_queue_length)
                      (pointcloud_type)
                      (processing_retry_period, SiUnit::kSeconds)
                      (max_wait_for_pose, SiUnit::kSeconds)
                      (sensor_frame_id)
                      (time_offset, SiUnit::kSeconds)
                      (undistort_motion)
                      (reprojected_pointcloud_topic_name)
                      (projected_range_image_topic_name));

bool PointcloudInputHandlerConfig::isValid(bool verbose) const {
  bool all_valid = true;

  all_valid &= IS_PARAM_NE(topic_name, std::string(""), verbose);
  all_valid &= IS_PARAM_GT(topic_queue_length, 0, verbose);
  all_valid &= IS_PARAM_GT(processing_retry_period, 0.f, verbose);
  all_valid &= IS_PARAM_GE(max_wait_for_pose, 0.f, verbose);

  return all_valid;
}

PointcloudInputHandler::PointcloudInputHandler(
    const PointcloudInputHandlerConfig& config, const param::Map& params,
    std::string world_frame, VolumetricDataStructureBase::Ptr occupancy_map,
    std::shared_ptr<TfTransformer> transformer, ros::NodeHandle nh,
    ros::NodeHandle nh_private)
    : InputHandler(config, params, std::move(world_frame),
                   std::move(occupancy_map), std::move(transformer), nh,
                   nh_private),
      config_(config.checkValid()),
      pointcloud_undistorter_(transformer) {
  // Subscribe to the pointcloud input
  switch (config_.pointcloud_type.toTypeId()) {
    case PointcloudType::kPointCloud2:
      pointcloud_sub_ = nh.subscribe<sensor_msgs::PointCloud2>(
          config_.topic_name, config_.topic_queue_length,
          [this](sensor_msgs::PointCloud2::ConstPtr msg) {
            pointcloudCallback(*msg);
          });
      break;
    case PointcloudType::kLivox:
#ifdef LIVOX_AVAILABLE
      pointcloud_sub_ = nh.subscribe<livox_ros_driver2::CustomMsg>(
          config_.topic_name, config_.topic_queue_length,
          [this](livox_ros_driver2::CustomMsg::ConstPtr msg) {
            pointcloudCallback(*msg);
          });
#else
      LOG(ERROR) << "Livox support is currently not available. Please install "
                    "livox_ros_driver2 and rebuild wavemap.";
      return nullptr;
#endif
      break;
  }
}

void PointcloudInputHandler::pointcloudCallback(
    const sensor_msgs::PointCloud2& pointcloud_msg) {
  // Skip empty clouds
  const size_t num_points = pointcloud_msg.height * pointcloud_msg.width;
  if (num_points == 0) {
    ROS_WARN_STREAM("Skipping empty pointcloud with timestamp "
                    << pointcloud_msg.header.stamp << ".");
    return;
  }

  // Get the sensor frame
  const std::string sensor_frame_id = config_.sensor_frame_id.empty()
                                          ? pointcloud_msg.header.frame_id
                                          : config_.sensor_frame_id;

  // Convert to our universal point cloud format
  StampedPointcloud stamped_pointcloud(0u, world_frame_, sensor_frame_id,
                                       num_points);
  // Get the index of the x field, and assert that the y and z fields follow
  auto x_field_iter = std::find_if(
      pointcloud_msg.fields.cbegin(), pointcloud_msg.fields.cend(),
      [](const sensor_msgs::PointField& field) { return field.name == "x"; });
  if (x_field_iter == pointcloud_msg.fields.end()) {
    ROS_WARN("Received pointcloud with missing field x");
    return;
  } else if ((++x_field_iter)->name != "y") {
    ROS_WARN("Received pointcloud with missing or out-of-order field y");
    return;
  } else if ((++x_field_iter)->name != "z") {
    ROS_WARN("Received pointcloud with missing or out-of-order field z");
    return;
  }
  // Load the points
  for (sensor_msgs::PointCloud2ConstIterator<float> it(pointcloud_msg, "x");
       it != it.end(); ++it) {
    stamped_pointcloud.emplace(it[0], it[1], it[2], 0);
  }

  pointcloud_queue_.emplace(stamped_pointcloud);
}

void PointcloudInputHandler::pointcloudCallback(
    const livox_ros_driver2::CustomMsg& pointcloud_msg) {
  // Skip empty clouds
  if (pointcloud_msg.points.empty()) {
    ROS_WARN_STREAM("Skipping empty pointcloud with timestamp "
                    << pointcloud_msg.header.stamp << ".");
    return;
  }

  // Get the sensor frame
  const std::string sensor_frame_id = config_.sensor_frame_id.empty()
                                          ? pointcloud_msg.header.frame_id
                                          : config_.sensor_frame_id;

  // Convert to our universal point cloud format
  StampedPointcloud stamped_pointcloud(pointcloud_msg.timebase, world_frame_,
                                       sensor_frame_id,
                                       pointcloud_msg.points.size());
  for (const auto& point : pointcloud_msg.points) {
    stamped_pointcloud.emplace(point.x, point.y, point.z, point.offset_time);
  }
  stamped_pointcloud.sort();

  pointcloud_queue_.emplace(stamped_pointcloud);
}

void PointcloudInputHandler::processQueue() {
  while (!pointcloud_queue_.empty()) {
    auto& oldest_msg = pointcloud_queue_.front();

    // Undistort the point cloud
    PosedPointcloud<> posed_pointcloud;
    const auto undistortion_result =
        pointcloud_undistorter_.undistortPointcloud(oldest_msg,
                                                    posed_pointcloud);
    if (undistortion_result != PointcloudUndistorter::Result::kSuccess) {
      const auto newest_msg = pointcloud_queue_.back();
      const uint64_t start_time = oldest_msg.getStartTime();
      const uint64_t end_time = oldest_msg.getEndTime();
      switch (undistortion_result) {
        case PointcloudUndistorter::Result::kEndTimeNotInTfBuffer:
          if ((convert::nanoSecondsToRosTime(newest_msg.time_base) -
               convert::nanoSecondsToRosTime(end_time))
                  .toSec() < config_.max_wait_for_pose) {
            // Try to get this pointcloud's pose again at the next iteration
            return;
          } else {
            ROS_WARN_STREAM("Waited "
                            << config_.max_wait_for_pose
                            << "s but still could not look up end pose "
                               "for pointcloud with frame \""
                            << oldest_msg.sensor_frame << "\" in world frame \""
                            << world_frame_ << "\" spanning time interval ["
                            << start_time << ", " << end_time
                            << "]. Skipping pointcloud.");
          }
          break;
        case PointcloudUndistorter::Result::kStartTimeNotInTfBuffer:
          ROS_WARN_STREAM(
              "Pointcloud end pose is available but start pose at time "
              << start_time << " is not. Skipping pointcloud.");
          break;
        case PointcloudUndistorter::Result::kIntermediateTimeNotInTfBuffer:
          ROS_WARN_STREAM(
              "Could not buffer all transforms for pointcloud spanning time "
              "interval ["
              << start_time << ", " << end_time
              << "]. This should never happen. Skipping pointcloud.");
          break;
        default:
          ROS_WARN("Unknown point cloud undistortion error.");
      }

      pointcloud_queue_.pop();
      continue;
    }

    // Integrate the pointcloud
    ROS_INFO_STREAM("Inserting pointcloud with "
                    << posed_pointcloud.size()
                    << " points. Remaining pointclouds in queue: "
                    << pointcloud_queue_.size() - 1 << ".");
    integration_timer_.start();
    for (const auto& integrator : integrators_) {
      integrator->integratePointcloud(posed_pointcloud);
    }
    integration_timer_.stop();
    ROS_INFO_STREAM("Integrated new pointcloud in "
                    << integration_timer_.getLastEpisodeWallTime()
                    << "s. Total integration time: "
                    << integration_timer_.getTotalWallTime() << "s.");

    // Publish debugging visualizations
    if (shouldPublishReprojectedPointcloud()) {
      publishReprojectedPointcloud(
          convert::nanoSecondsToRosTime(oldest_msg.time_base),
          posed_pointcloud);
    }
    if (shouldPublishProjectedRangeImage()) {
      auto projective_integrator =
          std::dynamic_pointer_cast<ProjectiveIntegrator>(integrators_.front());
      if (projective_integrator) {
        const auto& range_image = projective_integrator->getPosedRangeImage();
        if (range_image) {
          publishProjectedRangeImage(
              convert::nanoSecondsToRosTime(oldest_msg.time_base),
              *range_image);
        }
      }
    }

    // Remove the pointcloud from the queue
    pointcloud_queue_.pop();
  }
}
}  // namespace wavemap
