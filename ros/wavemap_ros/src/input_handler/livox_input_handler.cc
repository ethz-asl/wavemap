#include "wavemap_ros/input_handler/livox_input_handler.h"

#include <sensor_msgs/point_cloud2_iterator.h>

#include "wavemap_ros/utils/time_conversions.h"

namespace wavemap {
LivoxInputHandler::LivoxInputHandler(
    const Config& config, const param::Map& params, std::string world_frame,
    VolumetricDataStructureBase::Ptr occupancy_map,
    std::shared_ptr<TfTransformer> transformer, ros::NodeHandle nh,
    ros::NodeHandle nh_private)
    : InputHandler(config, params, std::move(world_frame),
                   std::move(occupancy_map), std::move(transformer), nh,
                   std::move(nh_private)) {
  // Subscribe to the pointcloud input
  pointcloud_sub_ = nh.subscribe(config_.topic_name, config_.topic_queue_length,
                                 &LivoxInputHandler::pointcloudCallback, this);
}

void LivoxInputHandler::processQueue() {
  while (!pointcloud_queue_.empty()) {
    auto& oldest_msg = pointcloud_queue_.front();
    const std::string sensor_frame_id = config_.sensor_frame_id.empty()
                                            ? oldest_msg.header.frame_id
                                            : config_.sensor_frame_id;

    // Skip empty clouds
    if (oldest_msg.points.empty()) {
      ROS_WARN_STREAM("Skipping empty pointcloud with timestamp "
                      << oldest_msg.header.stamp << ".");
      pointcloud_queue_.pop();
      continue;
    }

    // Sort the points by time
    std::sort(oldest_msg.points.begin(), oldest_msg.points.end(),
              [](const auto& lhs, const auto& rhs) {
                return lhs.offset_time < rhs.offset_time;
              });

    // Make sure all transforms are available
    uint64_t start_time =
        oldest_msg.timebase + oldest_msg.points.front().offset_time;
    uint64_t end_time =
        oldest_msg.timebase + oldest_msg.points.back().offset_time;
    if (!transformer_->isTransformAvailable(world_frame_, sensor_frame_id,
                                            nanoSecondsToRosTime(end_time))) {
      const auto newest_msg = pointcloud_queue_.back();
      if ((newest_msg.header.stamp - nanoSecondsToRosTime(end_time)).toSec() <
          config_.max_wait_for_pose) {
        // Try to get this pointcloud's pose again at the next iteration
        return;
      } else {
        ROS_WARN_STREAM("Waited " << config_.max_wait_for_pose
                                  << "s but still could not look up pose for "
                                     "pointcloud with frame \""
                                  << sensor_frame_id << "\" in world frame \""
                                  << world_frame_ << "\" at timestamp "
                                  << end_time << ". Skipping pointcloud.");
        pointcloud_queue_.pop();
        continue;
      }
    }
    if (!transformer_->isTransformAvailable(world_frame_, sensor_frame_id,
                                            nanoSecondsToRosTime(start_time))) {
      ROS_WARN_STREAM(
          "Pointcloud end pose is available but start pose is not. Skipping "
          "pointcloud.");
      pointcloud_queue_.pop();
      continue;
    }

    // Buffer the transforms
    constexpr int kNumTimeIntervals = 100;
    constexpr int kNumTimeSteps = kNumTimeIntervals + 1;
    std::vector<std::pair<uint64_t, Transformation3D>> timed_poses;
    timed_poses.reserve(kNumTimeSteps);
    const uint64_t step_size = (oldest_msg.points.back().offset_time -
                                oldest_msg.points.front().offset_time) /
                               (kNumTimeIntervals - 1);
    for (unsigned int step_idx = 0u; step_idx < kNumTimeSteps; ++step_idx) {
      auto& timed_pose = timed_poses.emplace_back();
      timed_pose.first = start_time + step_idx * step_size;
      if (!transformer_->lookupTransform(world_frame_, sensor_frame_id,
                                         nanoSecondsToRosTime(timed_pose.first),
                                         timed_pose.second)) {
        ROS_WARN_STREAM("Failed to buffer intermediate pose at timestamp "
                        << nanoSecondsToRosTime(timed_pose.first)
                        << " This should never happen. Skipping "
                           "pointcloud.");
        pointcloud_queue_.pop();
        continue;
      }
    }

    // Motion undistort the points
    const auto num_rays = static_cast<int>(oldest_msg.points.size());
    Eigen::Matrix<FloatingPoint, 3, Eigen::Dynamic> t_W_points;
    t_W_points.resize(3, num_rays);
    int l_idx = 0u;
    uint64_t prev_time = -1u;
    Transformation3D T_WCi;
    for (int idx = 0u; idx < num_rays; ++idx) {
      const auto& point = oldest_msg.points[idx];
      const Point3D Ci_p{point.x, point.y, point.z};

      const uint64_t time = oldest_msg.timebase + point.offset_time;
      if (time != prev_time) {
        prev_time = time;
        while (timed_poses[l_idx + 1].first < time &&
               l_idx + 2 < kNumTimeSteps) {
          ++l_idx;
        }
        CHECK_LT(l_idx + 1, timed_poses.size());
        const uint64_t time_l = timed_poses[l_idx].first;
        const uint64_t time_u = timed_poses[l_idx + 1].first;
        const Transformation3D& T_WCl = timed_poses[l_idx].second;
        const Transformation3D& T_WCu = timed_poses[l_idx + 1].second;
        FloatingPoint a = static_cast<FloatingPoint>((time - time_l)) /
                          static_cast<FloatingPoint>((time_u - time_l));
        CHECK_GE(a, 0.f);
        CHECK_LE(a, 1.f);
        T_WCi = interpolateComponentwise(T_WCl, T_WCu, a);
      }

      t_W_points.col(idx) = T_WCi * Ci_p;
    }
    const Transformation3D T_WCmid = timed_poses[kNumTimeIntervals / 2].second;
    const PosedPointcloud<> posed_pointcloud(
        T_WCmid, T_WCmid.inverse().transformVectorized(t_W_points));

    // Reproject if enabled
    if (isReprojectionEnabled()) {
      publishReprojected(oldest_msg.header.stamp, posed_pointcloud);
    }

    // Integrate the pointcloud
    ROS_INFO_STREAM("Inserting pointcloud with "
                    << posed_pointcloud.size()
                    << " points. Remaining pointclouds in queue: "
                    << pointcloud_queue_.size() - 1 << ".");
    integration_timer_.start();
    integrator_->integratePointcloud(posed_pointcloud);
    const double pointcloud_integration_time = integration_timer_.stop();
    const double total_pointcloud_integration_time =
        integration_timer_.getTotal();
    ROS_INFO_STREAM("Integrated new pointcloud in "
                    << pointcloud_integration_time
                    << "s. Total integration time: "
                    << total_pointcloud_integration_time << "s.");

    // Remove the pointcloud from the queue
    pointcloud_queue_.pop();
  }
}
}  // namespace wavemap
