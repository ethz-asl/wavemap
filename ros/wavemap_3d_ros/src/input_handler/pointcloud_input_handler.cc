#include "wavemap_3d_ros/input_handler/pointcloud_input_handler.h"

#include <sensor_msgs/point_cloud2_iterator.h>

namespace wavemap {
PointcloudInputHandler::PointcloudInputHandler(
    const param::Map& params, std::string world_frame,
    VolumetricDataStructure3D::Ptr occupancy_map,
    std::shared_ptr<TfTransformer> transformer, ros::NodeHandle nh)
    : InputHandler(params, std::move(world_frame), std::move(occupancy_map),
                   std::move(transformer), nh) {
  // Subscribe to the pointcloud input
  pointcloud_sub_ =
      nh.subscribe(config_.topic_name, config_.topic_queue_length,
                   &PointcloudInputHandler::pointcloudCallback, this);
}

void PointcloudInputHandler::processQueue() {
  while (!pointcloud_queue_.empty()) {
    const sensor_msgs::PointCloud2& pointcloud_msg = pointcloud_queue_.front();

    // Get the sensor pose in world frame
    Transformation3D T_W_C;
    if (!transformer_->isTransformAvailable(world_frame_,
                                            pointcloud_msg.header.frame_id,
                                            pointcloud_msg.header.stamp) ||
        !transformer_->lookupTransform(world_frame_,
                                       pointcloud_msg.header.frame_id,
                                       pointcloud_msg.header.stamp, T_W_C)) {
      if ((pointcloud_queue_.back().header.stamp - pointcloud_msg.header.stamp)
              .toSec() < config_.max_wait_for_pose) {
        // Try to get this pointcloud's pose again at the next iteration
        return;
      } else {
        ROS_WARN_STREAM("Waited " << config_.max_wait_for_pose
                                  << "s but still could not look up pose for "
                                     "pointcloud with frame \""
                                  << pointcloud_msg.header.frame_id
                                  << "\" in world frame \"" << world_frame_
                                  << "\" at timestamp "
                                  << pointcloud_msg.header.stamp
                                  << "; skipping pointcloud.");
        pointcloud_queue_.pop();
        continue;
      }
    }

    // Convert the scan to a pointcloud
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
    // Load the points into our internal Pointcloud type
    const size_t num_rays = pointcloud_msg.width * pointcloud_msg.height;
    std::vector<Point3D> t_C_points;
    t_C_points.reserve(num_rays);
    for (sensor_msgs::PointCloud2ConstIterator<float> it(pointcloud_msg, "x");
         it != it.end(); ++it) {
      t_C_points.emplace_back(it[0], it[1], it[2]);
    }

    // Integrate the pointcloud
    ROS_INFO_STREAM("Inserting pointcloud with "
                    << t_C_points.size()
                    << " points. Remaining pointclouds in queue: "
                    << pointcloud_queue_.size() - 1 << ".");
    const PosedPointcloud posed_pointcloud(T_W_C,
                                           Pointcloud<Point3D>(t_C_points));
    integration_timer_.start();
    integrator_->integratePointcloud(posed_pointcloud);
    const double pointcloud_integration_time = integration_timer_.stop();
    const double total_pointcloud_integration_time =
        integration_timer_.getTotal();
    ROS_INFO_STREAM("Integrated new pointcloud in "
                    << pointcloud_integration_time
                    << "s. Total integration time: "
                    << total_pointcloud_integration_time << "s.");

    //  if (config_.publish_performance_stats) {
    //    const size_t map_memory_usage = occupancy_map_->getMemoryUsage();
    //    ROS_INFO_STREAM("Map memory usage: " << map_memory_usage / 1e6 <<
    //    "MB.");
    //
    //    wavemap_msgs::PerformanceStats performance_stats_msg;
    //    performance_stats_msg.map_memory_usage = map_memory_usage;
    //    performance_stats_msg.total_pointcloud_integration_time =
    //        total_pointcloud_integration_time;
    //    performance_stats_pub_.publish(performance_stats_msg);
    //  }

    // Remove the pointcloud from the queue
    pointcloud_queue_.pop();
  }
}
}  // namespace wavemap
