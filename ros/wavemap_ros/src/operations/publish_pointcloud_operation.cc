#include "wavemap_ros/operations/publish_pointcloud_operation.h"

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tracy/Tracy.hpp>
#include <wavemap/indexing/index_conversions.h>
#include <wavemap/utils/iterate/grid_iterator.h>
#include <wavemap_ros_conversions/geometry_msg_conversions.h>

namespace wavemap {
DECLARE_CONFIG_MEMBERS(PublishPointcloudOperationConfig,
                      (once_every)
                      (occupancy_threshold_log_odds)
                      (topic));

bool PublishPointcloudOperationConfig::isValid(bool verbose) const {
  bool all_valid = true;

  all_valid &= IS_PARAM_GT(once_every, 0.f, verbose);
  all_valid &= IS_PARAM_NE(topic, std::string(), verbose);

  return all_valid;
}

PublishPointcloudOperation::PublishPointcloudOperation(
    const PublishPointcloudOperationConfig& config, std::string world_frame,
    VolumetricDataStructureBase::Ptr occupancy_map, ros::NodeHandle nh_private)
    : config_(config.checkValid()),
      world_frame_(std::move(world_frame)),
      occupancy_map_(std::move(occupancy_map)) {
  pointcloud_pub_ =
      nh_private.advertise<sensor_msgs::PointCloud2>(config_.topic, 10);
}

void PublishPointcloudOperation::publishPointcloud(
    const ros::Time& current_time) {
  ZoneScoped;
  // If the map is empty, there's no work to do
  if (occupancy_map_->empty()) {
    return;
  }

  // Create the message and set the header
  sensor_msgs::PointCloud obstacle_pointcloud_msg;
  obstacle_pointcloud_msg.header.frame_id = world_frame_;
  obstacle_pointcloud_msg.header.stamp = current_time;

  // Add the points
  occupancy_map_->forEachLeaf([&point_msg_vector =
                                   obstacle_pointcloud_msg.points,
                               cell_width = occupancy_map_->getMinCellWidth(),
                               occupancy_threshold =
                                   config_.occupancy_threshold_log_odds](
                                  const OctreeIndex& node_index,
                                  FloatingPoint node_log_odds) {
    if (occupancy_threshold < node_log_odds) {
      if (node_index.height == 0) {
        const Point3D center =
            convert::indexToCenterPoint(node_index.position, cell_width);
        point_msg_vector.emplace_back(convert::point3DToPoint32Msg(center));
      } else {
        const Index3D node_min_corner =
            convert::nodeIndexToMinCornerIndex(node_index);
        const Index3D node_max_corner =
            convert::nodeIndexToMaxCornerIndex(node_index);
        for (const Index3D& index : Grid(node_min_corner, node_max_corner)) {
          const Point3D center = convert::indexToCenterPoint(index, cell_width);
          point_msg_vector.emplace_back(convert::point3DToPoint32Msg(center));
        }
      }
    }
  });

  // Publish the msg
  sensor_msgs::PointCloud2 obstacle_pointcloud2_msg;
  sensor_msgs::convertPointCloudToPointCloud2(obstacle_pointcloud_msg,
                                              obstacle_pointcloud2_msg);
  pointcloud_pub_.publish(obstacle_pointcloud2_msg);
}
}  // namespace wavemap
