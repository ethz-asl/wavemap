#include "wavemap_ros/map_operations/publish_pointcloud_operation.h"

#include <string>
#include <utility>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <wavemap/core/indexing/index_conversions.h>
#include <wavemap/core/map/hashed_chunked_wavelet_octree.h>
#include <wavemap/core/map/hashed_wavelet_octree.h>
#include <wavemap/core/utils/iterate/grid_iterator.h>
#include <wavemap/core/utils/profiler_interface.h>
#include <wavemap_ros_conversions/geometry_msg_conversions.h>

namespace wavemap {
DECLARE_CONFIG_MEMBERS(PublishPointcloudOperationConfig,
                      (once_every)
                      (occupancy_threshold_log_odds)
                      (only_publish_changed_blocks)
                      (topic));

bool PublishPointcloudOperationConfig::isValid(bool verbose) const {
  bool all_valid = true;

  all_valid &= IS_PARAM_GT(once_every, 0.f, verbose);
  all_valid &=
      IS_PARAM_FALSE(std::isnan(occupancy_threshold_log_odds), verbose);
  all_valid &= IS_PARAM_NE(topic, "", verbose);

  return all_valid;
}

PublishPointcloudOperation::PublishPointcloudOperation(
    const PublishPointcloudOperationConfig& config, MapBase::Ptr occupancy_map,
    std::string world_frame, ros::NodeHandle nh_private)
    : MapOperationBase(std::move(occupancy_map)),
      config_(config.checkValid()),
      world_frame_(std::move(world_frame)) {
  pointcloud_pub_ =
      nh_private.advertise<sensor_msgs::PointCloud2>(config_.topic, 10);
}

bool PublishPointcloudOperation::shouldRun(const ros::Time& current_time) {
  return config_.once_every < (current_time - last_run_timestamp_).toSec();
}

void PublishPointcloudOperation::run(bool force_run) {
  const ros::Time current_time = ros::Time::now();
  if (force_run || shouldRun(current_time)) {
    publishPointcloud(current_time);
    last_run_timestamp_ = current_time;
  }
}

void PublishPointcloudOperation::publishPointcloud(
    const ros::Time& current_time) {
  ProfilerZoneScoped;
  // If the map is empty, there's no work to do
  if (occupancy_map_->empty()) {
    return;
  }

  // Create the message and set the header
  sensor_msgs::PointCloud obstacle_pointcloud_msg;
  obstacle_pointcloud_msg.header.frame_id = world_frame_;
  obstacle_pointcloud_msg.header.stamp = current_time;

  // Define a functor that converts map leaf nodes into pointcloud points
  auto add_points_for_leaf_node =
      [min_cell_width = occupancy_map_->getMinCellWidth(),
       occupancy_threshold = config_.occupancy_threshold_log_odds,
       &point_msg_vector = obstacle_pointcloud_msg.points](
          const OctreeIndex& node_index, FloatingPoint node_log_odds) {
        if (occupancy_threshold < node_log_odds) {
          if (node_index.height == 0) {
            const Point3D center = convert::indexToCenterPoint(
                node_index.position, min_cell_width);
            point_msg_vector.emplace_back(convert::point3DToPoint32Msg(center));
          } else {
            const Index3D node_min_corner =
                convert::nodeIndexToMinCornerIndex(node_index);
            const Index3D node_max_corner =
                convert::nodeIndexToMaxCornerIndex(node_index);
            for (const Index3D& index :
                 Grid(node_min_corner, node_max_corner)) {
              const Point3D center =
                  convert::indexToCenterPoint(index, min_cell_width);
              point_msg_vector.emplace_back(
                  convert::point3DToPoint32Msg(center));
            }
          }
        }
      };

  // Define a functor that allows us to skip unchanged map blocks
  auto process_block_if_changed = [this, &add_points_for_leaf_node](
                                      const Index3D& block_index,
                                      const auto& block) {
    const bool block_changed =
        last_run_timestamp_internal_ < block.getLastUpdatedStamp();
    if (!config_.only_publish_changed_blocks || block_changed) {
      block.forEachLeaf(block_index, add_points_for_leaf_node);
    }
  };

  // Iterate over the map
  const Timestamp start_time_internal = Time::now();
  if (const auto* hashed_wavelet_octree =
          dynamic_cast<const HashedWaveletOctree*>(occupancy_map_.get());
      hashed_wavelet_octree) {
    hashed_wavelet_octree->forEachBlock(process_block_if_changed);
  } else if (const auto* hashed_chunked_wavelet_octree =
                 dynamic_cast<const HashedChunkedWaveletOctree*>(
                     occupancy_map_.get());
             hashed_chunked_wavelet_octree) {
    hashed_chunked_wavelet_octree->forEachBlock(process_block_if_changed);
  } else {  // Fallback for non-hashed map types: simply process all leaves
    occupancy_map_->forEachLeaf(add_points_for_leaf_node);
  }
  last_run_timestamp_internal_ = start_time_internal;

  // Publish the msg
  sensor_msgs::PointCloud2 obstacle_pointcloud2_msg;
  sensor_msgs::convertPointCloudToPointCloud2(obstacle_pointcloud_msg,
                                              obstacle_pointcloud2_msg);
  pointcloud_pub_.publish(obstacle_pointcloud2_msg);
}
}  // namespace wavemap
