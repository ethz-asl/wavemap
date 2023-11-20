#ifndef WAVEMAP_ROS_OPERATIONS_IMPL_PUBLISH_MAP_OPERATION_INL_H_
#define WAVEMAP_ROS_OPERATIONS_IMPL_PUBLISH_MAP_OPERATION_INL_H_

#include <unordered_set>

#include <tracy/Tracy.hpp>
#include <wavemap/indexing/index_hashes.h>
#include <wavemap_msgs/Map.h>
#include <wavemap_ros_conversions/map_msg_conversions.h>

namespace wavemap {
template <typename HashedMapT>
void PublishMapOperation::publishHashedMap(const ros::Time& current_time,
                                           HashedMapT* hashed_map,
                                           bool republish_whole_map) {
  // Find the blocks that changed since the last publication time
  const Timestamp start_time_internal = Time::now();
  std::unordered_set<Index3D, Index3DHash> changed_blocks;
  for (const auto& [block_idx, block] : hashed_map->getBlocks()) {
    if (republish_whole_map ||
        last_run_timestamp_internal_ < block.getLastUpdatedStamp()) {
      changed_blocks.emplace(block_idx);
    }
  }
  last_run_timestamp_internal_ = start_time_internal;

  // Publish the changed blocks, 'max_num_blocks_per_msg' at a time
  while (!changed_blocks.empty()) {
    // Prepare the blocks to publish in the current iteration
    std::unordered_set<Index3D, Index3DHash> blocks_to_publish;
    int block_cnt = 0;
    for (const auto& block_idx : changed_blocks) {
      hashed_map->getBlock(block_idx).threshold();
      blocks_to_publish.insert(block_idx);
      if (config_.max_num_blocks_per_msg <= ++block_cnt) {
        break;
      }
    }

    // Serialize and publish the selected blocks
    wavemap_msgs::Map map_msg;
    map_msg.header.frame_id = world_frame_;
    map_msg.header.stamp = current_time;
    convert::mapToRosMsg(*hashed_map,
                         map_msg.hashed_wavelet_octree.emplace_back(),
                         blocks_to_publish, thread_pool_);
    {
      ZoneScopedN("publishMapRosMsg");
      map_pub_.publish(map_msg);
    }

    // Remove the published blocks from the publication queue
    auto last_published_block = changed_blocks.begin();
    std::advance(last_published_block, blocks_to_publish.size());
    changed_blocks.erase(changed_blocks.begin(), last_published_block);
  }
}
}  // namespace wavemap

#endif  // WAVEMAP_ROS_OPERATIONS_IMPL_PUBLISH_MAP_OPERATION_INL_H_
