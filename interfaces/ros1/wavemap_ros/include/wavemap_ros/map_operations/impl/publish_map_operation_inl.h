#ifndef WAVEMAP_ROS_MAP_OPERATIONS_IMPL_PUBLISH_MAP_OPERATION_INL_H_
#define WAVEMAP_ROS_MAP_OPERATIONS_IMPL_PUBLISH_MAP_OPERATION_INL_H_

#include <unordered_set>

#include <wavemap/core/indexing/index_hashes.h>
#include <wavemap/core/utils/profiler_interface.h>
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
  hashed_map->forEachBlock([&changed_blocks, republish_whole_map,
                            last_run_stamp = last_run_timestamp_internal_](
                               const Index3D& block_index, const auto& block) {
    if (republish_whole_map || last_run_stamp < block.getLastUpdatedStamp()) {
      changed_blocks.emplace(block_index);
    }
  });
  last_run_timestamp_internal_ = start_time_internal;

  // Publish the changed blocks, 'max_num_blocks_per_msg' at a time
  while (!changed_blocks.empty()) {
    // Prepare the blocks to publish in the current iteration
    std::unordered_set<Index3D, Index3DHash> blocks_to_publish;
    int block_cnt = 0;
    for (const auto& block_index : changed_blocks) {
      if (auto* block = hashed_map->getBlock(block_index); block) {
        block->threshold();
        blocks_to_publish.insert(block_index);
        if (config_.max_num_blocks_per_msg <= ++block_cnt) {
          break;
        }
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
      ProfilerZoneScopedN("publishMapRosMsg");
      map_pub_.publish(map_msg);
    }

    // Remove the published blocks from the publication queue
    auto last_published_block = changed_blocks.begin();
    std::advance(last_published_block, blocks_to_publish.size());
    changed_blocks.erase(changed_blocks.begin(), last_published_block);
  }
}
}  // namespace wavemap

#endif  // WAVEMAP_ROS_MAP_OPERATIONS_IMPL_PUBLISH_MAP_OPERATION_INL_H_
