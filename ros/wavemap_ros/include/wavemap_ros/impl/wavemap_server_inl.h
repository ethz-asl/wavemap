#ifndef WAVEMAP_ROS_IMPL_WAVEMAP_SERVER_INL_H_
#define WAVEMAP_ROS_IMPL_WAVEMAP_SERVER_INL_H_

#include <functional>
#include <unordered_set>
#include <utility>
#include <vector>

#include <tracy/Tracy.hpp>
#include <wavemap_msgs/Map.h>
#include <wavemap_ros_conversions/map_msg_conversions.h>

namespace wavemap {
template <typename HashedMapT>
void WavemapServer::publishHashedMap(HashedMapT* hashed_map,
                                     bool republish_whole_map) {
  // Find the blocks that changed since the last publication time
  const Timestamp start_time = Time::now();
  std::unordered_set<Index3D, Index3DHash> changed_blocks;
  for (const auto& [block_idx, block] : hashed_map->getBlocks()) {
    if (republish_whole_map ||
        last_map_pub_time_ < block.getLastUpdatedStamp()) {
      changed_blocks.emplace(block_idx);
    }
  }
  last_map_pub_time_ = start_time;

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
    map_msg.header.frame_id = config_.world_frame;
    map_msg.header.stamp = ros::Time::now();
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

#endif  // WAVEMAP_ROS_IMPL_WAVEMAP_SERVER_INL_H_
