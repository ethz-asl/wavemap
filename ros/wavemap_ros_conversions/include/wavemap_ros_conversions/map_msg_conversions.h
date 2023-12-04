#ifndef WAVEMAP_ROS_CONVERSIONS_MAP_MSG_CONVERSIONS_H_
#define WAVEMAP_ROS_CONVERSIONS_MAP_MSG_CONVERSIONS_H_

#include <algorithm>
#include <memory>
#include <stack>
#include <string>
#include <unordered_set>
#include <utility>

#include <rclcpp/time.hpp>
#include <wavemap/data_structure/volumetric/hashed_chunked_wavelet_octree.h>
#include <wavemap/data_structure/volumetric/hashed_wavelet_octree.h>
#include <wavemap/data_structure/volumetric/volumetric_octree.h>
#include <wavemap/data_structure/volumetric/wavelet_octree.h>
#include <wavemap/utils/thread_pool.h>
#include <wavemap_msgs/msg/map.hpp>

namespace wavemap::convert {
bool mapToRosMsg(const VolumetricDataStructureBase& map,
                 const std::string& frame_id, const rclcpp::Time& stamp,
                 wavemap_msgs::msg::Map& msg);
bool rosMsgToMap(const wavemap_msgs::msg::Map& msg,
                 VolumetricDataStructureBase::Ptr& map);

void mapToRosMsg(const WaveletOctree& map, wavemap_msgs::msg::WaveletOctree& msg);
void rosMsgToMap(const wavemap_msgs::msg::WaveletOctree& msg,
                 WaveletOctree::Ptr& map);

void mapToRosMsg(const HashedWaveletOctree& map,
                 wavemap_msgs::msg::HashedWaveletOctree& msg,
                 std::optional<std::unordered_set<Index3D, Index3DHash>>
                     include_blocks = std::nullopt,
                 std::shared_ptr<ThreadPool> thread_pool = nullptr);
void blockToRosMsg(const HashedWaveletOctree::BlockIndex& block_index,
                   const HashedWaveletOctree::Block& block,
                   FloatingPoint min_log_odds, FloatingPoint max_log_odds,
                   wavemap_msgs::msg::HashedWaveletOctreeBlock& msg);
void rosMsgToMap(const wavemap_msgs::msg::HashedWaveletOctree& msg,
                 HashedWaveletOctree::Ptr& map);

void mapToRosMsg(const HashedChunkedWaveletOctree& map,
                 wavemap_msgs::msg::HashedWaveletOctree& msg,
                 std::optional<std::unordered_set<Index3D, Index3DHash>>
                     include_blocks = std::nullopt,
                 std::shared_ptr<ThreadPool> thread_pool = nullptr);
void blockToRosMsg(const HashedChunkedWaveletOctree::BlockIndex& block_index,
                   const HashedChunkedWaveletOctree::Block& block,
                   FloatingPoint min_log_odds, FloatingPoint max_log_odds,
                   IndexElement tree_height,
                   wavemap_msgs::msg::HashedWaveletOctreeBlock& msg);
}  // namespace wavemap::convert

#endif  // WAVEMAP_ROS_CONVERSIONS_MAP_MSG_CONVERSIONS_H_
