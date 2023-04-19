#ifndef WAVEMAP_ROS_CONVERSIONS_MAP_MSG_CONVERSIONS_H_
#define WAVEMAP_ROS_CONVERSIONS_MAP_MSG_CONVERSIONS_H_

#include <algorithm>
#include <stack>
#include <string>
#include <utility>

#include <ros/time.h>
#include <wavemap/data_structure/volumetric/hashed_chunked_wavelet_octree.h>
#include <wavemap/data_structure/volumetric/hashed_wavelet_octree.h>
#include <wavemap/data_structure/volumetric/volumetric_octree.h>
#include <wavemap/data_structure/volumetric/wavelet_octree.h>
#include <wavemap_msgs/Map.h>

namespace wavemap::convert {
wavemap_msgs::Map mapToRosMsg(const VolumetricDataStructureBase::ConstPtr& map,
                              const std::string& frame_id,
                              const ros::Time& stamp,
                              FloatingPoint ignore_blocks_older_than = -1.f);

wavemap_msgs::Map mapToRosMsg(const VolumetricOctree& map,
                              const std::string& frame_id,
                              const ros::Time& stamp);

wavemap_msgs::Map mapToRosMsg(const WaveletOctree& map,
                              const std::string& frame_id,
                              const ros::Time& stamp);

wavemap_msgs::Map mapToRosMsg(const HashedWaveletOctree& map,
                              const std::string& frame_id,
                              const ros::Time& stamp,
                              FloatingPoint ignore_blocks_older_than = -1.f);

wavemap_msgs::Map mapToRosMsg(const HashedChunkedWaveletOctree& map,
                              const std::string& frame_id,
                              const ros::Time& stamp,
                              FloatingPoint ignore_blocks_older_than = -1.f);

void rosMsgToMap(const wavemap_msgs::Map& map_msg,
                 VolumetricDataStructureBase::Ptr& map);

void rosMsgToMap(const wavemap_msgs::Map& map_msg, VolumetricOctree::Ptr& map);

void rosMsgToMap(const wavemap_msgs::Map& map_msg, WaveletOctree::Ptr& map);

void rosMsgToMap(const wavemap_msgs::Map& map_msg,
                 HashedWaveletOctree::Ptr& map);
}  // namespace wavemap::convert

#endif  // WAVEMAP_ROS_CONVERSIONS_MAP_MSG_CONVERSIONS_H_
