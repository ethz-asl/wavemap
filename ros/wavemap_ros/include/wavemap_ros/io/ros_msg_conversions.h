#ifndef WAVEMAP_ROS_IO_ROS_MSG_CONVERSIONS_H_
#define WAVEMAP_ROS_IO_ROS_MSG_CONVERSIONS_H_

#include <algorithm>
#include <stack>
#include <string>
#include <utility>

#include <wavemap/data_structure/volumetric/hashed_chunked_wavelet_octree.h>
#include <wavemap/data_structure/volumetric/hashed_wavelet_octree.h>
#include <wavemap/data_structure/volumetric/volumetric_octree.h>
#include <wavemap/data_structure/volumetric/wavelet_octree.h>
#include <wavemap_msgs/Map.h>

namespace wavemap {
wavemap_msgs::Map mapToRosMsg(const VolumetricOctree& map,
                              const std::string& frame_id);

wavemap_msgs::Map mapToRosMsg(const WaveletOctree& map,
                              const std::string& frame_id);

wavemap_msgs::Map mapToRosMsg(
    const HashedWaveletOctree& map, const std::string& frame_id,
    std::optional<FloatingPoint> ignore_blocks_older_than = std::nullopt);

wavemap_msgs::Map mapToRosMsg(
    const HashedChunkedWaveletOctree& map, const std::string& frame_id,
    std::optional<FloatingPoint> ignore_blocks_older_than = std::nullopt);
}  // namespace wavemap

#endif  // WAVEMAP_ROS_IO_ROS_MSG_CONVERSIONS_H_
