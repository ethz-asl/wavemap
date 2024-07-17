#ifndef WAVEMAP_CORE_UTILS_UNDISTORTION_POINTCLOUD_UNDISTORTION_H_
#define WAVEMAP_CORE_UTILS_UNDISTORTION_POINTCLOUD_UNDISTORTION_H_

#include "wavemap/core/data_structure/pointcloud.h"
#include "wavemap/core/utils/undistortion/stamped_pointcloud.h"
#include "wavemap/core/utils/undistortion/stamped_pose_buffer.h"

namespace wavemap::undistortion {
PosedPointcloud<> compensate_motion(const StampedPoseBuffer& pose_buffer,
                                    StampedPointcloud& stamped_pointcloud);
}  // namespace wavemap::undistortion

#endif  // WAVEMAP_CORE_UTILS_UNDISTORTION_POINTCLOUD_UNDISTORTION_H_
