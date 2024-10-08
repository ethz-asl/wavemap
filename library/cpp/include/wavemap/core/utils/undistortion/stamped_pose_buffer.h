#ifndef WAVEMAP_CORE_UTILS_UNDISTORTION_STAMPED_POSE_BUFFER_H_
#define WAVEMAP_CORE_UTILS_UNDISTORTION_STAMPED_POSE_BUFFER_H_

#include <vector>

#include "wavemap/core/utils/undistortion/timestamps.h"

namespace wavemap::undistortion {
struct StampedPose {
  TimeAbsolute stamp;
  Transformation3D pose;
};
using StampedPoseBuffer = std::vector<StampedPose>;
}  // namespace wavemap::undistortion

#endif  // WAVEMAP_CORE_UTILS_UNDISTORTION_STAMPED_POSE_BUFFER_H_
