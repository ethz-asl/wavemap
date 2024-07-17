#include "wavemap_ros/utils/pointcloud_undistorter.h"

#include <wavemap/core/utils/profiler_interface.h>
#include <wavemap/core/utils/undistortion/pointcloud_undistortion.h>
#include <wavemap_ros_conversions/time_conversions.h>

namespace wavemap {
PointcloudUndistorter::Result PointcloudUndistorter::undistortPointcloud(
    undistortion::StampedPointcloud& stamped_pointcloud,
    PosedPointcloud<>& undistorted_pointcloud, const std::string& fixed_frame) {
  ProfilerZoneScoped;
  using undistortion::TimeAbsolute;

  // Get the time interval
  const TimeAbsolute start_time = stamped_pointcloud.getStartTime();
  const TimeAbsolute end_time = stamped_pointcloud.getEndTime();

  // Calculate the step size for the undistortion transform buffer
  const auto& points = stamped_pointcloud.getPoints();
  const int num_time_steps = num_interpolation_intervals_per_cloud_ + 1;
  const TimeAbsolute step_size =
      (points.back().time_offset - points.front().time_offset) /
      (num_interpolation_intervals_per_cloud_ - 1);
  const TimeAbsolute buffer_start_time = start_time - step_size;
  const TimeAbsolute buffer_end_time = end_time + step_size;

  // Make sure all transforms are available
  if (!transformer_->isTransformAvailable(
          fixed_frame, stamped_pointcloud.getSensorFrame(),
          convert::nanoSecondsToRosTime(buffer_start_time))) {
    return Result::kStartTimeNotInTfBuffer;
  }
  if (!transformer_->isTransformAvailable(
          fixed_frame, stamped_pointcloud.getSensorFrame(),
          convert::nanoSecondsToRosTime(buffer_end_time))) {
    return Result::kEndTimeNotInTfBuffer;
  }

  // Buffer the transforms
  undistortion::StampedPoseBuffer pose_buffer;
  pose_buffer.reserve(num_time_steps);
  for (int step_idx = 0; step_idx < num_time_steps; ++step_idx) {
    auto& stamped_pose = pose_buffer.emplace_back();
    stamped_pose.stamp = start_time + step_idx * step_size;
    if (!transformer_->lookupTransform(
            fixed_frame, stamped_pointcloud.getSensorFrame(),
            convert::nanoSecondsToRosTime(stamped_pose.stamp),
            stamped_pose.pose)) {
      ROS_WARN_STREAM("Failed to buffer intermediate pose at time "
                      << convert::nanoSecondsToRosTime(stamped_pose.stamp)
                      << ".");
      return Result::kIntermediateTimeNotInTfBuffer;
    }
  }

  // Apply motion undistortion and return the result
  undistorted_pointcloud =
      undistortion::compensate_motion(pose_buffer, stamped_pointcloud);
  return Result::kSuccess;
}
}  // namespace wavemap
