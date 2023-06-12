#include "wavemap_ros/input_handler/pointcloud_undistorter.h"

#include <wavemap_ros_conversions/time_conversions.h>

namespace wavemap {
PointcloudUndistorter::Result PointcloudUndistorter::undistortPointcloud(
    StampedPointcloud& stamped_pointcloud,
    PosedPointcloud<>& undistorted_pointcloud) {
  // Get the time interval
  const uint64_t start_time = stamped_pointcloud.getStartTime();
  const uint64_t end_time = stamped_pointcloud.getEndTime();

  // Calculate the step size for the undistortion transform buffer
  const auto& points = stamped_pointcloud.points;
  constexpr int kNumTimeIntervals = 400;
  constexpr int kNumTimeSteps = kNumTimeIntervals + 1;
  const uint64_t step_size =
      (points.back().time_offset - points.front().time_offset) /
      (kNumTimeIntervals - 1);
  const uint64_t buffer_start_time = start_time - step_size;
  const uint64_t buffer_end_time = end_time + step_size;

  // Make sure all transforms are available
  if (!transformer_->isTransformAvailable(
          stamped_pointcloud.world_frame, stamped_pointcloud.sensor_frame,
          convert::nanoSecondsToRosTime(buffer_end_time))) {
    return Result::kEndTimeNotInTfBuffer;
  }
  if (!transformer_->isTransformAvailable(
          stamped_pointcloud.world_frame, stamped_pointcloud.sensor_frame,
          convert::nanoSecondsToRosTime(buffer_start_time))) {
    return Result::kStartTimeNotInTfBuffer;
  }

  // Buffer the transforms
  std::vector<std::pair<uint64_t, Transformation3D>> timed_poses;
  timed_poses.reserve(kNumTimeSteps);
  for (unsigned int step_idx = 0u; step_idx < kNumTimeSteps; ++step_idx) {
    auto& timed_pose = timed_poses.emplace_back();
    timed_pose.first = start_time + step_idx * step_size;
    if (!transformer_->lookupTransform(
            stamped_pointcloud.world_frame, stamped_pointcloud.sensor_frame,
            convert::nanoSecondsToRosTime(timed_pose.first),
            timed_pose.second)) {
      ROS_WARN_STREAM("Failed to buffer intermediate pose at time "
                      << convert::nanoSecondsToRosTime(timed_pose.first)
                      << ".");
      return Result::kIntermediateTimeNotInTfBuffer;
    }
  }

  // Motion undistort the points
  const auto num_rays = static_cast<int>(points.size());
  Eigen::Matrix<FloatingPoint, 3, Eigen::Dynamic> t_W_points;
  t_W_points.resize(3, num_rays);
  int l_idx = 0u;
  uint64_t prev_time = -1u;
  Transformation3D T_WCi;
  for (int idx = 0u; idx < num_rays; ++idx) {
    const auto& point = points[idx];
    const Point3D& Ci_p = point.point;

    const uint64_t time = stamped_pointcloud.time_base + point.time_offset;
    if (time != prev_time) {
      prev_time = time;
      while (timed_poses[l_idx + 1].first < time && l_idx + 2 < kNumTimeSteps) {
        ++l_idx;
      }
      CHECK_LT(l_idx + 1, timed_poses.size());
      const uint64_t time_l = timed_poses[l_idx].first;
      const uint64_t time_u = timed_poses[l_idx + 1].first;
      const Transformation3D& T_WCl = timed_poses[l_idx].second;
      const Transformation3D& T_WCu = timed_poses[l_idx + 1].second;
      FloatingPoint a = static_cast<FloatingPoint>((time - time_l)) /
                        static_cast<FloatingPoint>((time_u - time_l));
      CHECK_GE(a, 0.f);
      CHECK_LE(a, 1.f);
      T_WCi = interpolateComponentwise(T_WCl, T_WCu, a);
    }

    t_W_points.col(idx) = T_WCi * Ci_p;
  }
  const Transformation3D T_WCmid = timed_poses[kNumTimeIntervals / 2].second;
  const PosedPointcloud<> posed_pointcloud(
      T_WCmid, T_WCmid.inverse().transformVectorized(t_W_points));
}
}  // namespace wavemap
