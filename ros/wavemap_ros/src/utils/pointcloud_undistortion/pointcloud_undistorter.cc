#include "wavemap_ros/utils/pointcloud_undistortion/pointcloud_undistorter.h"

#include <tracy/Tracy.hpp>
#include <wavemap_ros_conversions/time_conversions.h>

namespace wavemap {
PointcloudUndistorter::Result PointcloudUndistorter::undistortPointcloud(
    StampedPointcloud& stamped_pointcloud,
    PosedPointcloud<>& undistorted_pointcloud, const std::string& fixed_frame) {
  ZoneScoped;
  // Get the time interval
  const uint64_t start_time = stamped_pointcloud.getStartTime();
  const uint64_t end_time = stamped_pointcloud.getEndTime();

  // Calculate the step size for the undistortion transform buffer
  const auto& points = stamped_pointcloud.getPoints();
  const int num_time_steps = num_interpolation_intervals_per_cloud_ + 1;
  const uint64_t step_size =
      (points.back().time_offset - points.front().time_offset) /
      (num_interpolation_intervals_per_cloud_ - 1);
  const uint64_t buffer_start_time = start_time - step_size;
  const uint64_t buffer_end_time = end_time + step_size;

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
  std::vector<std::pair<uint64_t, Transformation3D>> timed_poses;
  timed_poses.reserve(num_time_steps);
  for (int step_idx = 0; step_idx < num_time_steps; ++step_idx) {
    auto& timed_pose = timed_poses.emplace_back();
    timed_pose.first = start_time + step_idx * step_size;
    if (!transformer_->lookupTransform(
            fixed_frame, stamped_pointcloud.getSensorFrame(),
            convert::nanoSecondsToRosTime(timed_pose.first),
            timed_pose.second)) {
      ROS_WARN_STREAM("Failed to buffer intermediate pose at time "
                      << convert::nanoSecondsToRosTime(timed_pose.first)
                      << ".");
      return Result::kIntermediateTimeNotInTfBuffer;
    }
  }

  // Motion undistort
  // NOTE: The undistortion is done by transforming the points into a fixed
  //       (inertial) frame using the sensor's pose at each point's timestamp.
  const auto num_points = static_cast<int>(points.size());
  Eigen::Matrix<FloatingPoint, 3, Eigen::Dynamic> t_W_points;
  t_W_points.setZero(3, num_points);
  uint64_t previous_point_time = -1u;
  int pose_left_idx = 0u;
  Transformation3D T_WCi;
  Transformation3D T_WCmedian;
  for (int idx = 0u; idx < num_points; ++idx) {
    const auto& point = points[idx];
    const Point3D& Ci_p = point.position;

    // Get the sensor pose at the current point's time stamp
    const uint64_t time = stamped_pointcloud.getTimeBase() + point.time_offset;
    if (time != previous_point_time) {
      previous_point_time = time;
      while (timed_poses[pose_left_idx + 1].first < time &&
             pose_left_idx + 2 < num_time_steps) {
        ++pose_left_idx;
      }
      CHECK_LT(pose_left_idx + 1, timed_poses.size());
      const uint64_t time_left = timed_poses[pose_left_idx].first;
      const uint64_t time_right = timed_poses[pose_left_idx + 1].first;
      const Transformation3D& T_WCleft = timed_poses[pose_left_idx].second;
      const Transformation3D& T_WCright = timed_poses[pose_left_idx + 1].second;
      FloatingPoint a = static_cast<FloatingPoint>((time - time_left)) /
                        static_cast<FloatingPoint>((time_right - time_left));
      DCHECK_GE(a, 0.f);
      DCHECK_LE(a, 1.f);
      T_WCi = interpolateComponentwise(T_WCleft, T_WCright, a);
    }

    // Transform the current point into the fixed frame
    t_W_points.col(idx) = T_WCi * Ci_p;

    // Store the sensor's pose at the median timestamp
    if (idx == num_points / 2) {
      T_WCmedian = T_WCi;
    }
  }

  // Transform the undistorted pointcloud back into sensor frame,
  // as needed by the integrators
  auto t_C_points = T_WCmedian.inverse().transformVectorized(t_W_points);

  // Return the result
  undistorted_pointcloud = PosedPointcloud<>(T_WCmedian, std::move(t_C_points));
  return Result::kSuccess;
}
}  // namespace wavemap
