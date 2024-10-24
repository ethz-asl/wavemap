#include "wavemap/core/utils/undistortion/pointcloud_undistortion.h"

#include <utility>

namespace wavemap {
PosedPointcloud<> undistortion::compensate_motion(
    const StampedPoseBuffer& pose_buffer,
    undistortion::StampedPointcloud& stamped_pointcloud) {
  // Check that timestamps of all points fall within range of pose buffer
  {
    const auto& buffer_start_time = pose_buffer.front().stamp;
    const auto& buffer_end_time = pose_buffer.back().stamp;
    CHECK_GE(stamped_pointcloud.getStartTime(), buffer_start_time);
    CHECK_LE(stamped_pointcloud.getEndTime(), buffer_end_time);
  }

  // Motion undistort
  // NOTE: The undistortion is done by transforming the points into a fixed
  //       (inertial) frame using the sensor's pose at each point's timestamp.
  const auto& points = stamped_pointcloud.getPoints();
  const auto num_points = static_cast<int>(points.size());
  const auto num_time_steps = static_cast<int>(pose_buffer.size());
  Eigen::Matrix<FloatingPoint, 3, Eigen::Dynamic> t_W_points;
  t_W_points.setZero(3, num_points);
  TimeAbsolute previous_point_time = -1u;
  int pose_left_idx = 0u;
  Transformation3D T_WCi;
  Transformation3D T_WCmedian;
  for (int idx = 0u; idx < num_points; ++idx) {
    const auto& point = points[idx];
    const Point3D& Ci_p = point.position;

    // Get the sensor pose at the current point's time stamp
    const TimeAbsolute time =
        stamped_pointcloud.getTimeBase() + point.time_offset;
    if (time != previous_point_time) {
      previous_point_time = time;
      while (pose_buffer[pose_left_idx + 1].stamp < time &&
             pose_left_idx + 2 < num_time_steps) {
        ++pose_left_idx;
      }
      CHECK_LT(pose_left_idx + 1, pose_buffer.size());
      const TimeAbsolute time_left = pose_buffer[pose_left_idx].stamp;
      const TimeAbsolute time_right = pose_buffer[pose_left_idx + 1].stamp;
      const Transformation3D& T_WCleft = pose_buffer[pose_left_idx].pose;
      const Transformation3D& T_WCright = pose_buffer[pose_left_idx + 1].pose;
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
  return PosedPointcloud<>{T_WCmedian, std::move(t_C_points)};
}
}  // namespace wavemap
