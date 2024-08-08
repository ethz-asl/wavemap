#ifndef WAVEMAP_CORE_UTILS_UNDISTORTION_STAMPED_POINTCLOUD_H_
#define WAVEMAP_CORE_UTILS_UNDISTORTION_STAMPED_POINTCLOUD_H_

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include "wavemap/core/common.h"
#include "wavemap/core/utils/undistortion/timestamps.h"

namespace wavemap::undistortion {
struct StampedPoint {
  Point3D position;
  TimeOffset time_offset;

  StampedPoint(FloatingPoint x, FloatingPoint y, FloatingPoint z,
               TimeOffset time_offset)
      : position(x, y, z), time_offset(time_offset) {}

  std::string toStr() const;
};

class StampedPointcloud {
 public:
  StampedPointcloud(TimeAbsolute time_base, std::string sensor_frame,
                    size_t expected_num_points = 0);

  // Add points
  template <typename... Args>
  auto emplace(Args... args) {
    is_sorted = false;
    return points_.emplace_back(std::forward<Args>(args)...);
  }

  // Frame related getters
  std::string& getSensorFrame() { return sensor_frame_; }
  const std::string& getSensorFrame() const { return sensor_frame_; }

  // Time related getters
  TimeAbsolute& getTimeBase() { return time_base_; }
  const TimeAbsolute& getTimeBase() const { return time_base_; }

  TimeAbsolute getStartTime();
  TimeAbsolute getMedianTime();
  TimeAbsolute getEndTime();

  // Point related getters
  std::vector<StampedPoint>& getPoints() { return points_; }
  const std::vector<StampedPoint>& getPoints() const { return points_; }

  StampedPoint& operator[](size_t point_idx) { return points_[point_idx]; }
  const StampedPoint& operator[](size_t point_idx) const {
    return points_[point_idx];
  }

  // Sort points by time
  void sort();

 private:
  std::string sensor_frame_;
  TimeAbsolute time_base_;
  std::vector<StampedPoint> points_;

  bool is_sorted = false;
};
}  // namespace wavemap::undistortion

#endif  // WAVEMAP_CORE_UTILS_UNDISTORTION_STAMPED_POINTCLOUD_H_
