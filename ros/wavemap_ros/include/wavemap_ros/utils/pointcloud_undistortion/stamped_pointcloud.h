#ifndef WAVEMAP_ROS_UTILS_POINTCLOUD_UNDISTORTION_STAMPED_POINTCLOUD_H_
#define WAVEMAP_ROS_UTILS_POINTCLOUD_UNDISTORTION_STAMPED_POINTCLOUD_H_

#include <algorithm>
#include <string>
#include <utility>
#include <vector>

#include <wavemap/common.h>

namespace wavemap {
struct StampedPoint {
  Point3D position;
  uint32_t time_offset;

  StampedPoint(FloatingPoint x, FloatingPoint y, FloatingPoint z,
               uint64_t time_offset)
      : position(x, y, z), time_offset(time_offset) {}

  std::string toStr() const {
    return "[" + std::to_string(position[0]) + ", " +
           std::to_string(position[1]) + ", " + std::to_string(position[2]) +
           ", " + std::to_string(time_offset) + "]";
  }
};

class StampedPointcloud {
 public:
  StampedPointcloud(uint64_t time_base, std::string sensor_frame,
                    size_t expected_num_points = 0)
      : sensor_frame_(std::move(sensor_frame)), time_base_(time_base) {
    points_.reserve(expected_num_points);
  }

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
  uint64_t& getTimeBase() { return time_base_; }
  const uint64_t& getTimeBase() const { return time_base_; }

  uint64_t getStartTime() {
    sort();
    return time_base_ + points_.front().time_offset;
  }
  uint64_t getMedianTime() {
    sort();
    return time_base_ + points_[points_.size() / 2].time_offset;
  }
  uint64_t getEndTime() {
    sort();
    return time_base_ + points_.back().time_offset;
  }

  // Point related getters
  std::vector<StampedPoint>& getPoints() { return points_; }
  const std::vector<StampedPoint>& getPoints() const { return points_; }

  StampedPoint& operator[](size_t point_idx) { return points_[point_idx]; }
  const StampedPoint& operator[](size_t point_idx) const {
    return points_[point_idx];
  }

  // Sort points by time
  void sort() {
    if (!is_sorted) {
      std::sort(points_.begin(), points_.end(),
                [](const auto& lhs, const auto& rhs) {
                  return lhs.time_offset < rhs.time_offset;
                });
      is_sorted = true;
    }
  }

 private:
  std::string sensor_frame_;
  uint64_t time_base_;
  std::vector<StampedPoint> points_;

  bool is_sorted = false;
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_UTILS_POINTCLOUD_UNDISTORTION_STAMPED_POINTCLOUD_H_
