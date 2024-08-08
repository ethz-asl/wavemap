#include "wavemap/core/utils/undistortion/stamped_pointcloud.h"

namespace wavemap::undistortion {
std::string StampedPoint::toStr() const {
  return "[" + std::to_string(position[0]) + ", " +
         std::to_string(position[1]) + ", " + std::to_string(position[2]) +
         ", " + std::to_string(time_offset) + "]";
}

StampedPointcloud::StampedPointcloud(TimeAbsolute time_base,
                                     std::string sensor_frame,
                                     size_t expected_num_points)
    : sensor_frame_(std::move(sensor_frame)), time_base_(time_base) {
  points_.reserve(expected_num_points);
}

TimeAbsolute StampedPointcloud::getStartTime() {
  sort();
  return time_base_ + points_.front().time_offset;
}

TimeAbsolute StampedPointcloud::getMedianTime() {
  sort();
  return time_base_ + points_[points_.size() / 2].time_offset;
}

TimeAbsolute StampedPointcloud::getEndTime() {
  sort();
  return time_base_ + points_.back().time_offset;
}

void StampedPointcloud::sort() {
  if (!is_sorted) {
    std::sort(points_.begin(), points_.end(),
              [](const auto& lhs, const auto& rhs) {
                return lhs.time_offset < rhs.time_offset;
              });
    is_sorted = true;
  }
}
}  // namespace wavemap::undistortion
