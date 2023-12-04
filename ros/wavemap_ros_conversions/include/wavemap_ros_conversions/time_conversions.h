#ifndef WAVEMAP_ROS_CONVERSIONS_TIME_CONVERSIONS_H_
#define WAVEMAP_ROS_CONVERSIONS_TIME_CONVERSIONS_H_

#include <rclcpp/rclcpp.hpp>

namespace wavemap::convert {
inline rclcpp::Time nanoSecondsToRosTime(uint64_t nsec) {
  constexpr uint64_t kSecToNsec = 1000000000ull;
  return {static_cast<uint32_t>(nsec / kSecToNsec),
          static_cast<uint32_t>(nsec % kSecToNsec)};
}
inline double nanoSecondsToSeconds(uint64_t nsec) {
  constexpr double kNsecToSec = 1e-9;
  return static_cast<double>(nsec) * kNsecToSec;
}

inline uint64_t rosTimeToNanoSeconds(const rclcpp::Time& time) {
  constexpr uint64_t kSecToNsec = 1000000000ull;
  return time.seconds() * kSecToNsec + time.nanoseconds();
}
}  // namespace wavemap::convert

#endif  // WAVEMAP_ROS_CONVERSIONS_TIME_CONVERSIONS_H_
