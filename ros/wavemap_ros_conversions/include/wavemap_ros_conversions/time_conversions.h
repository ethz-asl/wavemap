#ifndef WAVEMAP_ROS_CONVERSIONS_TIME_CONVERSIONS_H_
#define WAVEMAP_ROS_CONVERSIONS_TIME_CONVERSIONS_H_

#include <ros/ros.h>

namespace wavemap::convert {
inline ros::Time nanoSecondsToRosTime(uint64_t nsec) {
  uint64_t kSecToNsec = 1000000000ull;
  return {static_cast<uint32_t>(nsec / kSecToNsec),
          static_cast<uint32_t>(nsec % kSecToNsec)};
}
}  // namespace wavemap::convert

#endif  // WAVEMAP_ROS_CONVERSIONS_TIME_CONVERSIONS_H_
