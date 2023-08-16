#ifndef WAVEMAP_UTILS_TIME_H_
#define WAVEMAP_UTILS_TIME_H_

#include <chrono>

namespace wavemap {
using Time = std::chrono::steady_clock;
using Timestamp = std::chrono::time_point<Time>;
using Duration = Timestamp::duration;

template <typename T>
T to_seconds(Duration duration) {
  return std::chrono::duration_cast<std::chrono::duration<T>>(duration).count();
}
}  // namespace wavemap

#endif  // WAVEMAP_UTILS_TIME_H_
