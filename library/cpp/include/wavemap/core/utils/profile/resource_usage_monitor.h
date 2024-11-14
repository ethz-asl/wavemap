#ifndef WAVEMAP_CORE_UTILS_PROFILE_RESOURCE_USAGE_MONITOR_H_
#define WAVEMAP_CORE_UTILS_PROFILE_RESOURCE_USAGE_MONITOR_H_

#include <ctime>
#include <fstream>
#include <sstream>

#include "wavemap/core/common.h"
#include "wavemap/core/utils/time/time.h"

namespace wavemap {
class ResourceMonitor {
 public:
  void start();
  void stop();

  double getLastEpisodeCpuTime() const {
    return time::to_seconds<double>(last_episode_cpu_duration_);
  }
  double getLastEpisodeWallTime() const {
    return time::to_seconds<double>(last_episode_wall_duration_);
  }
  static std::optional<size_t> getCurrentRamUsageInKB();

 private:
  bool running_ = false;

  timespec episode_start_cpu_time_{};
  timespec episode_start_wall_time_{};

  Duration last_episode_cpu_duration_{};
  Duration last_episode_wall_duration_{};

  Duration total_cpu_duration_{};
  Duration total_wall_duration_{};

  static Duration computeDuration(const timespec& start, const timespec& stop);
};
}  // namespace wavemap

#endif  // WAVEMAP_CORE_UTILS_PROFILE_RESOURCE_USAGE_MONITOR_H_
