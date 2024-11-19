#include "wavemap/core/utils/profile/resource_monitor.h"

#include <string>

namespace wavemap {
void ResourceMonitor::start() {
  if (running_) {
    LOG(WARNING) << "Tried to start timer that was already running.";
    return;
  }

  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &episode_start_cpu_time_);
  clock_gettime(CLOCK_MONOTONIC, &episode_start_wall_time_);
  running_ = true;
}

void ResourceMonitor::stop() {
  if (!running_) {
    LOG(WARNING) << "Tried to stop timer that was not running.";
    return;
  }

  timespec episode_stop_cpu_time{};
  timespec episode_stop_wall_time{};
  clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &episode_stop_cpu_time);
  clock_gettime(CLOCK_MONOTONIC, &episode_stop_wall_time);
  running_ = false;

  last_episode_cpu_duration_ =
      computeDuration(episode_start_cpu_time_, episode_stop_cpu_time);
  last_episode_wall_duration_ =
      computeDuration(episode_start_wall_time_, episode_stop_wall_time);
  total_cpu_duration_ += last_episode_cpu_duration_;
  total_wall_duration_ += last_episode_wall_duration_;
}

std::optional<size_t> ResourceMonitor::getCurrentRamUsageInKB() {
  std::ifstream file("/proc/self/status");
  std::string line;
  while (std::getline(file, line)) {
    if (line.rfind("VmRSS", 0) == 0) {
      break;
    }
  }
  if (file.eof()) {
    LOG(ERROR) << "Failed to read RAM usage. Could not find VmRSS entry in "
                  "\"/proc/self/status\".";
    return std::nullopt;
  }

  std::string quantity;
  size_t ram_usage;
  std::string unit;
  if (std::istringstream iss(line); !(iss >> quantity >> ram_usage >> unit)) {
    LOG(ERROR) << "Failed to read RAM usage. Could not parse VmRSS line "
                  "in \"/proc/self/status\".";
    return std::nullopt;
  }

  if (unit != "kB") {
    LOG(ERROR) << "Failed to read RAM usage. Expected VmRSS entry in "
                  "\"/proc/self/status\" to be specified in kB, got "
               << unit << ".";
    return std::nullopt;
  }

  return ram_usage;
}

Duration ResourceMonitor::computeDuration(const timespec& start,
                                          const timespec& stop) {
  return std::chrono::seconds(stop.tv_sec - start.tv_sec) +
         std::chrono::nanoseconds(stop.tv_nsec - start.tv_nsec);
}
}  // namespace wavemap
