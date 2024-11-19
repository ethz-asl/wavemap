#ifndef WAVEMAP_CORE_UTILS_PROFILE_RESOURCE_MONITOR_H_
#define WAVEMAP_CORE_UTILS_PROFILE_RESOURCE_MONITOR_H_

#include <ctime>
#include <fstream>
#include <sstream>

#include "wavemap/core/common.h"
#include "wavemap/core/utils/time/time.h"

namespace wavemap {
/**
 * @brief Monitors system resource usage, including CPU time, wall time,
 *        and RAM usage.
 *
 * The `ResourceMonitor` class tracks CPU and wall clock time over timed
 * episodes, much like wavemap's Stopwatch class. It also provides functionality
 * to retrieve the total RAM usage of the current process.
 */
class ResourceMonitor {
 public:
  /**
   * @brief Starts a new CPU and wall time monitoring episode.
   *
   * Records the CPU and wall clock start times for the current episode.
   * If monitoring is already running, calling `start()` has no effect.
   */
  void start();

  /**
   * @brief Stops timing the current episode.
   *
   * Records the end CPU and wall clock times for the current episode,
   * updating the last episode duration and total accumulated duration.
   * If monitoring is not running, calling `stop()` has no effect.
   */
  void stop();

  /**
   * @brief Checks if the stopwatch is currently running.
   *
   * @return `true` if the stopwatch is running, `false` otherwise.
   */
  bool isRunning() const { return running_; }

  /**
   * @brief Gets the CPU time duration of the last episode.
   *
   * @return The CPU time duration of the last episode in seconds.
   *
   * The value represents the CPU time elapsed between the most recent
   * `start()` and `stop()` calls. If no episode has been monitored, this
   * returns 0.
   */
  double getLastEpisodeCpuTime() const {
    return time::to_seconds<double>(last_episode_cpu_duration_);
  }

  /**
   * @brief Gets the wall clock time duration of the last episode.
   *
   * @return The wall clock time duration of the last episode in seconds.
   *
   * The value represents the real-world time elapsed between the most recent
   * `start()` and `stop()` calls. If no episode has been monitored, this
   * returns 0.
   */
  double getLastEpisodeWallTime() const {
    return time::to_seconds<double>(last_episode_wall_duration_);
  }

  /**
   * @brief Gets the total accumulated CPU time of all episodes.
   *
   * @return The total CPU time in seconds.
   *
   * The value represents the sum of the CPU times of all episodes that have
   * been timed since the creation of the resource monitor or since it was last
   * reset.
   */
  double getTotalCpuTime() const {
    return time::to_seconds<double>(total_cpu_duration_);
  }

  /**
   * @brief Gets the total accumulated wall clock time of all episodes.
   *
   * @return The total wall time in seconds.
   *
   * The value represents the sum of the wall times of all episodes that have
   * been timed since the creation of the resource monitor or since it was last
   * reset.
   */
  double getTotalWallTime() const {
    return time::to_seconds<double>(total_wall_duration_);
  }

  /**
   * @brief Gets the current RAM usage of the application.
   *
   * @return The current RAM usage in kilobytes, or `std::nullopt` (an empty
   *         optional) if retrieving RAM usage is not supported on the given
   *         platform.
   */
  static std::optional<size_t> getCurrentRamUsageInKB();

  /**
   * @brief Resets the stopwatch to its initial state.
   *
   * This method resets all member variables by reassigning the object to a
   * default-constructed instance.
   */
  void reset() { *this = ResourceMonitor{}; }

 private:
  /// @brief Indicates whether resource monitoring is currently running.
  bool running_ = false;

  /// @brief Stores the CPU time at the start of the current episode.
  timespec episode_start_cpu_time_{};

  /// @brief Stores the wall clock time at the start of the current episode.
  timespec episode_start_wall_time_{};

  /// @brief Stores the CPU time duration of the last completed episode.
  Duration last_episode_cpu_duration_{};

  /// @brief Stores the wall clock time duration of the last completed episode.
  Duration last_episode_wall_duration_{};

  /// @brief Accumulates the total CPU time duration of all episodes.
  Duration total_cpu_duration_{};

  /// @brief Accumulates the total wall clock time duration of all episodes.
  Duration total_wall_duration_{};

  /**
   * @brief Computes the duration between two POSIX timestamps.
   *
   * @param start The starting POSIX timestamp.
   * @param stop The ending POSIX timestamp.
   * @return The computed duration as a wavemap::Duration.
   */
  static Duration computeDuration(const timespec& start, const timespec& stop);
};

}  // namespace wavemap

#endif  // WAVEMAP_CORE_UTILS_PROFILE_RESOURCE_MONITOR_H_
