#ifndef WAVEMAP_CORE_UTILS_TIME_STOPWATCH_H_
#define WAVEMAP_CORE_UTILS_TIME_STOPWATCH_H_

#include "wavemap/core/utils/time/time.h"

namespace wavemap {
/**
 * @brief A simple utility class for measuring elapsed time across episodes.
 *
 * The `Stopwatch` class allows tracking the duration of multiple timed
 * episodes. It provides functionality to start, stop, and retrieve timing
 * information for the last episode as well as the total duration of all
 * episodes.
 */
class Stopwatch {
 public:
  /**
   * @brief Starts the stopwatch for a new timing episode.
   *
   * Records the start time for the current episode. If the stopwatch is
   * already running, calling `start()` has no effect.
   */
  void start();

  /**
   * @brief Stops the stopwatch for the current timing episode.
   *
   * Records the end time for the current episode and updates the total
   * accumulated duration. If the stopwatch is not running, calling `stop()`
   * has no effect.
   */
  void stop();

  /**
   * @brief Checks if the stopwatch is currently running.
   *
   * @return `true` if the stopwatch is running, `false` otherwise.
   */
  bool isRunning() const { return running_; }

  /**
   * @brief Gets the duration of the last timing episode.
   *
   * @return The duration of the last episode in seconds as a `double`.
   *
   * The value represents the time elapsed during the most recently completed
   * pair of `start()` and `stop()` calls. If no episode has been completed,
   * this returns 0.
   */
  double getLastEpisodeDuration() const {
    return time::to_seconds<double>(last_episode_duration_);
  }

  /**
   * @brief Gets the total accumulated duration of all episodes.
   *
   * @return The total duration in seconds as a `double`.
   *
   * The value represents the sum of the durations of all episodes that have
   * been completed since the creation of the stopwatch or since it was last
   * reset.
   */
  double getTotalDuration() const {
    return time::to_seconds<double>(total_duration_);
  }

  /**
   * @brief Resets the stopwatch to its initial state.
   *
   * This method resets all member variables by reassigning the object to a
   * default-constructed instance.
   */
  void reset() { *this = Stopwatch{}; }

 private:
  /// @brief Indicates whether the stopwatch is currently running.
  bool running_ = false;

  /// @brief Stores the start time of the current episode.
  Timestamp episode_start_time_{};

  /// @brief Stores the duration of the last completed episode.
  Duration last_episode_duration_{};

  /// @brief Accumulates the total duration of all episodes.
  Duration total_duration_{};
};
}  // namespace wavemap

#endif  // WAVEMAP_CORE_UTILS_TIME_STOPWATCH_H_
