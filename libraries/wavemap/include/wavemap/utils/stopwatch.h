#ifndef WAVEMAP_UTILS_STOPWATCH_H_
#define WAVEMAP_UTILS_STOPWATCH_H_

#include <glog/logging.h>

#include "wavemap/utils/time.h"

namespace wavemap {
class Stopwatch {
 public:
  void start() {
    episode_start_time_ = Time::now();
    running = true;
  }

  void stop() {
    if (!running) {
      LOG(WARNING) << "Tried to stop timer that was not running.";
      return;
    }
    const Timestamp stop_wall_time = Time::now();
    running = false;

    last_episode_duration_ = stop_wall_time - episode_start_time_;
    total_duration_ += last_episode_duration_;
  }

  double getLastEpisodeDuration() const {
    return to_seconds<double>(last_episode_duration_);
  }
  double getTotalDuration() const {
    return to_seconds<double>(total_duration_);
  }

 private:
  bool running = false;

  Timestamp episode_start_time_;
  Duration last_episode_duration_;
  Duration total_duration_;
};
}  // namespace wavemap

#endif  // WAVEMAP_UTILS_STOPWATCH_H_
