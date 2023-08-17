#include "wavemap/utils/stopwatch.h"

#include <glog/logging.h>

namespace wavemap {
void Stopwatch::start() {
  if (running) {
    LOG(WARNING) << "Tried to start timer that was already running.";
    return;
  }
  episode_start_time_ = Time::now();
  running = true;
}

void Stopwatch::stop() {
  if (!running) {
    LOG(WARNING) << "Tried to stop timer that was not running.";
    return;
  }
  const Timestamp stop_wall_time = Time::now();
  running = false;

  last_episode_duration_ = stop_wall_time - episode_start_time_;
  total_duration_ += last_episode_duration_;
}
}  // namespace wavemap
