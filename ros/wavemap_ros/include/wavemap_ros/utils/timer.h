#ifndef WAVEMAP_ROS_UTILS_TIMER_H_
#define WAVEMAP_ROS_UTILS_TIMER_H_

#include <pthread.h>

#include <glog/logging.h>

namespace wavemap {
struct Time : timespec {
  Time() : timespec{} {}
  Time(int64_t sec, int64_t nsec) : timespec{sec, nsec} {}

  double toSec() const {
    return static_cast<double>(tv_sec) + static_cast<double>(tv_nsec) * 1e-9;
  }

  Time operator+(const Time& rhs) const {
    return Time{tv_sec + rhs.tv_sec, tv_nsec + rhs.tv_nsec};
  }
  Time operator-(const Time& rhs) const {
    return Time{tv_sec - rhs.tv_sec, tv_nsec - rhs.tv_nsec};
  }

  Time& operator+=(const Time& rhs) { return *this = *this + rhs; }
  Time& operator-=(const Time& rhs) { return *this = *this - rhs; }
};

class Timer {
 public:
  void start() {
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &episode_start_cpu_time_);
    clock_gettime(CLOCK_MONOTONIC, &episode_start_wall_time_);
    running = true;
  }

  void stop() {
    if (!running) {
      LOG(WARNING) << "Tried to stop timer that was not running.";
      return;
    }

    Time stop_cpu_time;
    Time stop_wall_time;
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &stop_cpu_time);
    clock_gettime(CLOCK_MONOTONIC, &stop_wall_time);
    running = false;

    last_episode_cpu_time_ = stop_cpu_time - episode_start_cpu_time_;
    last_episode_wall_time_ = stop_wall_time - episode_start_wall_time_;
    total_cpu_time_ += last_episode_cpu_time_;
    total_wall_time_ += last_episode_wall_time_;
  }

  double getLastEpisodeCpuTime() const {
    return last_episode_cpu_time_.toSec();
  }
  double getLastEpisodeWallTime() const {
    return last_episode_wall_time_.toSec();
  }

  double getTotalCpuTime() const { return total_cpu_time_.toSec(); }
  double getTotalWallTime() const { return total_wall_time_.toSec(); }

 private:
  bool running = false;

  Time episode_start_cpu_time_;
  Time episode_start_wall_time_;

  Time last_episode_cpu_time_;
  Time last_episode_wall_time_;

  Time total_cpu_time_;
  Time total_wall_time_;
};
}  // namespace wavemap

#endif  // WAVEMAP_ROS_UTILS_TIMER_H_
