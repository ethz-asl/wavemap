#ifndef WAVEMAP_2D_ROS_UTILS_TIMER_H_
#define WAVEMAP_2D_ROS_UTILS_TIMER_H_

namespace wavemap_2d {
class CpuTimer {
 public:
  CpuTimer() : running(false), cpu_time_total(0.0) {}  // NOLINT

  void start() {
    pthread_getcpuclockid(pthread_self(), &thread_clock_id_start_);
    clock_gettime(thread_clock_id_start_, &cpu_time_start_);
    running = true;
  }

  double stop() {
    if (!running) {
      LOG(WARNING) << "Tried to stop timer that was not running.";
      return 0.0;
    }

    clockid_t thread_clock_id_stop;
    pthread_getcpuclockid(pthread_self(), &thread_clock_id_stop);
    if (thread_clock_id_stop != thread_clock_id_start_) {
      LOG(WARNING) << "Thread migrated. CPU measurement will be discarded.";
      return 0.0;
    }

    struct timespec cpu_time_stop;  // NOLINT
    clock_gettime(thread_clock_id_start_, &cpu_time_stop);
    running = false;

    double cpu_time_delta =
        static_cast<double>(cpu_time_stop.tv_sec - cpu_time_start_.tv_sec) +
        static_cast<double>(cpu_time_stop.tv_nsec - cpu_time_start_.tv_nsec) *
            1e-9;
    cpu_time_total += cpu_time_delta;

    return cpu_time_delta;
  }

  double getTotal() const { return cpu_time_total; }

 protected:
  bool running;
  double cpu_time_total;

  struct timespec cpu_time_start_;
  clockid_t thread_clock_id_start_;
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_ROS_UTILS_TIMER_H_
