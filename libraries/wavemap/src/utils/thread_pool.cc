#include "wavemap/utils/thread_pool.h"

namespace wavemap {
ThreadPool::ThreadPool(size_t thread_count)
    : task_count_(0), terminate_(false) {
  // Create the worker threads
  for (size_t i = 0; i < thread_count; ++i) {
    workers_.emplace_back([this] { worker_loop(); });
  }
}

ThreadPool::~ThreadPool() {
  {
    auto lock = std::unique_lock<std::mutex>(tasks_mutex_);
    terminate_ = true;
  }
  worker_condition_.notify_all();
  for (auto& worker : workers_) {
    worker.join();
  }
  wait_all_condition_.notify_all();
}

void ThreadPool::wait_all() {
  auto lock = std::unique_lock<std::mutex>(tasks_mutex_);
  wait_all_condition_.wait(lock,
                           [this] { return terminate_ || task_count_ <= 0; });
  task_count_ = 0;
}

void ThreadPool::worker_loop() {
  while (true) {
    // Obtain the next task
    auto task = std::function<void()>();
    {
      auto lock = std::unique_lock<std::mutex>(tasks_mutex_);

      // Wait for something to do
      worker_condition_.wait(lock,
                             [this] { return terminate_ || !tasks_.empty(); });

      if (terminate_ && tasks_.empty()) {
        return;
      }

      task = std::move(tasks_.front());
      tasks_.pop();
    }

    // Execute the task
    task();
    task_count_--;

    // Notify the waiting threads if we're done
    if (task_count_ == 0) {
      wait_all_condition_.notify_all();
    }
  }
}
}  // namespace wavemap
