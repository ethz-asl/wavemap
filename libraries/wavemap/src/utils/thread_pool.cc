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
}

void ThreadPool::wait_all() {
  while (task_count_ > 0) {
    std::this_thread::yield();
  }
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
  }
}
}  // namespace wavemap
