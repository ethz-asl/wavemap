#ifndef WAVEMAP_UTILS_THREAD_POOL_H_
#define WAVEMAP_UTILS_THREAD_POOL_H_

#include <algorithm>
#include <atomic>
#include <condition_variable>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <utility>
#include <vector>

#include <glog/logging.h>

namespace wavemap {
/**
 * \brief Implements a thread pool with a fixed number of threads.
 */
class ThreadPool {
 public:
  /**
   * \brief Creates a new thread pool with the a number of workers.
   *
   * \param thread_count number of worker threads
   */
  explicit ThreadPool(
      size_t thread_count = std::max(1u, std::thread::hardware_concurrency()));

  // Prevent copying etc. of this class
  ThreadPool(ThreadPool const& other) = delete;
  ThreadPool& operator=(ThreadPool const& other) = delete;

  /**
   * \brief Destructor, ensuring all threads terminate.
   */
  ~ThreadPool();

  /**
   * \brief Waits for all work to be complete.
   */
  void wait_all();

  /**
   * \brief Adds a callable task to the task queue.
   *
   * \param callable the executable task
   * \param args the additional arguments for the task
   * \return std::future containing the result of the computation
   */
  template <typename Callable, typename... Args>
  std::future<std::result_of_t<Callable(Args...)>> add_task(Callable&& callable,
                                                            Args&&... args);

 private:
  /**
   * \brief Loop executed by each of the workers.
   */
  void worker_loop();

 private:
  //! Worker threads of the pool
  std::vector<std::thread> workers_;
  //! Queue of tasks to execute
  std::queue<std::function<void()>> tasks_;
  //! Count of tasks yet to be completed
  std::atomic<int> task_count_;

  //! Task queue synchronization mutex
  std::mutex tasks_mutex_;
  //! Worker thread notification condition variable
  std::condition_variable worker_condition_;
  //! Waiting thread notification condition variable
  std::condition_variable wait_all_condition_;
  //! Flag indicating the termination of all workers
  std::atomic<bool> terminate_;
};

template <typename Callable, typename... Args>
std::future<std::result_of_t<Callable(Args...)>> ThreadPool::add_task(
    Callable&& callable, Args&&... args) {
  using ReturnType = std::result_of_t<Callable(Args...)>;

  auto task = std::make_shared<std::packaged_task<ReturnType()>>(
      std::bind(std::forward<Callable>(callable), std::forward<Args>(args)...));

  {
    auto lock = std::unique_lock<std::mutex>(tasks_mutex_);
    if (terminate_) {
      LOG(FATAL) << "Adding tasks to an already stopped pool.";
    }
    tasks_.emplace([task]() { (*task)(); });
    task_count_++;
  }
  worker_condition_.notify_one();

  return task->get_future();
}
}  // namespace wavemap

#endif  // WAVEMAP_UTILS_THREAD_POOL_H_
