#include <chrono>
#include <thread>

#include <gtest/gtest.h>

#include "wavemap/core/utils/thread_pool.h"
#include "wavemap/core/utils/time/stopwatch.h"

namespace wavemap {
TEST(ThreadPoolTest, WaitAll) {
  constexpr Duration kSleepTime = std::chrono::milliseconds{10};
  auto dummy_fn = [kSleepTime]() { std::this_thread::sleep_for(kSleepTime); };

  // Create the thread pool
  constexpr int kNumThreads = 2;
  ThreadPool pool(kNumThreads);

  // Start measuring time
  Stopwatch stopwatch;
  stopwatch.start();

  // Add the tasks to the pool
  constexpr int kNumTasks = 4;
  std::vector<std::future<void>> futures;
  futures.reserve(kNumTasks);
  for (int task_idx = 0; task_idx < kNumTasks; ++task_idx) {
    auto future = pool.add_task(dummy_fn);
    futures.emplace_back(std::move(future));
  }
  pool.wait_all();
  stopwatch.stop();

  // Check that all futures are available after calling wait_all
  for (int task_idx = 0; task_idx < kNumTasks; ++task_idx) {
    auto& future = futures[task_idx];
    EXPECT_TRUE(future.valid());
  }

  // Check that executing the tasks took as long as expected
  EXPECT_GE(stopwatch.getLastEpisodeDuration(),
            time::to_seconds<double>(kSleepTime * kNumTasks / kNumThreads));
}

TEST(ThreadPoolTest, FutureResults) {
  constexpr int kNumThreads = 2;
  ThreadPool pool(kNumThreads);

  // Add the tasks to the pool
  constexpr int kNumTasks = 4;
  std::vector<std::future<int>> futures;
  futures.reserve(kNumTasks);
  for (int task_idx = 0; task_idx < kNumTasks; ++task_idx) {
    const auto sleep_time = std::chrono::milliseconds(kNumTasks - task_idx);
    auto future = pool.add_task([task_idx, sleep_time]() {
      std::this_thread::sleep_for(sleep_time);
      return task_idx;
    });
    futures.emplace_back(std::move(future));
  }

  // Wait for the futures one by one and check their results
  for (int task_idx = 0; task_idx < kNumTasks; ++task_idx) {
    auto& future = futures[task_idx];
    future.wait();
    EXPECT_TRUE(future.valid());
    EXPECT_EQ(future.get(), task_idx);
  }
}
}  // namespace wavemap
