#include <chrono>
#include <thread>

#include <gtest/gtest.h>

#include "wavemap/utils/thread_pool.h"

TEST(ThreadPoolTest, WaitAll) {
  auto dummy_fn = []() {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  };

  wavemap::ThreadPool pool(2);
  pool.add_task(dummy_fn);
  pool.add_task(dummy_fn);
  pool.add_task(dummy_fn);
  pool.add_task(dummy_fn);

  pool.wait_all();
}
