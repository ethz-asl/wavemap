#include <chrono>
#include <thread>

#include <gtest/gtest.h>

#include "wavemap/utils/thread_pool.h"

TEST(ThreadPoolTest, WaitAll) {
  auto dummy_fn = []() {
    std::cout << std::this_thread::get_id() << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
  };

  wavemap::ThreadPool pool(2);
  pool.add_task(dummy_fn);
  pool.add_task(dummy_fn);
  pool.add_task(dummy_fn);
  pool.add_task(dummy_fn);

  pool.wait_all();
}
