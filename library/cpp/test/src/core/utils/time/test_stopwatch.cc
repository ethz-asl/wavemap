#include <thread>

#include <gtest/gtest.h>

#include "wavemap/core/utils/time/stopwatch.h"

namespace wavemap {
class StopwatchTest : public ::testing::Test {
 protected:
  Stopwatch stopwatch;
};

TEST_F(StopwatchTest, InitialState) {
  EXPECT_FALSE(stopwatch.isRunning());
  EXPECT_EQ(stopwatch.getLastEpisodeDuration(), 0.0);
  EXPECT_EQ(stopwatch.getTotalDuration(), 0.0);
}

TEST_F(StopwatchTest, Start) {
  stopwatch.start();
  EXPECT_TRUE(stopwatch.isRunning());
}

TEST_F(StopwatchTest, Stop) {
  stopwatch.start();
  stopwatch.stop();
  EXPECT_FALSE(stopwatch.isRunning());
}

TEST_F(StopwatchTest, Reset) {
  stopwatch.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  stopwatch.stop();

  stopwatch.reset();
  EXPECT_FALSE(stopwatch.isRunning());
  EXPECT_EQ(stopwatch.getLastEpisodeDuration(), 0.0);
  EXPECT_EQ(stopwatch.getTotalDuration(), 0.0);
}

TEST_F(StopwatchTest, DurationTracking) {
  stopwatch.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  stopwatch.stop();

  const double first_episode_duration = stopwatch.getLastEpisodeDuration();
  EXPECT_GT(first_episode_duration,
            0.09);  // Allow for small timing inaccuracies
  EXPECT_LT(first_episode_duration, 0.11);  // Tolerance of ±10ms

  const double total_duration = stopwatch.getTotalDuration();
  EXPECT_DOUBLE_EQ(first_episode_duration, total_duration);

  stopwatch.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  stopwatch.stop();

  const double second_episode_duration = stopwatch.getLastEpisodeDuration();
  EXPECT_GT(second_episode_duration,
            0.04);  // Allow for small timing inaccuracies
  EXPECT_LT(second_episode_duration, 0.06);  // Tolerance of ±10ms

  EXPECT_DOUBLE_EQ(stopwatch.getTotalDuration(),
                   first_episode_duration + second_episode_duration);
}

TEST_F(StopwatchTest, StopWithoutStart) {
  stopwatch.stop();
  EXPECT_FALSE(stopwatch.isRunning());
  EXPECT_EQ(stopwatch.getLastEpisodeDuration(), 0.0);
  EXPECT_EQ(stopwatch.getTotalDuration(), 0.0);
}

TEST_F(StopwatchTest, MultipleStarts) {
  stopwatch.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  stopwatch.start();  // Should have no effect
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  stopwatch.stop();

  const double episode_duration = stopwatch.getLastEpisodeDuration();
  EXPECT_GT(episode_duration, 0.14);  // Combined duration
  EXPECT_LT(episode_duration, 0.16);  // Tolerance of ±10ms
}
}  // namespace wavemap
