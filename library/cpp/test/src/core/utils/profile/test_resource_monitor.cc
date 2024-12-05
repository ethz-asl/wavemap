#include <thread>

#include <gtest/gtest.h>

#include "wavemap/core/utils/profile/resource_monitor.h"

namespace wavemap {
class ResourceMonitorTest : public ::testing::Test {
 protected:
  ResourceMonitor resource_monitor;
};

TEST_F(ResourceMonitorTest, InitialState) {
  EXPECT_FALSE(resource_monitor.isRunning());
  EXPECT_EQ(resource_monitor.getLastEpisodeCpuTime(), 0.0);
  EXPECT_EQ(resource_monitor.getLastEpisodeWallTime(), 0.0);
  EXPECT_EQ(resource_monitor.getTotalCpuTime(), 0.0);
  EXPECT_EQ(resource_monitor.getTotalWallTime(), 0.0);
}

TEST_F(ResourceMonitorTest, Start) {
  resource_monitor.start();
  EXPECT_TRUE(resource_monitor.isRunning());
}

TEST_F(ResourceMonitorTest, Stop) {
  resource_monitor.start();
  resource_monitor.stop();
  EXPECT_FALSE(resource_monitor.isRunning());
}

TEST_F(ResourceMonitorTest, Reset) {
  resource_monitor.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
  resource_monitor.stop();

  resource_monitor.reset();
  EXPECT_FALSE(resource_monitor.isRunning());
  EXPECT_EQ(resource_monitor.getLastEpisodeCpuTime(), 0.0);
  EXPECT_EQ(resource_monitor.getLastEpisodeWallTime(), 0.0);
  EXPECT_EQ(resource_monitor.getTotalCpuTime(), 0.0);
  EXPECT_EQ(resource_monitor.getTotalWallTime(), 0.0);
}

TEST_F(ResourceMonitorTest, DurationTracking) {
  resource_monitor.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  resource_monitor.stop();

  const double first_wall_time = resource_monitor.getLastEpisodeWallTime();
  EXPECT_GT(first_wall_time, 0.09);  // Allow for small timing inaccuracies
  EXPECT_LT(first_wall_time, 0.11);  // Tolerance of ±10ms

  const double first_cpu_time = resource_monitor.getLastEpisodeCpuTime();
  EXPECT_GT(first_cpu_time, 0.0);  // CPU time should be non-zero

  EXPECT_EQ(resource_monitor.getTotalWallTime(), first_wall_time);
  EXPECT_EQ(resource_monitor.getTotalCpuTime(), first_cpu_time);

  resource_monitor.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  resource_monitor.stop();

  const double second_wall_time = resource_monitor.getLastEpisodeWallTime();
  EXPECT_GT(second_wall_time, 0.04);  // Allow for small timing inaccuracies
  EXPECT_LT(second_wall_time, 0.06);  // Tolerance of ±10ms

  const double second_cpu_time = resource_monitor.getLastEpisodeCpuTime();
  EXPECT_DOUBLE_EQ(resource_monitor.getTotalWallTime(),
                   first_wall_time + second_wall_time);
  EXPECT_DOUBLE_EQ(resource_monitor.getTotalCpuTime(),
                   first_cpu_time + second_cpu_time);
}

TEST_F(ResourceMonitorTest, StopWithoutStart) {
  resource_monitor.stop();
  EXPECT_FALSE(resource_monitor.isRunning());
  EXPECT_EQ(resource_monitor.getLastEpisodeCpuTime(), 0.0);
  EXPECT_EQ(resource_monitor.getLastEpisodeWallTime(), 0.0);
}

TEST_F(ResourceMonitorTest, MultipleStarts) {
  resource_monitor.start();
  std::this_thread::sleep_for(std::chrono::milliseconds(100));
  resource_monitor.start();  // Should have no effect
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  resource_monitor.stop();

  const double wall_time = resource_monitor.getLastEpisodeWallTime();
  EXPECT_GT(wall_time, 0.14);  // Combined duration
  EXPECT_LT(wall_time, 0.16);  // Tolerance of ±10ms
}

TEST_F(ResourceMonitorTest, RamUsage) {
  const auto ram_usage = ResourceMonitor::getCurrentRamUsageInKB();
  ASSERT_TRUE(ram_usage.has_value());  // Check if the optional contains a value
  EXPECT_GT(ram_usage.value(), 0);     // RAM usage should be a positive value
}
}  // namespace wavemap
