#include <string>

#include <gtest/gtest.h>

#include "wavemap/io/config/file_conversions.h"
#include "wavemap/test/fixture_base.h"

namespace wavemap {
using ConfigFileConversionTest = FixtureBase;

TEST_F(ConfigFileConversionTest, Reading) {
  std::filesystem::path data_dir = DATADIR;
  std::filesystem::path config_file_path = data_dir / "config_file.yaml";

  const auto params = io::yamlFileToParams(config_file_path);

#ifndef YAML_CPP_AVAILABLE
  EXPECT_EQ(params, std::nullopt);
  return;
#endif

  ASSERT_TRUE(params.has_value());
  EXPECT_TRUE(params->holds<param::Map>());
  EXPECT_TRUE(params->hasChild("general"));
  EXPECT_TRUE(params->hasChild("map"));
  EXPECT_TRUE(params->hasChild("map_operations"));
  EXPECT_TRUE(params->hasChild("measurement_integrators"));
  EXPECT_TRUE(params->hasChild("inputs"));

  // Test map loading
  const auto map = params->getChildAs<param::Map>("map");
  ASSERT_TRUE(map.has_value());
  EXPECT_EQ(map->size(), 2);

  // Test array loading
  const auto map_operations =
      params->getChildAs<param::Array>("map_operations");
  ASSERT_TRUE(map_operations.has_value())
      << "Could not convert value for key \"map_operations\" to a "
         "param::Array.";
  EXPECT_EQ(map_operations->size(), 3);

  // Test loading primitive types
  const auto inputs = params->getChildAs<param::Array>("inputs");
  ASSERT_TRUE(inputs.has_value());
  const auto& input = inputs->operator[](0);
  // Booleans
  const auto undistort_motion = input.getChildAs<bool>("undistort_motion");
  ASSERT_TRUE(undistort_motion.has_value());
  EXPECT_EQ(undistort_motion.value(), true);
  // Integers
  const auto topic_queue_length = input.getChildAs<int>("topic_queue_length");
  ASSERT_TRUE(topic_queue_length.has_value());
  EXPECT_EQ(topic_queue_length.value(), 10);
  // Floating points
  const auto max_wait_for_pose = input.getChild("max_wait_for_pose");
  ASSERT_TRUE(max_wait_for_pose.has_value());
  const auto seconds = max_wait_for_pose->getChildAs<FloatingPoint>("seconds");
  ASSERT_TRUE(seconds.has_value());
  EXPECT_EQ(seconds.value(), 1.f);
  // Strings
  const auto topic_name = input.getChildAs<std::string>("topic_name");
  ASSERT_TRUE(topic_name.has_value());
  EXPECT_EQ(topic_name.value(), "/os_cloud_node/points");
}
}  // namespace wavemap
