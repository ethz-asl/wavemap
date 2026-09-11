#include <filesystem>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include <gtest/gtest.h>
#include <wavemap/core/common.h>
#include <wavemap/core/map/hashed_chunked_wavelet_octree.h>
#include <wavemap/core/map/hashed_wavelet_octree.h>
#include <wavemap/core/map/map_base.h>
#include <wavemap/core/map/wavelet_octree.h>
#include <wavemap/layered/integration/layer_update_policy.h>
#include <wavemap/layered/map/layered_map_definition.h>
#include <wavemap/layered/schema/layer_schema.h>
#include <wavemap/layered/types/rgb.h>
#include <wavemap/test/config_generator.h>
#include <wavemap/test/fixture_base.h>
#include <wavemap/test/geometry_generator.h>
#include <wavemap_msgs/Map.h>

#include "wavemap_ros_conversions/descriptor_layered_map_conversions.h"
#include "wavemap_ros_conversions/layered_map_file_conversions.h"
#include "wavemap_ros_conversions/map_msg_conversions.h"

namespace wavemap {
namespace {
struct TestReflectivityLayer
    : layered::schema::ContinuousLayer<
          FloatingPoint,
          layered::ReplaceLayerUpdatePolicy<FloatingPoint>> {
  static constexpr std::string_view name = "reflectivity";
};

using TestSchema = layered::schema::LayerSchema<TestReflectivityLayer>;
using ChunkedLayeredDefinition = layered::LayeredMapDefinition<
    TestSchema, layered::HashedChunkedWaveletOctreeBackend>;
using RegularLayeredDefinition = layered::LayeredMapDefinition<TestSchema>;
using TestLayerRosConverter = convert::DescriptorContinuousRosConverter<
    ChunkedLayeredDefinition::Voxel>;

struct GenericRgbLayer
    : layered::schema::ContinuousLayer<
          layered::Rgb,
          layered::ReplaceLayerUpdatePolicy<layered::Rgb>> {
  static constexpr std::string_view name = "generic_rgb";
};

struct GenericAverageLayer
    : layered::schema::StatefulContinuousLayer<
          FloatingPoint, layered::WeightedMeanState,
          layered::WeightedMeanLayerUpdatePolicy> {
  static constexpr std::string_view name = "generic_average";
};

struct GenericClassLayer
    : layered::schema::DiscreteLayer<
          int, layered::ReplaceLayerUpdatePolicy<int>> {
  static constexpr std::string_view name = "generic_class";
};

struct GenericMaskLayer
    : layered::schema::DiscreteLayer<
          bool, layered::ReplaceLayerUpdatePolicy<bool>> {
  static constexpr std::string_view name = "generic_mask";
};

using GenericFileDefinition = layered::LayeredMapDefinition<
    layered::schema::LayerSchema<TestReflectivityLayer, GenericRgbLayer,
                                 GenericAverageLayer, GenericClassLayer,
                                 GenericMaskLayer>>;
using GenericFileRosConverter = convert::DescriptorContinuousRosConverter<
    GenericFileDefinition::Voxel>;
}  // namespace

template <typename MapType>
class MapMsgConversionsTest : public FixtureBase,
                              public GeometryGenerator,
                              public ConfigGenerator {
 protected:
  void SetUp() override {
    ros::Time::init();
    stamp = ros::Time::now();
  }

  const std::string frame_id = "odom";
  ros::Time stamp{};
  static constexpr FloatingPoint kAcceptableReconstructionError = 5e-2f;
};

using MapTypes =
    ::testing::Types<HashedBlocks, WaveletOctree, HashedWaveletOctree,
                     HashedChunkedWaveletOctree>;
TYPED_TEST_SUITE(MapMsgConversionsTest, MapTypes, );

TYPED_TEST(MapMsgConversionsTest, MetadataPreservation) {
  const auto config =
      ConfigGenerator::getRandomConfig<typename TypeParam::Config>();

  // Create the original map and make sure it matches the config
  typename TypeParam::ConstPtr map = std::make_shared<TypeParam>(config);
  ASSERT_EQ(map->getMinCellWidth(), config.min_cell_width);
  ASSERT_EQ(map->getMinLogOdds(), config.min_log_odds);
  ASSERT_EQ(map->getMaxLogOdds(), config.max_log_odds);
  if constexpr (!std::is_same_v<TypeParam, HashedBlocks>) {
    ASSERT_EQ(map->getTreeHeight(), config.tree_height);
  }

  // Convert to base pointer
  MapBase::ConstPtr map_base = map;
  ASSERT_EQ(map_base->getMinCellWidth(), config.min_cell_width);
  ASSERT_EQ(map_base->getMinLogOdds(), config.min_log_odds);
  ASSERT_EQ(map_base->getMaxLogOdds(), config.max_log_odds);

  // Serialize and deserialize
  wavemap_msgs::Map map_msg;
  ASSERT_TRUE(convert::mapToRosMsg(*map_base, TestFixture::frame_id,
                                   TestFixture::stamp, map_msg));
  MapBase::Ptr map_base_round_trip;
  ASSERT_TRUE(convert::rosMsgToMap(map_msg, map_base_round_trip));
  ASSERT_TRUE(map_base_round_trip);

  // Check the header
  EXPECT_EQ(map_msg.header.frame_id, TestFixture::frame_id);
  EXPECT_EQ(map_msg.header.stamp, TestFixture::stamp);

  // TODO(victorr): Add option to deserialize into hashed chunked wavelet
  //                octrees, instead of implicitly converting them to regular
  //                hashed wavelet octrees.
  if constexpr (std::is_same_v<TypeParam, HashedChunkedWaveletOctree>) {
    HashedWaveletOctree::ConstPtr map_round_trip =
        std::dynamic_pointer_cast<HashedWaveletOctree>(map_base_round_trip);
    ASSERT_TRUE(map_round_trip);

    // Check that the metadata still matches the original config
    EXPECT_EQ(map_round_trip->getMinCellWidth(), config.min_cell_width);
    EXPECT_EQ(map_round_trip->getMinLogOdds(), config.min_log_odds);
    EXPECT_EQ(map_round_trip->getMaxLogOdds(), config.max_log_odds);
    EXPECT_EQ(map_round_trip->getTreeHeight(), config.tree_height);
  } else {
    typename TypeParam::ConstPtr map_round_trip =
        std::dynamic_pointer_cast<TypeParam>(map_base_round_trip);
    ASSERT_TRUE(map_round_trip);

    // Check that the metadata still matches the original config
    EXPECT_EQ(map_round_trip->getMinCellWidth(), config.min_cell_width);
    EXPECT_EQ(map_round_trip->getMinLogOdds(), config.min_log_odds);
    EXPECT_EQ(map_round_trip->getMaxLogOdds(), config.max_log_odds);
    if constexpr (!std::is_same_v<TypeParam, HashedBlocks>) {
      EXPECT_EQ(map_round_trip->getTreeHeight(), config.tree_height);
    }
  }
}

TYPED_TEST(MapMsgConversionsTest, InsertionAndLeafVisitor) {
  constexpr int kNumRepetitions = 3;
  for (int i = 0; i < kNumRepetitions; ++i) {
    // Create a random map
    const auto config =
        ConfigGenerator::getRandomConfig<typename TypeParam::Config>();
    TypeParam map_original(config);
    const std::vector<Index3D> random_indices =
        GeometryGenerator::getRandomIndexVector<3>(
            1000u, 2000u, Index3D::Constant(-5000), Index3D::Constant(5000));
    for (const Index3D& index : random_indices) {
      const FloatingPoint update = TestFixture::getRandomUpdate();
      map_original.addToCellValue(index, update);
    }
    map_original.prune();

    // Serialize and deserialize
    wavemap_msgs::Map map_msg;
    ASSERT_TRUE(convert::mapToRosMsg(map_original, TestFixture::frame_id,
                                     TestFixture::stamp, map_msg));
    MapBase::Ptr map_base_round_trip;
    ASSERT_TRUE(convert::rosMsgToMap(map_msg, map_base_round_trip));
    ASSERT_TRUE(map_base_round_trip);

    // Check that both maps contain the same leaves
    map_base_round_trip->forEachLeaf(
        [&map_original](const OctreeIndex& node_index,
                        FloatingPoint round_trip_value) {
          if constexpr (std::is_same_v<TypeParam, HashedBlocks>) {
            EXPECT_EQ(node_index.height, 0);
            EXPECT_NEAR(round_trip_value,
                        map_original.getValueOrDefault(node_index.position),
                        TestFixture::kAcceptableReconstructionError);
          } else {
            EXPECT_NEAR(round_trip_value, map_original.getCellValue(node_index),
                        TestFixture::kAcceptableReconstructionError);
          }
        });

    // TODO(victorr): Remove this special case once deserializing directly
    //                into HashedChunkedWaveletOctrees is supported
    if (std::is_same_v<TypeParam, HashedChunkedWaveletOctree>) {
      HashedWaveletOctree::ConstPtr map_round_trip =
          std::dynamic_pointer_cast<HashedWaveletOctree>(map_base_round_trip);
      ASSERT_TRUE(map_round_trip);

      map_original.forEachLeaf([&map_round_trip](const OctreeIndex& node_index,
                                                 FloatingPoint original_value) {
        EXPECT_NEAR(original_value, map_round_trip->getCellValue(node_index),
                    TestFixture::kAcceptableReconstructionError);
      });
    } else {
      typename TypeParam::ConstPtr map_round_trip =
          std::dynamic_pointer_cast<TypeParam>(map_base_round_trip);
      ASSERT_TRUE(map_round_trip);

      map_original.forEachLeaf([&map_round_trip](const OctreeIndex& node_index,
                                                 FloatingPoint original_value) {
        if constexpr (std::is_same_v<TypeParam, HashedBlocks>) {
          EXPECT_EQ(node_index.height, 0);
          EXPECT_NEAR(original_value,
                      map_round_trip->getValueOrDefault(node_index.position),
                      TestFixture::kAcceptableReconstructionError);
        } else {
          EXPECT_NEAR(original_value, map_round_trip->getCellValue(node_index),
                      TestFixture::kAcceptableReconstructionError);
        }
      });
    }
  }
}

TEST(LayeredMapMsgConversionsTest,
     ChunkedMapUsesLogicalHashedWaveletRepresentation) {
  ros::Time::init();

  ChunkedLayeredDefinition::ContinuousMap::Config config;
  config.min_cell_width = 0.2f;
  config.min_log_odds = -2.f;
  config.max_log_odds = 4.f;
  config.tree_height = 4;

  ChunkedLayeredDefinition::ContinuousMap chunked_map(config);
  const Index3D cell_index{3, -2, 5};
  ChunkedLayeredDefinition::Voxel voxel;
  voxel.occupancy = 0.7f;
  voxel.data.get<TestReflectivityLayer>() = 0.35f;
  chunked_map.setVoxelValue(cell_index, voxel);

  wavemap_msgs::Map map_msg;
  ASSERT_TRUE((convert::mapToRosMsg<ChunkedLayeredDefinition::Voxel,
                                    TestLayerRosConverter>(
      chunked_map, "odom", ros::Time(42), map_msg)));
  ASSERT_EQ(map_msg.layered_hashed_wavelet_octree.size(), 1u);
  const auto& layered_msg = map_msg.layered_hashed_wavelet_octree.front();
  EXPECT_EQ(layered_msg.layer_names,
            std::vector<std::string>({"reflectivity"}));
  EXPECT_EQ(layered_msg.layer_types,
            std::vector<std::string>({"float32"}));
  EXPECT_FALSE(layered_msg.blocks.empty());

  RegularLayeredDefinition::ContinuousMap::Ptr regular_map;
  ASSERT_TRUE((convert::rosMsgToMap<RegularLayeredDefinition::Voxel,
                                    TestLayerRosConverter>(map_msg,
                                                           regular_map)));
  ASSERT_TRUE(regular_map);
  const auto reconstructed = regular_map->getVoxelValue(cell_index);
  EXPECT_NEAR(reconstructed.occupancy, voxel.occupancy, 1e-5f);
  EXPECT_NEAR(reconstructed.data.get<TestReflectivityLayer>(),
              voxel.data.get<TestReflectivityLayer>(), 1e-5f);
}

TEST(LayeredMapFileConversionsTest,
     LoadsBuiltInCodecsWithoutARegisteredRuntimeSchema) {
  ros::Time::init();

  GenericFileDefinition::Config config;
  config.continuous_map.min_cell_width = 0.125f;
  config.continuous_map.min_log_odds = -3.f;
  config.continuous_map.max_log_odds = 5.f;
  config.continuous_map.tree_height = 3;
  config.discrete_compression.block_height = 2;
  GenericFileDefinition::Map map(config);

  const Index3D first_index{1, 2, 3};
  const Index3D second_index{2, 2, 3};
  GenericFileDefinition::Voxel voxel;
  voxel.occupancy = 0.8f;
  voxel.data.get<TestReflectivityLayer>() = 0.35f;
  voxel.data.get<GenericRgbLayer>() = {0.2f, 0.4f, 0.6f};
  voxel.data.get<GenericAverageLayer>() = {2.4f, 3.f};
  map.continuousMap().setVoxelValue(first_index, voxel);
  map.discreteLayers().get<GenericClassLayer>().setValue(first_index, 4);
  map.discreteLayers().get<GenericClassLayer>().setValue(second_index, 7);
  map.discreteLayers().get<GenericMaskLayer>().setValue(first_index, true);
  map.discreteLayers().get<GenericMaskLayer>().setValue(second_index, false);

  const auto file_path = std::filesystem::temp_directory_path() /
                         "wavemap_runtime_schema_conversion_test.lwvmp";
  ASSERT_TRUE(GenericFileDefinition::MapIo::save(file_path, map));

  const ros::Time stamp(42);
  wavemap_msgs::LayeredMap expected;
  ASSERT_TRUE((convert::layeredMapToRosMsg<GenericFileDefinition::Map,
                                           GenericFileRosConverter>(
      map, "test_frame", stamp, expected)));
  wavemap_msgs::LayeredMap actual;
  std::string error_message;
  const bool loaded = convert::layeredMapFileToRosMsg(
      file_path, "test_frame", stamp, actual, &error_message);
  std::filesystem::remove(file_path);
  ASSERT_TRUE(loaded) << error_message;

  ASSERT_EQ(actual.continuous_map.layered_hashed_wavelet_octree.size(), 1u);
  ASSERT_EQ(expected.continuous_map.layered_hashed_wavelet_octree.size(), 1u);
  const auto& actual_continuous =
      actual.continuous_map.layered_hashed_wavelet_octree.front();
  const auto& expected_continuous =
      expected.continuous_map.layered_hashed_wavelet_octree.front();
  EXPECT_EQ(actual.header, expected.header);
  EXPECT_TRUE(actual.is_full_update);
  EXPECT_FLOAT_EQ(actual_continuous.min_cell_width,
                  expected_continuous.min_cell_width);
  EXPECT_FLOAT_EQ(actual_continuous.min_log_odds,
                  expected_continuous.min_log_odds);
  EXPECT_FLOAT_EQ(actual_continuous.max_log_odds,
                  expected_continuous.max_log_odds);
  EXPECT_EQ(actual_continuous.tree_height, expected_continuous.tree_height);
  EXPECT_EQ(actual_continuous.layer_names, expected_continuous.layer_names);
  EXPECT_EQ(actual_continuous.layer_types, expected_continuous.layer_types);
  EXPECT_EQ(actual_continuous.allocated_block_indices,
            expected_continuous.allocated_block_indices);
  EXPECT_EQ(actual_continuous.blocks, expected_continuous.blocks);
  EXPECT_EQ(actual.discrete_layers, expected.discrete_layers);
}
}  // namespace wavemap
