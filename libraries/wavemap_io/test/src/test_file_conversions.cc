#include <gtest/gtest.h>
#include <wavemap/common.h>
#include <wavemap/data_structure/volumetric/hashed_chunked_wavelet_octree.h>
#include <wavemap/data_structure/volumetric/hashed_wavelet_octree.h>
#include <wavemap/data_structure/volumetric/volumetric_data_structure_base.h>
#include <wavemap/data_structure/volumetric/wavelet_octree.h>
#include <wavemap/test/config_generator.h>
#include <wavemap/test/fixture_base.h>
#include <wavemap/test/geometry_generator.h>
#include <wavemap/utils/container_print_utils.h>

#include "wavemap_io/file_conversions.h"

namespace wavemap {
template <typename VolumetricDataStructureType>
class FileConversionsTest : public FixtureBase,
                            public GeometryGenerator,
                            public ConfigGenerator {
 protected:
  static constexpr FloatingPoint kAcceptableReconstructionError = 5e-2f;
  static constexpr auto kTemporaryFilePath = "/tmp/tmp.wvmp";
};

using VolumetricDataStructureTypes =
    ::testing::Types<WaveletOctree, HashedWaveletOctree,
                     HashedChunkedWaveletOctree>;
TYPED_TEST_SUITE(FileConversionsTest, VolumetricDataStructureTypes, );

TYPED_TEST(FileConversionsTest, MetadataPreservation) {
  const auto config =
      ConfigGenerator::getRandomConfig<typename TypeParam::Config>();

  // Create the original map and make sure it matches the config
  typename TypeParam::ConstPtr map = std::make_shared<TypeParam>(config);
  ASSERT_EQ(map->getMinCellWidth(), config.min_cell_width);
  ASSERT_EQ(map->getMinLogOdds(), config.min_log_odds);
  ASSERT_EQ(map->getMaxLogOdds(), config.max_log_odds);
  ASSERT_EQ(map->getTreeHeight(), config.tree_height);

  // Convert to base pointer
  VolumetricDataStructureBase::ConstPtr map_base = map;
  ASSERT_EQ(map_base->getMinCellWidth(), config.min_cell_width);
  ASSERT_EQ(map_base->getMinLogOdds(), config.min_log_odds);
  ASSERT_EQ(map_base->getMaxLogOdds(), config.max_log_odds);

  // Serialize and deserialize
  ASSERT_TRUE(io::mapToFile(*map_base, TestFixture::kTemporaryFilePath));
  VolumetricDataStructureBase::Ptr map_base_round_trip;
  ASSERT_TRUE(
      io::fileToMap(TestFixture::kTemporaryFilePath, map_base_round_trip));
  ASSERT_TRUE(map_base_round_trip);

  // TODO(victorr): Add option to deserialize into hashed chunked wavelet
  //                octrees, instead of implicitly converting them to regular
  //                hashed wavelet octrees.
  if (std::is_same_v<TypeParam, HashedChunkedWaveletOctree>) {
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
    EXPECT_EQ(map_round_trip->getTreeHeight(), config.tree_height);
  }
}

TYPED_TEST(FileConversionsTest, InsertionAndLeafVisitor) {
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
    ASSERT_TRUE(io::mapToFile(map_original, TestFixture::kTemporaryFilePath));
    VolumetricDataStructureBase::Ptr map_base_round_trip;
    ASSERT_TRUE(
        io::fileToMap(TestFixture::kTemporaryFilePath, map_base_round_trip));
    ASSERT_TRUE(map_base_round_trip);

    // Check that both maps contain the same leaves
    map_base_round_trip->forEachLeaf(
        [&map_original](const OctreeIndex& node_index,
                        FloatingPoint round_trip_value) {
          EXPECT_NEAR(round_trip_value, map_original.getCellValue(node_index),
                      TestFixture::kAcceptableReconstructionError);
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
        EXPECT_NEAR(original_value, map_round_trip->getCellValue(node_index),
                    TestFixture::kAcceptableReconstructionError);
      });
    }
  }
}
}  // namespace wavemap
