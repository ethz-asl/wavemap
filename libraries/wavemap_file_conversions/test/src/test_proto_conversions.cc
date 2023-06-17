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

#include "wavemap_file_conversions/proto_conversions.h"

namespace wavemap {
using GeneralProtoConversionsTest = FixtureBase;

TEST_F(GeneralProtoConversionsTest, Index) {
  const auto indices = GeometryGenerator().getRandomIndexVector<3>(1000, 2000);
  for (const auto& index : indices) {
    proto::Index index_proto;
    convert::indexToProto(index, &index_proto);
    Index3D index_round_trip;
    convert::protoToIndex(index_proto, index_round_trip);
    EXPECT_EQ(index_round_trip[0], index[0]);
    EXPECT_EQ(index_round_trip[1], index[1]);
    EXPECT_EQ(index_round_trip[2], index[2]);
  }
}

TEST_F(GeneralProtoConversionsTest, Details) {
  for (int repetition = 0; repetition < 1000; ++repetition) {
    using Details = HaarCoefficients<FloatingPoint, 3>::Details;
    using DetailsProto = google::protobuf::RepeatedField<float>;
    constexpr int kNumDetails =
        HaarCoefficients<FloatingPoint, 3>::kNumDetailCoefficients;
    Details coefficients{};
    std::generate(coefficients.begin(), coefficients.end(),
                  [this]() { return getRandomFloat(-1e3f, 1e3f); });
    DetailsProto coefficients_proto;
    convert::detailsToProto(coefficients, &coefficients_proto);
    Details coefficients_round_trip;
    convert::protoToDetails(coefficients_proto, coefficients_round_trip);
    for (int idx = 0; idx < kNumDetails; ++idx) {
      EXPECT_EQ(coefficients_round_trip[idx], coefficients[idx])
          << "For idx " << idx;
    }
  }
}

template <typename VolumetricDataStructureType>
class MapProtoConversionsTest : public FixtureBase,
                                public GeometryGenerator,
                                public ConfigGenerator {
 protected:
  static constexpr FloatingPoint kAcceptableReconstructionError = 5e-2f;
};

using VolumetricDataStructureTypes =
    ::testing::Types<WaveletOctree, HashedWaveletOctree,
                     HashedChunkedWaveletOctree>;
TYPED_TEST_SUITE(MapProtoConversionsTest, VolumetricDataStructureTypes, );

TYPED_TEST(MapProtoConversionsTest, MetadataPreservation) {
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
  proto::Map map_proto;
  ASSERT_TRUE(convert::mapToProto(*map_base, &map_proto));
  VolumetricDataStructureBase::Ptr map_base_round_trip;
  convert::protoToMap(map_proto, map_base_round_trip);
  ASSERT_TRUE(map_base_round_trip);

  // TODO(victorr): Remove this special case once deserializing directly into
  //                HashedChunkedWaveletOctrees is supported
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

TYPED_TEST(MapProtoConversionsTest, InsertionAndLeafVisitor) {
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
    proto::Map map_proto;
    convert::mapToProto(map_original, &map_proto);
    VolumetricDataStructureBase::Ptr map_base_round_trip;
    convert::protoToMap(map_proto, map_base_round_trip);
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
