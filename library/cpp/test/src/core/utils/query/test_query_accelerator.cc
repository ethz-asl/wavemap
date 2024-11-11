#include <vector>

#include <gtest/gtest.h>
#include <wavemap/core/common.h>
#include <wavemap/core/map/hashed_wavelet_octree.h>
#include <wavemap/core/map/wavelet_octree.h>
#include <wavemap/test/config_generator.h>
#include <wavemap/test/fixture_base.h>
#include <wavemap/test/geometry_generator.h>

#include "wavemap/core/utils/query/query_accelerator.h"

namespace wavemap {
template <typename MapType>
class QueryAcceleratorTest : public FixtureBase,
                             public GeometryGenerator,
                             public ConfigGenerator {
 protected:
  static constexpr FloatingPoint kNumericalNoise = 2 * kEpsilon;
};

using MapTypes =
    ::testing::Types<HashedWaveletOctree, HashedChunkedWaveletOctree>;
TYPED_TEST_SUITE(QueryAcceleratorTest, MapTypes, );

TYPED_TEST(QueryAcceleratorTest, Equivalence) {
  constexpr int kNumRepetitions = 10;
  for (int i = 0; i < kNumRepetitions; ++i) {
    // Create a random map
    const auto config =
        ConfigGenerator::getRandomConfig<typename TypeParam::Config>();
    TypeParam map(config);
    const std::vector<Index3D> random_indices =
        GeometryGenerator::getRandomIndexVector<3>(
            1000u, 2000u, Index3D::Constant(-5000), Index3D::Constant(5000));
    for (const Index3D& index : random_indices) {
      const FloatingPoint update = TestFixture::getRandomUpdate();
      map.addToCellValue(index, update);
    }
    map.prune();

    // Instantiate the query accelerator
    QueryAccelerator query_accelerator(map);

    // Test all leaves
    map.forEachLeaf(
        [&query_accelerator](const OctreeIndex& index, FloatingPoint value) {
          EXPECT_NEAR(query_accelerator.getCellValue(index), value,
                      TestFixture::kNumericalNoise);
        });

    // Test random indices
    const IndexElement tree_height = map.getTreeHeight();
    auto random_offsets = TestFixture::template getRandomIndexVector<3>(
        Index3D::Constant(-10), Index3D::Constant(10), 2, 10);
    random_offsets.emplace_back(Index3D::Zero());
    OctreeIndex previous_index{};
    for (const Index3D& index : random_indices) {
      for (const Index3D& offset : random_offsets) {
        const IndexElement height =
            TestFixture::getRandomInteger(0, tree_height);
        const auto node_index =
            OctreeIndex{0, index + offset}.computeParentIndex(height);
        EXPECT_NEAR(query_accelerator.getCellValue(node_index),
                    map.getCellValue(node_index), TestFixture::kNumericalNoise)
            << "For node_index " << node_index.toString() << " (in block"
            << print::eigen::oneLine(map.indexToBlockIndex(node_index)) << ")"
            << ", previous index " << previous_index.toString() << " (in block"
            << print::eigen::oneLine(map.indexToBlockIndex(previous_index))
            << ")"
            << " tree height " << tree_height << " has block"
            << map.hasBlock(map.indexToBlockIndex(node_index));
        previous_index = node_index;
      }
    }
  }
}
}  // namespace wavemap
