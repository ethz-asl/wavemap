#include <gtest/gtest.h>
#include <wavemap/common.h>
#include <wavemap/map/hashed_wavelet_octree.h>
#include <wavemap/map/wavelet_octree.h>
#include <wavemap/test/config_generator.h>
#include <wavemap/test/fixture_base.h>
#include <wavemap/test/geometry_generator.h>

#include "wavemap/utils/query/query_accelerator.h"

namespace wavemap {
class QueryAcceleratorTest : public FixtureBase,
                             public GeometryGenerator,
                             public ConfigGenerator {};

TEST_F(QueryAcceleratorTest, Equivalence) {
  constexpr int kNumRepetitions = 3;
  for (int i = 0; i < kNumRepetitions; ++i) {
    // Create a random map
    const auto config =
        ConfigGenerator::getRandomConfig<HashedWaveletOctree::Config>();
    HashedWaveletOctree map(config);
    const std::vector<Index3D> random_indices =
        GeometryGenerator::getRandomIndexVector<3>(
            10000u, 20000u, Index3D::Constant(-5000), Index3D::Constant(5000));
    for (const Index3D& index : random_indices) {
      const FloatingPoint update = getRandomUpdate();
      map.addToCellValue(index, update);
    }
    map.prune();

    // Instantiate the query accelerator
    QueryAccelerator query_accelerator(map);

    // Test all leaves
    map.forEachLeaf(
        [&query_accelerator](const OctreeIndex& index, FloatingPoint value) {
          EXPECT_NEAR(query_accelerator.getCellValue(index), value, kEpsilon);
        });

    // Test random indices
    const IndexElement tree_height = map.getTreeHeight();
    auto random_offsets = getRandomIndexVector<3>(Index3D::Constant(-10),
                                                  Index3D::Constant(10), 2, 10);
    random_offsets.emplace_back(Index3D::Zero());
    OctreeIndex previous_index{};
    for (const Index3D& index : random_indices) {
      for (const Index3D& offset : random_offsets) {
        const IndexElement height = getRandomInteger(0, tree_height);
        const auto node_index =
            OctreeIndex{0, index + offset}.computeParentIndex(height);
        EXPECT_NEAR(query_accelerator.getCellValue(node_index),
                    map.getCellValue(node_index), kEpsilon)
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
