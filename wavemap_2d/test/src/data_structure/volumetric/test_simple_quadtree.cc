#include <gtest/gtest.h>

#include "wavemap_2d/common.h"
#include "wavemap_2d/data_structure/volumetric/cell_types/occupancy_cell.h"
#include "wavemap_2d/data_structure/volumetric/simple_quadtree.h"
#include "wavemap_2d/indexing/index.h"
#include "wavemap_2d/test/fixture_base.h"
#include "wavemap_2d/utils/eigen_format.h"

namespace wavemap_2d {
template <typename CellType>
class SimpleQuadtreeTest : public FixtureBase {
 protected:
  SimpleQuadtree<CellType> getRandomMap() {
    SimpleQuadtree<CellType> random_map(getRandomResolution());
    const Index min_index = -getRandomIndex().cwiseAbs();
    const Index max_index = getRandomIndex().cwiseAbs();
    random_map.addToCellValue(min_index, 0.f);
    random_map.addToCellValue(max_index, 0.f);
    for (Index index = min_index; index.x() <= max_index.x(); ++index.x()) {
      for (index.y() = min_index.y(); index.y() <= max_index.y(); ++index.y()) {
        random_map.addToCellValue(index, getRandomUpdate());
      }
    }
    return random_map;
  }
};

using CellTypes =
    ::testing::Types<UnboundedOccupancyCell, SaturatingOccupancyCell>;
TYPED_TEST_SUITE(SimpleQuadtreeTest, CellTypes, );

// NOTE: Insertion tests are performed as part of the test suite for the
//       VolumetricDataStructure interface.
// TODO(victorr): Test whether out of bounds accesses/insertions are handled
//                correctly (e.g. throw error or do nothing and print error).

TYPED_TEST(SimpleQuadtreeTest, Initialization) {
  const FloatingPoint random_resolution = TestFixture::getRandomResolution();
  SimpleQuadtree<TypeParam> map(random_resolution);
  EXPECT_EQ(map.getResolution(), random_resolution);
  EXPECT_TRUE(map.empty());
  EXPECT_EQ(map.size(), 0u);
}

TYPED_TEST(SimpleQuadtreeTest, IndexConversions) {
  SimpleQuadtree<TypeParam> map(TestFixture::getRandomResolution());
  std::vector<Index> random_indices = TestFixture::getRandomIndexVector(
      map.getMinPossibleIndex(), map.getMaxPossibleIndex());
  random_indices.template emplace_back(map.getMinPossibleIndex());
  random_indices.template emplace_back(map.getMaxPossibleIndex());
  for (const Index& index : random_indices) {
    EXPECT_EQ(map.nodeIndexToIndex(map.indexToNodeIndex(index)), index);
  }
}

TYPED_TEST(SimpleQuadtreeTest, Resizing) {
  SimpleQuadtree<TypeParam> map(TestFixture::getRandomResolution());
  ASSERT_TRUE(map.empty());
  ASSERT_EQ(map.size(), 0u);

  const Index kMinIndex{-2e3, -1e3};
  const Index kMaxIndex{1e3, 2e3};
  const std::vector<Index> random_indices =
      TestFixture::getRandomIndexVector(kMinIndex, kMaxIndex);

  const Index& first_random_index = random_indices[0];
  map.addToCellValue(first_random_index, 0.f);
  EXPECT_FALSE(map.empty());
  EXPECT_EQ(map.size(), map.getMaxDepth());

  Index min_index = first_random_index;
  Index max_index = first_random_index;
  for (auto index_it = ++random_indices.begin();
       index_it != random_indices.end(); ++index_it) {
    min_index = min_index.cwiseMin(*index_it);
    max_index = max_index.cwiseMax(*index_it);
    map.addToCellValue(*index_it, 0.f);
  }
  EXPECT_GE(map.size(), map.getMaxDepth());
  size_t max_unique_nodes = 0u;
  const size_t num_inserted_nodes = random_indices.size();
  for (QuadtreeIndex::Element depth = 0u; depth <= map.getMaxDepth(); ++depth) {
    const size_t max_unique_nodes_at_depth =
        int_math::exp2(kMapDimension * depth);
    if (max_unique_nodes_at_depth < num_inserted_nodes) {
      max_unique_nodes += max_unique_nodes_at_depth;
    } else {
      max_unique_nodes += num_inserted_nodes;
    }
  }
  EXPECT_LE(map.size(), max_unique_nodes);

  for (Index index = min_index; index.x() <= max_index.x(); ++index.x()) {
    for (index.y() = min_index.y(); index.y() <= max_index.y(); ++index.y()) {
      EXPECT_FLOAT_EQ(map.getCellValue(index), 0.f);
    }
  }

  map.clear();
  EXPECT_TRUE(map.empty());
  EXPECT_EQ(map.size(), 0u);
}

TYPED_TEST(SimpleQuadtreeTest, Pruning) {
  constexpr int kNumRepetitions = 10;
  for (int i = 0; i < kNumRepetitions; ++i) {
    SimpleQuadtree<TypeParam> map(TestFixture::getRandomResolution());
    const std::vector<Index> random_indices = TestFixture::getRandomIndexVector(
        map.getMinPossibleIndex(), map.getMaxPossibleIndex());

    // Check that zero values are inserted but removed after pruning
    ASSERT_TRUE(map.empty());
    for (const Index& random_index : random_indices) {
      map.setCellValue(random_index, 0.f);
    }
    EXPECT_FALSE(map.empty());
    map.prune();
    EXPECT_TRUE(map.empty());

    // Check that non-zero values are inserted but not removed after pruning
    ASSERT_TRUE(map.empty());
    for (const Index& random_index : random_indices) {
      map.setCellValue(random_index, 1.f);
    }
    EXPECT_FALSE(map.empty());
    map.prune();
    EXPECT_FALSE(map.empty());
  }
}
}  // namespace wavemap_2d
