#include <gtest/gtest.h>
#include <wavemap_common/common.h>
#include <wavemap_common/data_structure/volumetric_cell_types/occupancy_cell.h>
#include <wavemap_common/test/fixture_base.h>

#include "wavemap_2d/data_structure/simple_quadtree.h"

namespace wavemap {
template <typename CellType>
class SimpleQuadtreeTest : public FixtureBase {
 protected:
  SimpleQuadtree<CellType> getRandomMap() {
    SimpleQuadtree<CellType> random_map(getRandomMinCellWidth());
    const Index2D min_index = -getRandomIndex<2>().cwiseAbs();
    const Index2D max_index = getRandomIndex<2>().cwiseAbs();
    random_map.addToCellValue(min_index, 0.f);
    random_map.addToCellValue(max_index, 0.f);
    for (Index2D index = min_index; index.x() <= max_index.x(); ++index.x()) {
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
  const FloatingPoint random_min_cell_width =
      TestFixture::getRandomMinCellWidth();
  SimpleQuadtree<TypeParam> map(random_min_cell_width);
  EXPECT_EQ(map.getMinCellWidth(), random_min_cell_width);
  EXPECT_TRUE(map.empty());
  EXPECT_EQ(map.size(), 1u);  // Contains exactly 1 node (the root)
}

TYPED_TEST(SimpleQuadtreeTest, IndexConversions) {
  SimpleQuadtree<TypeParam> map(TestFixture::getRandomMinCellWidth());
  std::vector<Index2D> random_indices = TestFixture::getRandomIndexVector(
      map.getMinPossibleIndex(), map.getMaxPossibleIndex());
  random_indices.template emplace_back(map.getMinPossibleIndex());
  random_indices.template emplace_back(map.getMaxPossibleIndex());
  for (const Index2D& index : random_indices) {
    // NOTE: Since the Index is converted into an internal NodeIndex at height
    //       0, the NodeIndex's min and max corner Indexes should be the same.
    EXPECT_EQ(map.toExternalIndex(
                  convert::nodeIndexToMinCornerIndex(map.toInternal(index))),
              index);
    EXPECT_EQ(map.toExternalIndex(
                  convert::nodeIndexToMaxCornerIndex(map.toInternal(index))),
              index);
  }
}

TYPED_TEST(SimpleQuadtreeTest, Resizing) {
  SimpleQuadtree<TypeParam> map(TestFixture::getRandomMinCellWidth());
  ASSERT_TRUE(map.empty());
  ASSERT_EQ(map.size(), 1u);

  const Index2D kMinIndex{-2e3, -1e3};
  const Index2D kMaxIndex{1e3, 2e3};
  const std::vector<Index2D> random_indices =
      TestFixture::template getRandomIndexVector<2>(kMinIndex, kMaxIndex);

  const Index2D& first_random_index = random_indices[0];
  map.addToCellValue(first_random_index, 0.f);
  EXPECT_FALSE(map.empty());
  EXPECT_EQ(map.size(), map.kMaxHeight + 1);

  Index2D min_index = first_random_index;
  Index2D max_index = first_random_index;
  for (auto index_it = ++random_indices.begin();
       index_it != random_indices.end(); ++index_it) {
    min_index = min_index.cwiseMin(*index_it);
    max_index = max_index.cwiseMax(*index_it);
    map.addToCellValue(*index_it, 0.f);
  }
  EXPECT_GE(map.size(), map.kMaxHeight);
  size_t max_unique_nodes = 0u;
  const size_t num_inserted_nodes = random_indices.size();
  for (QuadtreeIndex::Element depth = 0u; depth <= map.kMaxHeight; ++depth) {
    constexpr int kMapDimension = 2;
    const size_t max_unique_nodes_at_depth =
        int_math::exp2(kMapDimension * depth);
    if (max_unique_nodes_at_depth < num_inserted_nodes) {
      max_unique_nodes += max_unique_nodes_at_depth;
    } else {
      max_unique_nodes += num_inserted_nodes;
    }
  }
  EXPECT_LE(map.size(), max_unique_nodes);

  for (Index2D index = min_index; index.x() <= max_index.x(); ++index.x()) {
    for (index.y() = min_index.y(); index.y() <= max_index.y(); ++index.y()) {
      EXPECT_FLOAT_EQ(map.getCellValue(index), 0.f);
    }
  }

  map.clear();
  EXPECT_TRUE(map.empty());
  EXPECT_EQ(map.size(), 1u);
}

TYPED_TEST(SimpleQuadtreeTest, Pruning) {
  constexpr int kNumRepetitions = 10;
  for (int i = 0; i < kNumRepetitions; ++i) {
    SimpleQuadtree<TypeParam> map(TestFixture::getRandomMinCellWidth());
    const std::vector<Index2D> random_indices =
        TestFixture::template getRandomIndexVector<2>(
            map.getMinPossibleIndex(), map.getMaxPossibleIndex());

    // Check that zero values are inserted but removed after pruning
    ASSERT_TRUE(map.empty());
    for (const Index2D& random_index : random_indices) {
      map.setCellValue(random_index, 0.f);
    }
    EXPECT_FALSE(map.empty());
    map.prune();
    EXPECT_TRUE(map.empty());

    // Check that non-zero values are inserted but not removed after pruning
    ASSERT_TRUE(map.empty());
    for (const Index2D& random_index : random_indices) {
      map.setCellValue(random_index, 1.f);
    }
    EXPECT_FALSE(map.empty());
    map.prune();
    EXPECT_FALSE(map.empty());
  }
}
}  // namespace wavemap
