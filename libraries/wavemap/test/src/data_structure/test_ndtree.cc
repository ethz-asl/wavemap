#include <gtest/gtest.h>

#include "wavemap/data_structure/chunked_ndtree/chunked_ndtree.h"
#include "wavemap/data_structure/ndtree/ndtree.h"
#include "wavemap/indexing/index_hashes.h"
#include "wavemap/test/fixture_base.h"
#include "wavemap/test/geometry_generator.h"

namespace wavemap {
template <typename NdtreeT>
class NdtreeTest : public FixtureBase, public GeometryGenerator {};

using NdtreeTypes =
    ::testing::Types<Ndtree<int, 1>, Ndtree<int, 2>, Ndtree<int, 3>,
                     ChunkedNdtree<int, 1, 3>, ChunkedNdtree<int, 2, 3>,
                     ChunkedNdtree<int, 3, 3>>;
TYPED_TEST_SUITE(NdtreeTest, NdtreeTypes, );

TYPED_TEST(NdtreeTest, AllocatingAndClearing) {
  using IndexType = typename TypeParam::IndexType;
  using PositionType = typename IndexType::Position;
  constexpr int kNumRepetitions = 1000;

  for (int repetition = 0; repetition < kNumRepetitions; ++repetition) {
    const int test_height = TestFixture::getRandomNdtreeIndexHeight(4, 8);
    int tree_height = TestFixture::getRandomNdtreeIndexHeight(test_height, 14);
    TypeParam ndtree(tree_height);
    tree_height = ndtree.getMaxHeight();
    EXPECT_TRUE(ndtree.empty());

    // Generate a random index that lies in the tree
    const PositionType min_child_pos = convert::nodeIndexToMinCornerIndex(
        IndexType{tree_height, PositionType::Zero()});
    const PositionType max_child_pos = convert::nodeIndexToMaxCornerIndex(
        IndexType{tree_height, PositionType::Zero()});
    const auto random_index =
        GeometryGenerator::getRandomNdtreeIndex<IndexType>(min_child_pos,
                                                           max_child_pos, 0, 0)
            .computeParentIndex(test_height);
    const bool index_is_inside_root_chunk_node =
        (tree_height - random_index.height) < TypeParam::kChunkHeight;

    EXPECT_EQ(ndtree.hasNode(random_index), index_is_inside_root_chunk_node)
        << random_index.toString() << " and tree height " << tree_height;
    ndtree.getOrAllocateNode(random_index);
    EXPECT_TRUE(ndtree.hasNode(random_index))
        << random_index.toString() << " and tree height " << tree_height;
    EXPECT_EQ(ndtree.empty(), index_is_inside_root_chunk_node);

    ndtree.clear();
    EXPECT_EQ(ndtree.hasNode(random_index), index_is_inside_root_chunk_node)
        << random_index.toString() << " and tree height " << tree_height;
    EXPECT_TRUE(ndtree.empty());
  }
}

TYPED_TEST(NdtreeTest, GettingAndSetting) {
  using IndexType = typename TypeParam::IndexType;
  using PositionType = typename IndexType::Position;
  constexpr int kDim = IndexType::kDim;

  constexpr int kNumTrees = 10;
  constexpr int kNumValuesPerTree = 10;
  for (int nth_tree = 0; nth_tree < kNumTrees; ++nth_tree) {
    const int tree_height = TestFixture::getRandomNdtreeIndexHeight(2, 14);
    TypeParam ndtree(tree_height);

    // Compute the min and max descendants of the root node at height 0
    const PositionType min_child_pos = convert::nodeIndexToMinCornerIndex(
        IndexType{tree_height, PositionType::Zero()});
    const PositionType max_child_pos = convert::nodeIndexToMaxCornerIndex(
        IndexType{tree_height, PositionType::Zero()});

    std::unordered_map<IndexType, int, NdtreeIndexHash<kDim>> inserted_values;
    for (int nth_value = 0; nth_value < kNumValuesPerTree; ++nth_value) {
      // Generate a random index (that lies in the tree) and value
      const int random_height =
          TestFixture::getRandomNdtreeIndexHeight(0, tree_height);
      const auto random_index =
          GeometryGenerator::getRandomNdtreeIndex<IndexType>(
              min_child_pos, max_child_pos, 0, 0)
              .computeParentIndex(random_height);
      const int random_value = TestFixture::getRandomInteger(1, 100000);

      // Avoid inserting multiple values at the same index
      if (inserted_values.count(random_index)) {
        continue;
      }
      inserted_values.emplace(random_index, random_value);

      // Insert
      auto& data = ndtree.getOrAllocateNode(random_index).data();
      data = random_value;
    }

    // Test regular getter
    for (const auto& [index, value] : inserted_values) {
      EXPECT_TRUE(ndtree.hasNode(index)) << "At index " << index.toString();
      auto node = ndtree.getNode(index);
      ASSERT_TRUE(node) << "At index " << index.toString();
      if (node) {
        typename TypeParam::NodeDataType data;
        if constexpr (TypeParam::kChunkHeight == 1) {
          data = node->data();
        } else {
          data = node.data();
        }
        EXPECT_EQ(data, value) << "At index " << index.toString();
      }
    }

    // Test const getter
    const auto& ndtree_cref = ndtree;
    for (const auto& [index, value] : inserted_values) {
      EXPECT_TRUE(ndtree_cref.hasNode(index))
          << "At index " << index.toString();
      auto node = ndtree_cref.getNode(index);
      ASSERT_TRUE(node) << "At index " << index.toString();
      if (node) {
        typename TypeParam::NodeDataType data;
        if constexpr (TypeParam::kChunkHeight == 1) {
          data = node->data();
        } else {
          data = node.data();
        }
        EXPECT_EQ(data, value) << "At index " << index.toString();
      }
    }
  }
}
}  // namespace wavemap
