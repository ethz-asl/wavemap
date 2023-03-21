#include <gtest/gtest.h>

#include "wavemap/data_structure/chunked_ndtree/chunked_ndtree.h"
#include "wavemap/data_structure/ndtree/ndtree.h"
#include "wavemap/test/fixture_base.h"

namespace wavemap {
template <typename NdtreeT>
using NdtreeTest = FixtureBase;

using NdtreeTypes =
    ::testing::Types<Ndtree<int, 1>, Ndtree<int, 2>, Ndtree<int, 3>,
                     ChunkedNdtree<int, 1, 3>, ChunkedNdtree<int, 2, 3>,
                     ChunkedNdtree<int, 3, 3>>;
TYPED_TEST_SUITE(NdtreeTest, NdtreeTypes, );

TYPED_TEST(NdtreeTest, AllocatingAndClearing) {
  using IndexType = typename TypeParam::IndexType;
  using PositionType = typename IndexType::Position;

  for (int repetition = 0; repetition < 1000; ++repetition) {
    const int test_height = TestFixture::getRandomNdtreeIndexHeight(4, 8);
    int tree_height = TestFixture::getRandomNdtreeIndexHeight(test_height, 14);
    TypeParam ndtree(tree_height);
    tree_height = ndtree.getMaxHeight();
    EXPECT_TRUE(ndtree.empty());

    const PositionType min_child_pos = convert::nodeIndexToMinCornerIndex(
        IndexType{tree_height, PositionType::Zero()});
    const PositionType max_child_pos = convert::nodeIndexToMaxCornerIndex(
        IndexType{tree_height, PositionType::Zero()});
    const auto random_index =
        TestFixture::template getRandomNdtreeIndex<IndexType>(
            min_child_pos, max_child_pos, 0, 0)
            .computeParentIndex(test_height);
    const bool index_is_within_root_node =
        (tree_height - random_index.height) < TypeParam::kChunkHeight;

    EXPECT_EQ(ndtree.hasNode(random_index), index_is_within_root_node)
        << random_index.toString() << " and tree height " << tree_height;
    ndtree.allocateNode(random_index);
    EXPECT_TRUE(ndtree.hasNode(random_index))
        << random_index.toString() << " and tree height " << tree_height;
    EXPECT_EQ(ndtree.empty(), index_is_within_root_node);

    ndtree.clear();
    EXPECT_EQ(ndtree.hasNode(random_index), index_is_within_root_node)
        << random_index.toString() << " and tree height " << tree_height;
    EXPECT_TRUE(ndtree.empty());
  }
}

TYPED_TEST(NdtreeTest, GettingAndSetting) {
  using IndexType = typename TypeParam::IndexType;
  using PositionType = typename IndexType::Position;

  for (int repetition = 0; repetition < 1000; ++repetition) {
    const int max_height = TestFixture::getRandomNdtreeIndexHeight(2, 14);
    TypeParam ndtree(max_height);

    const auto random_value =
        TestFixture::random_number_generator_->getRandomInteger(1, 100000);
    const auto random_index =
        TestFixture::template getRandomNdtreeIndex<IndexType>(
            PositionType::Zero(), PositionType::Ones(), 2, max_height);

    // Setter
    {
      auto* data = ndtree.getNodeData(random_index);
      ASSERT_NE(data, nullptr);
      *data = random_value;
    }

    // Getter
    {
      EXPECT_TRUE(ndtree.hasNode(random_index)) << random_index.toString();
      auto* data = ndtree.getNodeData(random_index);
      ASSERT_NE(data, nullptr);
      EXPECT_EQ(*data, random_value);
    }

    // Const getter
    const auto& ndtree_cref = ndtree;
    {
      EXPECT_TRUE(ndtree_cref.hasNode(random_index)) << random_index.toString();
      auto* data = ndtree_cref.getNodeData(random_index);
      ASSERT_NE(data, nullptr);
      EXPECT_EQ(*data, random_value);
    }
  }
}
}  // namespace wavemap
