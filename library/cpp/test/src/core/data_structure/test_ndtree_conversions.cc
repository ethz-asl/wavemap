#include <algorithm>
#include <unordered_map>
#include <vector>

#include <gtest/gtest.h>

#include "wavemap/core/data_structure/chunked_ndtree/chunked_ndtree.h"
#include "wavemap/core/data_structure/linear_ndtree/linear_ndtree.h"
#include "wavemap/core/data_structure/ndtree/ndtree.h"
#include "wavemap/core/indexing/index_hashes.h"
#include "wavemap/test/fixture_base.h"
#include "wavemap/test/geometry_generator.h"

namespace wavemap {
template <typename NdtreeT>
class NdtreeConversionsTest : public FixtureBase, public GeometryGenerator {};

using NdtreeTypes =
    ::testing::Types<BinaryTree<int>, Quadtree<int>, Octree<int>,
                     ChunkedBinaryTree<int, 3>, ChunkedQuadtree<int, 3>,
                     ChunkedOctree<int, 3>, LinearBinaryTree<int>,
                     LinearQuadtree<int>, LinearOctree<int>>;
TYPED_TEST_SUITE(NdtreeConversionsTest, NdtreeTypes, );

TYPED_TEST(NdtreeConversionsTest, FromStandardNdtree) {
  using IndexType = typename TypeParam::IndexType;
  using PositionType = typename IndexType::Position;
  constexpr IndexElement kDim = IndexType::kDim;

  constexpr int kNumTrees = 10;
  constexpr int kNumValuesPerTree = 10;
  for (int nth_tree = 0; nth_tree < kNumTrees; ++nth_tree) {
    // Build a random source tree
    const int tree_height = TestFixture::getRandomNdtreeIndexHeight(2, 10);
    Ndtree<int, kDim> src_tree{tree_height};

    // Compute the min and max descendants of the root node at height 0
    const PositionType min_child_pos = convert::nodeIndexToMinCornerIndex(
        IndexType{tree_height, PositionType::Zero()});
    const PositionType max_child_pos = convert::nodeIndexToMaxCornerIndex(
        IndexType{tree_height, PositionType::Zero()});
    for (int nth_value = 0; nth_value < kNumValuesPerTree; ++nth_value) {
      // Generate a random index (that lies in the tree) and value
      const int random_height =
          TestFixture::getRandomNdtreeIndexHeight(0, tree_height);
      const auto random_index =
          GeometryGenerator::getRandomNdtreeIndex<IndexType>(
              min_child_pos, max_child_pos, 0, 0)
              .computeParentIndex(random_height);
      const int random_value = TestFixture::getRandomInteger(1, 100000);
      // Insert
      auto& data = src_tree.getOrAllocateNode(random_index).data();
      data = random_value;
    }

    // Construct our test tree by deep copying the src tree
    TypeParam test_tree = TypeParam::from(std::as_const(src_tree));

    // Test that the copied tree is identical to the original
    constexpr auto kOrder = TraversalOrder::kDepthFirstPreorder;
    auto src_it = src_tree.template getIterator<kOrder>().begin();
    auto test_it = test_tree.template getIterator<kOrder>().begin();
    const auto src_end = src_tree.template getIterator<kOrder>().end();
    const auto test_end = test_tree.template getIterator<kOrder>().end();
    size_t node_idx = 0;
    for (; src_it != src_end && test_it != test_end; ++src_it, ++test_it) {
      EXPECT_EQ(src_it->data(), test_it->data()) << "For node idx " << node_idx;
      ++node_idx;
    }
    EXPECT_EQ(src_it, src_end);
    EXPECT_EQ(test_it, test_end);
  }
}
}  // namespace wavemap
