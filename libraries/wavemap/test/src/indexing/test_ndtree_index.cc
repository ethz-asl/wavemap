#include <queue>

#include <gtest/gtest.h>

#include "wavemap/common.h"
#include "wavemap/indexing/index_conversions.h"
#include "wavemap/indexing/ndtree_index.h"
#include "wavemap/test/fixture_base.h"
#include "wavemap/utils/eigen_format.h"

namespace wavemap {
template <typename NdtreeIndexT>
class NdtreeIndexTest : public FixtureBase {
 protected:
  static constexpr NdtreeIndexElement kMaxHeight = 14;
  const typename NdtreeIndexT::Position kMinNdtreePositionIndex =
      NdtreeIndexT::Position::Zero();
  const typename NdtreeIndexT::Position kMaxNdtreePositionIndex =
      NdtreeIndexT::Position::Constant(int_math::exp2(kMaxHeight));

  FloatingPoint getRandomRootNodeWidth() const {
    return random_number_generator_->getRandomRealNumber(0.1f, 1e3f);
  }
};

using NdTreeIndexTypes =
    ::testing::Types<BinaryTreeIndex, QuadtreeIndex, OctreeIndex>;
TYPED_TEST_SUITE(NdtreeIndexTest, NdTreeIndexTypes, );

TYPED_TEST(NdtreeIndexTest, ChildParentIndexing) {
  // Generate a combination of random and handpicked node indices for testing
  std::vector<TypeParam> random_indices =
      TestFixture::template getRandomNdtreeIndexVector<TypeParam>(
          TestFixture::kMinNdtreePositionIndex,
          TestFixture::kMaxNdtreePositionIndex, 0, 0);
  for (auto& index : random_indices) {
    const int new_height =
        TestFixture::getRandomNdtreeIndexHeight(0, TestFixture::kMaxHeight);
    index = index.computeParentIndex(new_height);
  }
  random_indices.emplace_back(TypeParam{0, TypeParam::Position::Zero()});
  for (typename TypeParam::Element height_idx = 0;
       height_idx < TestFixture::kMaxHeight; ++height_idx) {
    for (int relative_child_idx = 0;
         relative_child_idx < TypeParam::kNumChildren; ++relative_child_idx) {
      typename TypeParam::Position position_index = TypeParam::Position::Zero();
      for (int dim_idx = 0; dim_idx < TypeParam::kDim; ++dim_idx) {
        position_index[dim_idx] = (relative_child_idx >> dim_idx) & 0b1;
      }
      random_indices.emplace_back(TypeParam{height_idx, position_index});
    }
  }

  // Test parent and child conversions
  for (const TypeParam& node_index : random_indices) {
    if (node_index.height <= 0) {
      // The node does not have children
      continue;
    }
    // Test round trips between children and parents
    const auto child_indices = node_index.computeChildIndices();
    for (typename TypeParam::RelativeChild relative_child_idx = 0;
         relative_child_idx < TypeParam::kNumChildren; ++relative_child_idx) {
      const TypeParam child_index =
          node_index.computeChildIndex(relative_child_idx);
      EXPECT_EQ(child_index.computeRelativeChildIndex(), relative_child_idx)
          << "The child's index relative to its parent \""
          << std::to_string(child_index.computeRelativeChildIndex())
          << "\" should match the requested relative child index \""
          << std::to_string(relative_child_idx) << "\".";
      EXPECT_EQ(child_index.computeRelativeChildIndex(
                    convert::nodeIndexToMorton(child_index), node_index.height),
                relative_child_idx)
          << "The child's index relative to its parent \""
          << std::to_string(child_index.computeRelativeChildIndex(
                 convert::nodeIndexToMorton(child_index), node_index.height))
          << "\" should match the requested relative child index \""
          << std::to_string(relative_child_idx) << "\".";
      EXPECT_EQ(child_index, child_indices[relative_child_idx])
          << "The child indices obtained through "
             "node_index.computeChildIndex(relative_child_idx) and "
             "node_index.computeChildIndices()[relative_child_idx] should be "
             "equal and appear in the same order. But for relative child idx "
          << std::to_string(relative_child_idx) << " we got "
          << child_index.toString() << " and "
          << child_indices[relative_child_idx].toString() << " instead.";
      EXPECT_EQ(child_index.computeParentIndex(), node_index)
          << "The current child's parent "
          << child_index.computeParentIndex().toString()
          << " should be equal to the parent it was derived from "
          << node_index.toString() << ".";
    }
  }
}

TYPED_TEST(NdtreeIndexTest, LinearOffsets) {
  // Generate a combination of random and handpicked node indices for testing
  std::vector<TypeParam> random_indices =
      TestFixture::template getRandomNdtreeIndexVector<TypeParam>(
          TestFixture::kMinNdtreePositionIndex,
          TestFixture::kMaxNdtreePositionIndex, 0, 0);
  random_indices.emplace_back(TypeParam{0, TypeParam::Position::Zero()});
  for (auto& index : random_indices) {
    const int test_height =
        TestFixture::getRandomNdtreeIndexHeight(index.height, 14);
    index = index.computeParentIndex(test_height);
  }

  // Check that the tree traversal distance equals the total BFS distance and
  // that the level traversal distance equals the BFS distance in the level of
  // the child node, for a perfect tree whose root is at the parent index
  constexpr int kMaxHeightDifference = 5;
  for (const auto& parent_index : random_indices) {
    std::queue<TypeParam> queue;
    queue.emplace(parent_index);
    int last_height = -1;
    LinearIndex level_traversal_bfs_distance = 0u;
    LinearIndex tree_traversal_bfs_distance = 0u;
    while (!queue.empty()) {
      const TypeParam& current_index = queue.front();
      if (current_index.height != last_height) {
        last_height = current_index.height;
        level_traversal_bfs_distance = 0u;
      }

      const MortonCode morton = convert::nodeIndexToMorton(current_index);
      EXPECT_EQ(TypeParam::computeLevelTraversalDistance(
                    morton, parent_index.height, current_index.height),
                level_traversal_bfs_distance);
      EXPECT_EQ(TypeParam::computeTreeTraversalDistance(
                    morton, parent_index.height, current_index.height),
                tree_traversal_bfs_distance);

      if (0 < current_index.height &&
          parent_index.height - current_index.height < kMaxHeightDifference) {
        for (const auto& child_index : current_index.computeChildIndices()) {
          queue.push(child_index);
        }
      }

      ++level_traversal_bfs_distance;
      ++tree_traversal_bfs_distance;
      queue.pop();
    }
    EXPECT_EQ(tree_traversal_bfs_distance,
              tree_math::perfect_tree::num_total_nodes<TypeParam::kDim>(
                  std::min(parent_index.height, kMaxHeightDifference) + 1))
        << "For kDim " << TypeParam::kDim << " parent_index.height "
        << parent_index.height << " and max height difference "
        << kMaxHeightDifference;
  }
}
}  // namespace wavemap
