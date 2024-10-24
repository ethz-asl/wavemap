#include <algorithm>
#include <bitset>
#include <queue>
#include <vector>

#include <gtest/gtest.h>

#include "wavemap/core/common.h"
#include "wavemap/core/indexing/index_conversions.h"
#include "wavemap/core/indexing/ndtree_index.h"
#include "wavemap/core/utils/print/eigen.h"
#include "wavemap/test/fixture_base.h"
#include "wavemap/test/geometry_generator.h"

namespace wavemap {
template <typename NdtreeIndexT>
class NdtreeIndexTest : public FixtureBase, public GeometryGenerator {
 protected:
  static constexpr IndexElement kMaxHeight = 14;
  const typename NdtreeIndexT::Position kMinNdtreePositionIndex =
      NdtreeIndexT::Position::Zero();
  const typename NdtreeIndexT::Position kMaxNdtreePositionIndex =
      NdtreeIndexT::Position::Constant(int_math::exp2(kMaxHeight));

  FloatingPoint getRandomRootNodeWidth() { return getRandomFloat(0.1f, 1e3f); }
};

using NdTreeIndexTypes =
    ::testing::Types<BinaryTreeIndex, QuadtreeIndex, OctreeIndex>;
TYPED_TEST_SUITE(NdtreeIndexTest, NdTreeIndexTypes, );

TYPED_TEST(NdtreeIndexTest, ChildParentIndexing) {
  // Generate a combination of random and handpicked node indices for testing
  std::vector<TypeParam> random_indices =
      GeometryGenerator::getRandomNdtreeIndexVector<TypeParam>(
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

TYPED_TEST(NdtreeIndexTest, LastCommonAncestor) {
  // Generate a combination of random and handpicked node indices for testing
  constexpr IndexElement kMaxTreeHeight =
      morton::kMaxTreeHeight<TypeParam::kDim>;
  constexpr IndexElement kNumChildren = TypeParam::kNumChildren;
  std::vector<TypeParam> random_indices =
      GeometryGenerator::getRandomNdtreeIndexVector<TypeParam>(
          TestFixture::kMinNdtreePositionIndex,
          TestFixture::kMaxNdtreePositionIndex, 0, 0);
  for (auto& index : random_indices) {
    const int test_height =
        TestFixture::getRandomNdtreeIndexHeight(index.height, 14);
    index = index.computeParentIndex(test_height);
  }
  random_indices.emplace_back(TypeParam{0, TypeParam::Position::Zero()});
  random_indices.emplace_back(
      TypeParam{kMaxTreeHeight, TypeParam::Position::Zero()});

  for (const auto& index : random_indices) {
    const MortonIndex morton = convert::nodeIndexToMorton(index);
    // Test identity
    EXPECT_EQ(TypeParam::computeLastCommonAncestorHeight(morton, index.height,
                                                         morton, index.height),
              index.height);

    // Test all ancestors up to max height
    for (auto parent_index = index.computeParentIndex();
         parent_index.height < kMaxTreeHeight;
         parent_index = parent_index.computeParentIndex()) {
      const MortonIndex parent_morton =
          convert::nodeIndexToMorton(parent_index);
      EXPECT_EQ(TypeParam::computeLastCommonAncestorHeight(
                    morton, index.height, parent_morton, parent_index.height),
                parent_index.height)
          << "For index " << index.toString() << " with morton code "
          << std::bitset<64>(morton) << ", parent index "
          << parent_index.toString() << " with morton code "
          << std::bitset<64>(parent_morton);
    }

    // Test descendants down to height 0
    for (auto child_index = index; 0 < child_index.height;) {
      if (31 <= index.height - child_index.height) {
        // Beyond this point the height difference is so large that the integers
        // would overflow
        break;
      }
      const NdtreeIndexRelativeChild relative_child_idx =
          TestFixture::getRandomInteger(0, kNumChildren - 1);
      child_index = child_index.computeChildIndex(relative_child_idx);
      const MortonIndex child_morton = convert::nodeIndexToMorton(child_index);
      EXPECT_EQ(TypeParam::computeLastCommonAncestorHeight(
                    morton, index.height, child_morton, child_index.height),
                index.height)
          << "For index " << index.toString() << " with morton code "
          << std::bitset<64>(morton) << ", child index "
          << child_index.toString() << " with morton code "
          << std::bitset<64>(child_morton);
    }

    // Test offsets
    if (index.height != kMaxTreeHeight) {
      for (const auto& offset : {TypeParam::Position::Constant(-1),
                                 TypeParam::Position::Constant(1)}) {
        const TypeParam index_offset{index.height, index.position + offset};
        const MortonIndex morton_offset =
            convert::nodeIndexToMorton(index_offset);
        EXPECT_GT(TypeParam::computeLastCommonAncestorHeight(
                      morton, index.height, morton_offset, index_offset.height),
                  index.height);
      }
    }
  }
}

TYPED_TEST(NdtreeIndexTest, LinearOffsets) {
  // Generate a combination of random and handpicked node indices for testing
  std::vector<TypeParam> random_indices =
      GeometryGenerator::getRandomNdtreeIndexVector<TypeParam>(
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

      const MortonIndex morton = convert::nodeIndexToMorton(current_index);
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
