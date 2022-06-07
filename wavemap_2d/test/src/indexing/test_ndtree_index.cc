#include <gtest/gtest.h>

#include "wavemap_2d/common.h"
#include "wavemap_2d/indexing/index_conversions.h"
#include "wavemap_2d/indexing/ndtree_index.h"
#include "wavemap_2d/test/fixture_base.h"
#include "wavemap_2d/utils/eigen_format.h"

namespace wavemap_2d {
template <typename NdtreeIndexT>
class NdtreeIndexTest : public FixtureBase {
 protected:
  static constexpr typename NdtreeIndexT::Element kMaxDepth = 14;
  const typename NdtreeIndexT::Position kMinNdtreePositionIndex =
      NdtreeIndexT::Position::Zero();
  const typename NdtreeIndexT::Position kMaxNdtreePositionIndex =
      NdtreeIndexT::Position::Constant(int_math::exp2(kMaxDepth));

  FloatingPoint getRandomRootNodeWidth() const {
    return random_number_generator_->getRandomRealNumber(0.1f, 1e3f);
  }
};

using Dimensions =
    ::testing::Types<BinaryTreeIndex, QuadtreeIndex, OctreeIndex>;
TYPED_TEST_SUITE(NdtreeIndexTest, Dimensions, );

TYPED_TEST(NdtreeIndexTest, ChildParentIndexing) {
  // Generate a combination of random and handpicked node indices for testing
  std::vector<TypeParam> random_indices =
      TestFixture::template getRandomNdtreeIndexVector<TypeParam>(
          TestFixture::kMinNdtreePositionIndex,
          TestFixture::kMaxNdtreePositionIndex, TestFixture::kMaxDepth,
          TestFixture::kMaxDepth);
  random_indices.emplace_back(TypeParam{0, TypeParam::Position::Zero()});
  for (typename TypeParam::Element depth_idx = 1;
       depth_idx < TestFixture::kMaxDepth; ++depth_idx) {
    for (int relative_child_idx = 0;
         relative_child_idx < TypeParam::kNumChildren; ++relative_child_idx) {
      typename TypeParam::Position position_index = TypeParam::Position::Zero();
      for (int dim_idx = 0; dim_idx < TypeParam::kDim; ++dim_idx) {
        position_index[dim_idx] = (relative_child_idx >> dim_idx) & 0b1;
      }
      random_indices.emplace_back(TypeParam{depth_idx, position_index});
    }
  }

  // Test parent and child conversions
  for (const TypeParam& node_index : random_indices) {
    // Check all parents from the random node up to the root of the tree
    const std::vector<TypeParam> parent_index_list =
        node_index.computeParentIndices();
    TypeParam parent_index_iterative = node_index;
    for (typename TypeParam::Element depth = node_index.depth - 1; 0 <= depth;
         --depth) {
      parent_index_iterative = parent_index_iterative.computeParentIndex();
      EXPECT_EQ(parent_index_list[depth], parent_index_iterative);
      EXPECT_EQ(node_index.computeParentIndex(depth), parent_index_iterative);
    }
    if (node_index.depth == 0) {
      EXPECT_EQ(parent_index_list.size(), 0)
          << "The list of parent indices for the root node should be empty.";
    }

    // Test round trips between children and parents
    const std::vector<TypeParam> child_indices =
        node_index.computeChildIndices();
    for (typename TypeParam::RelativeChild relative_child_idx = 0;
         relative_child_idx < TypeParam::kNumChildren; ++relative_child_idx) {
      const TypeParam child_index =
          node_index.computeChildIndex(relative_child_idx);
      EXPECT_EQ(child_index.computeRelativeChildIndex(), relative_child_idx)
          << "The child's index relative to its parent \""
          << std::to_string(child_index.computeRelativeChildIndex())
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
}  // namespace wavemap_2d
