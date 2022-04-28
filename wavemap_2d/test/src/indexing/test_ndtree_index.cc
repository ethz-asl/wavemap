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
      NdtreeIndexT::Position::Constant(constexpr_functions::exp2(kMaxDepth));

  FloatingPoint getRandomRootNodeWidth() {
    return random_number_generator_->getRandomRealNumber(0.1f, 1e3f);
  }

  std::vector<NdtreeIndexT> getRandomNdtreeIndexVector(
      typename NdtreeIndexT::Position min_index,
      typename NdtreeIndexT::Position max_index,
      typename NdtreeIndexT::Element min_depth,
      typename NdtreeIndexT::Element max_depth, size_t min_num_indices = 2u,
      size_t max_num_indices = 100u) const {
    CHECK((min_index.array() <= max_index.array()).all());
    CHECK_LE(min_depth, max_depth);

    const size_t num_indices = random_number_generator_->getRandomInteger(
        min_num_indices, max_num_indices);

    std::vector<NdtreeIndexT> random_indices(num_indices);
    std::generate(random_indices.begin(), random_indices.end(), [&]() {
      typename NdtreeIndexT::Position position_index;
      for (int i = 0; i < NdtreeIndexT::kDim; ++i) {
        position_index[i] = getRandomIndexElement(min_index[i], max_index[i]);
      }
      return NdtreeIndexT{
          .depth = getRandomQuadtreeIndexDepth(min_depth, max_depth),
          .position = position_index};
    });
    return random_indices;
  }
};

using Dimensions =
    ::testing::Types<BinaryTreeIndex, QuadtreeIndex, OctreeIndex>;
TYPED_TEST_SUITE(NdtreeIndexTest, Dimensions);

TYPED_TEST(NdtreeIndexTest, ChildParentIndexing) {
  // Generate a combination of random and handpicked node indices for testing
  std::vector<TypeParam> random_indices =
      TestFixture::getRandomNdtreeIndexVector(
          TestFixture::kMinNdtreePositionIndex,
          TestFixture::kMaxNdtreePositionIndex, TestFixture::kMaxDepth,
          TestFixture::kMaxDepth);
  random_indices.emplace_back(TypeParam{.depth = 0, .position = {0, 0}});
  for (typename TypeParam::Element index_depth = 1;
       index_depth < TestFixture::kMaxDepth; ++index_depth) {
    for (typename TypeParam::Element index_x = 0; index_x <= 1; ++index_x) {
      for (typename TypeParam::Element index_y = 0; index_y <= 1; ++index_y) {
        random_indices.emplace_back(
            TypeParam{.depth = index_depth, .position = {index_x, index_y}});
      }
    }
  }

  // Test parent <-> child conversions
  const TypeParam root_index{.depth = 0, .position = {0, 0}};
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
