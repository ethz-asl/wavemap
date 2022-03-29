#include <gtest/gtest.h>

#include "wavemap_2d/common.h"
#include "wavemap_2d/indexing/index_conversions.h"
#include "wavemap_2d/indexing/quadtree_index.h"
#include "wavemap_2d/test/fixture_base.h"
#include "wavemap_2d/utils/eigen_format.h"

namespace wavemap_2d {
class QuadtreeIndexTest : public FixtureBase {
 protected:
  static constexpr NodeIndexElement kMaxDepth = 14;
  const Index kMinPossibleQuadtreeIndex = Index::Constant(0);
  const Index kMaxPossibleQuadtreeIndex =
      Index::Constant(constexpr_functions::exp2(QuadtreeIndexTest::kMaxDepth));

  FloatingPoint getRandomRootNodeWidth() {
    return random_number_generator_->getRandomRealNumber(0.1f, 1e3f);
  }
};

TEST_F(QuadtreeIndexTest, NodeIndexConversions) {
  // Generate a combination of random and handpicked node indices for testing
  std::vector<NodeIndex> random_indices = getRandomQuadtreeIndexVector(
      kMinPossibleQuadtreeIndex, kMaxPossibleQuadtreeIndex, 1, kMaxDepth);
  random_indices.emplace_back(NodeIndex{.depth = 0, .position = {0, 0}});
  for (NodeIndexElement index_depth = 1; index_depth < kMaxDepth;
       ++index_depth) {
    for (NodeIndexElement index_x = -1; index_x <= 1; ++index_x) {
      for (NodeIndexElement index_y = -1; index_y <= 1; ++index_y) {
        random_indices.emplace_back(
            NodeIndex{.depth = index_depth, .position = {index_x, index_y}});
      }
    }
  }

  // Test conversions from node indices to other coordinate and index types
  for (const NodeIndex& node_index : random_indices) {
    // Compare to coordinate convention
    {
      const Index index_from_quadtree =
          computeIndexFromNodeIndex(node_index, kMaxDepth);
      const Index index_from_convention = computeNearestIndexForScaledPoint(
          node_index.position.template cast<FloatingPoint>() *
          std::exp2(kMaxDepth - node_index.depth));
      EXPECT_EQ(index_from_quadtree, index_from_convention)
          << "Quadtree converts node index " << node_index.toString()
          << " to regular index " << EigenFormat::oneLine(index_from_quadtree)
          << " does not match convention "
          << EigenFormat::oneLine(index_from_convention);
    }

    // Roundtrip through regular indices (integer coordinates)
    {
      const Index index = computeIndexFromNodeIndex(node_index, kMaxDepth);
      const NodeIndex roundtrip_node_index =
          computeNodeIndexFromIndexAndDepth(index, node_index.depth, kMaxDepth);
      EXPECT_EQ(roundtrip_node_index, node_index)
          << "Going from node index " << node_index.toString()
          << " to regular index " << EigenFormat::oneLine(index)
          << " and back should yield the same node index, but got "
          << roundtrip_node_index.toString() << " instead.";
    }

    // Roundtrip through real valued coordinates
    {
      const FloatingPoint random_root_node_width = getRandomRootNodeWidth();
      const Point node_center = computeNodeCenterFromNodeIndex(
          node_index, random_root_node_width, node_index.depth);
      const NodeIndex roundtrip_node_index = computeNodeIndexFromCenter(
          node_center, random_root_node_width, node_index.depth);
      EXPECT_EQ(roundtrip_node_index, node_index)
          << "Going from node index " << node_index.toString()
          << " to node center " << EigenFormat::oneLine(node_center)
          << " and back should yield the same node index, but got "
          << roundtrip_node_index.toString() << " instead.";
    }
  }
}

TEST_F(QuadtreeIndexTest, ChildParentIndexing) {
  // Generate a combination of random and handpicked node indices for testing
  std::vector<NodeIndex> random_indices = getRandomQuadtreeIndexVector(
      kMinPossibleQuadtreeIndex, kMaxPossibleQuadtreeIndex, kMaxDepth,
      kMaxDepth);
  random_indices.emplace_back(NodeIndex{.depth = 0, .position = {0, 0}});
  for (NodeIndexElement index_depth = 1; index_depth < kMaxDepth;
       ++index_depth) {
    for (NodeIndexElement index_x = 0; index_x <= 1; ++index_x) {
      for (NodeIndexElement index_y = 0; index_y <= 1; ++index_y) {
        random_indices.emplace_back(
            NodeIndex{.depth = index_depth, .position = {index_x, index_y}});
      }
    }
  }

  // Test round trips between children and parents
  const NodeIndex root_index{.depth = 0, .position = {0, 0}};
  for (const NodeIndex& node_index : random_indices) {
    if (node_index.depth != 0) {
      const NodeIndex top_parent_index = node_index.computeParentIndex(0);
      EXPECT_EQ(top_parent_index, root_index)
          << "The index of the highest parent of node " << node_index.toString()
          << " is " << top_parent_index.toString()
          << " while it should equal the root node index "
          << root_index.toString() << ".";
    }
    for (NodeRelativeChildIndex relative_child_idx = 0;
         relative_child_idx < NodeIndex::kNumChildren; ++relative_child_idx) {
      const NodeIndex child_index =
          node_index.computeChildIndex(relative_child_idx);
      EXPECT_EQ(child_index.computeRelativeChildIndex(), relative_child_idx);
      EXPECT_EQ(child_index.computeParentIndex(), node_index);
    }
  }
}
}  // namespace wavemap_2d
