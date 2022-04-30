#include <gtest/gtest.h>

#include "wavemap_2d/common.h"
#include "wavemap_2d/indexing/index_conversions.h"
#include "wavemap_2d/test/fixture_base.h"
#include "wavemap_2d/utils/eigen_format.h"

namespace wavemap_2d {
// TODO(victorr): Add tests for remaining index conversions:
//                - Index
//                - Point
class IndexConversionsTest : public FixtureBase {
 protected:
  static constexpr QuadtreeIndex::Element kMaxDepth = 14;
  const QuadtreeIndex::Position kMinQuadtreePositionIndex =
      QuadtreeIndex::Position::Zero();
  const QuadtreeIndex::Position kMaxQuadtreePositionIndex =
      QuadtreeIndex::Position::Constant(constexpr_functions::exp2(kMaxDepth));

  FloatingPoint getRandomRootNodeWidth() {
    return random_number_generator_->getRandomRealNumber(0.1f, 1e3f);
  }
};

TEST_F(IndexConversionsTest, NodeIndexConversions) {
  // Generate a combination of random and handpicked node indices for testing
  std::vector<QuadtreeIndex> random_indices =
      getRandomNdtreeIndexVector<QuadtreeIndex>(
          kMinQuadtreePositionIndex, kMaxQuadtreePositionIndex, 1, kMaxDepth);
  random_indices.emplace_back(QuadtreeIndex{.depth = 0, .position = {0, 0}});
  for (QuadtreeIndex::Element index_depth = 1; index_depth < kMaxDepth;
       ++index_depth) {
    for (QuadtreeIndex::Element index_x = -1; index_x <= 1; ++index_x) {
      for (QuadtreeIndex::Element index_y = -1; index_y <= 1; ++index_y) {
        random_indices.emplace_back(QuadtreeIndex{
            .depth = index_depth, .position = {index_x, index_y}});
      }
    }
  }

  // Test conversions from node indices to other coordinate and index types
  for (const QuadtreeIndex& node_index : random_indices) {
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
      const QuadtreeIndex roundtrip_node_index =
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
      const Point node_center =
          computeNodeCenterFromNodeIndex(node_index, random_root_node_width);
      const QuadtreeIndex roundtrip_node_index = computeNodeIndexFromCenter(
          node_center, random_root_node_width, node_index.depth);
      EXPECT_EQ(roundtrip_node_index, node_index)
          << "Going from node index " << node_index.toString()
          << " to node center " << EigenFormat::oneLine(node_center)
          << " and back should yield the same node index, but got "
          << roundtrip_node_index.toString() << " instead.";
    }
  }
}
}  // namespace wavemap_2d
