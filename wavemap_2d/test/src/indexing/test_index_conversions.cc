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
  static constexpr QuadtreeIndex::Element kMaxHeight = 14;
  const QuadtreeIndex::Position kMinQuadtreePositionIndex =
      QuadtreeIndex::Position::Zero();
  const QuadtreeIndex::Position kMaxQuadtreePositionIndex =
      QuadtreeIndex::Position::Constant(int_math::exp2(kMaxHeight));
};

TEST_F(IndexConversionsTest, NodeIndexConversions) {
  // Generate a combination of random and handpicked node indices for testing
  std::vector<QuadtreeIndex> random_indices =
      getRandomNdtreeIndexVector<QuadtreeIndex>(
          kMinQuadtreePositionIndex, kMaxQuadtreePositionIndex, 1, kMaxHeight);
  random_indices.emplace_back(QuadtreeIndex{0, {0, 0}});
  for (QuadtreeIndex::Element index_height = 0; index_height < kMaxHeight;
       ++index_height) {
    for (QuadtreeIndex::Element index_x = -1; index_x <= 1; ++index_x) {
      for (QuadtreeIndex::Element index_y = -1; index_y <= 1; ++index_y) {
        random_indices.emplace_back(
            QuadtreeIndex{index_height, {index_x, index_y}});
      }
    }
  }

  // Test conversions from node indices to other coordinate and index types
  for (const QuadtreeIndex& node_index : random_indices) {
    // Compare to coordinate convention
    {
      const Index min_corner_index =
          convert::nodeIndexToMinCornerIndex(node_index);
      const Index min_corner_index_from_convention =
          node_index.position * int_math::exp2(node_index.height);
      EXPECT_EQ(min_corner_index, min_corner_index_from_convention)
          << "Quadtree converts node index " << node_index.toString()
          << " to regular index " << EigenFormat::oneLine(min_corner_index)
          << " does not match convention "
          << EigenFormat::oneLine(min_corner_index_from_convention);
    }
    {
      const Index max_corner_index =
          convert::nodeIndexToMaxCornerIndex(node_index);
      const Index max_corner_index_from_convention =
          (node_index.position + Index::Ones()) *
              int_math::exp2(node_index.height) -
          Index::Ones();
      EXPECT_EQ(max_corner_index, max_corner_index_from_convention)
          << "Quadtree converts node index " << node_index.toString()
          << " to regular index " << EigenFormat::oneLine(max_corner_index)
          << " does not match convention "
          << EigenFormat::oneLine(max_corner_index_from_convention);
    }

    // Roundtrip through regular indices (integer coordinates)
    {
      const Index min_corner_index =
          convert::nodeIndexToMinCornerIndex(node_index);
      const QuadtreeIndex roundtrip_node_index =
          convert::indexAndHeightToNodeIndex(min_corner_index,
                                             node_index.height);
      EXPECT_EQ(roundtrip_node_index, node_index)
          << "Going from node index " << node_index.toString()
          << " to min corner index " << EigenFormat::oneLine(min_corner_index)
          << " and back should yield the same node index, but got "
          << roundtrip_node_index.toString() << " instead.";
    }
    {
      const Index max_corner_index =
          convert::nodeIndexToMaxCornerIndex(node_index);
      const QuadtreeIndex roundtrip_node_index =
          convert::indexAndHeightToNodeIndex(max_corner_index,
                                             node_index.height);
      EXPECT_EQ(roundtrip_node_index, node_index)
          << "Going from node index " << node_index.toString()
          << " to max corner index " << EigenFormat::oneLine(max_corner_index)
          << " and back should yield the same node index, but got "
          << roundtrip_node_index.toString() << " instead.";
    }

    // Roundtrip through real valued coordinates
    {
      const FloatingPoint random_min_cell_width = getRandomMinCellWidth();
      const Point node_center =
          convert::nodeIndexToCenterPoint(node_index, random_min_cell_width);
      const QuadtreeIndex roundtrip_node_index = convert::pointToNodeIndex(
          node_center, random_min_cell_width, node_index.height);
      EXPECT_EQ(roundtrip_node_index, node_index)
          << "Going from node index " << node_index.toString()
          << " to node center " << EigenFormat::oneLine(node_center)
          << " and back should yield the same node index, but got "
          << roundtrip_node_index.toString() << " instead.";
    }
  }
}
}  // namespace wavemap_2d
