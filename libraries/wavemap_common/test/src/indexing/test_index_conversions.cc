#include <gtest/gtest.h>
#include <wavemap_common/common.h>
#include <wavemap_common/test/fixture_base.h>
#include <wavemap_common/utils/eigen_format.h>

#include "wavemap_common/indexing/index_conversions.h"

namespace wavemap {
// TODO(victorr): Add tests for remaining index conversions:
//                - Index
//                - Point
template <typename DimT>
class IndexConversionsTest : public FixtureBase {
 protected:
  static constexpr NdtreeIndexElement kMaxHeight = 14;
  const typename NdtreeIndex<DimT::kDim>::Position kMinNdtreePositionIndex =
      NdtreeIndex<DimT::kDim>::Position::Zero();
  const typename NdtreeIndex<DimT::kDim>::Position kMaxNdtreePositionIndex =
      NdtreeIndex<DimT::kDim>::Position::Constant(int_math::exp2(kMaxHeight));
};

template <int dim>
struct DimType {
  static constexpr int kDim = dim;
};
using Dimensions = ::testing::Types<DimType<2>, DimType<3>>;
TYPED_TEST_SUITE(IndexConversionsTest, Dimensions, );

TYPED_TEST(IndexConversionsTest, NodeIndexConversions) {
  using NdtreeIndexType = NdtreeIndex<TypeParam::kDim>;
  using IndexType = Index<TypeParam::kDim>;
  using PointType = Point<TypeParam::kDim>;

  // Generate a combination of random and handpicked node indices for testing
  std::vector<NdtreeIndexType> random_indices =
      TestFixture::template getRandomNdtreeIndexVector<NdtreeIndexType>(
          TestFixture::kMinNdtreePositionIndex,
          TestFixture::kMaxNdtreePositionIndex, 1, TestFixture::kMaxHeight);
  random_indices.emplace_back(NdtreeIndexType{0, {0, 0}});
  for (NdtreeIndexElement index_height = 0;
       index_height < TestFixture::kMaxHeight; ++index_height) {
    for (NdtreeIndexElement index_x = -1; index_x <= 1; ++index_x) {
      for (NdtreeIndexElement index_y = -1; index_y <= 1; ++index_y) {
        random_indices.emplace_back(
            NdtreeIndexType{index_height, {index_x, index_y}});
      }
    }
  }

  // Test conversions from node indices to other coordinate and index types
  for (const NdtreeIndexType& node_index : random_indices) {
    // Compare to coordinate convention
    {
      const IndexType min_corner_index =
          convert::nodeIndexToMinCornerIndex(node_index);
      const IndexType min_corner_index_from_convention =
          node_index.position * int_math::exp2(node_index.height);
      EXPECT_EQ(min_corner_index, min_corner_index_from_convention)
          << "Quadtree converts node index " << node_index.toString()
          << " to regular index " << EigenFormat::oneLine(min_corner_index)
          << " does not match convention "
          << EigenFormat::oneLine(min_corner_index_from_convention);
    }
    {
      const IndexType max_corner_index =
          convert::nodeIndexToMaxCornerIndex(node_index);
      const IndexType max_corner_index_from_convention =
          (node_index.position + IndexType::Ones()) *
              int_math::exp2(node_index.height) -
          IndexType::Ones();
      EXPECT_EQ(max_corner_index, max_corner_index_from_convention)
          << "Quadtree converts node index " << node_index.toString()
          << " to regular index " << EigenFormat::oneLine(max_corner_index)
          << " does not match convention "
          << EigenFormat::oneLine(max_corner_index_from_convention);
    }

    // Roundtrip through regular indices (integer coordinates)
    {
      const IndexType min_corner_index =
          convert::nodeIndexToMinCornerIndex(node_index);
      const NdtreeIndexType roundtrip_node_index =
          convert::indexAndHeightToNodeIndex(min_corner_index,
                                             node_index.height);
      EXPECT_EQ(roundtrip_node_index, node_index)
          << "Going from node index " << node_index.toString()
          << " to min corner index " << EigenFormat::oneLine(min_corner_index)
          << " and back should yield the same node index, but got "
          << roundtrip_node_index.toString() << " instead.";
    }
    {
      const IndexType max_corner_index =
          convert::nodeIndexToMaxCornerIndex(node_index);
      const NdtreeIndexType roundtrip_node_index =
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
      const FloatingPoint random_min_cell_width =
          TestFixture::getRandomMinCellWidth();
      const PointType node_center =
          convert::nodeIndexToCenterPoint(node_index, random_min_cell_width);
      const NdtreeIndexType roundtrip_node_index = convert::pointToNodeIndex(
          node_center, random_min_cell_width, node_index.height);
      EXPECT_EQ(roundtrip_node_index, node_index)
          << "Going from node index " << node_index.toString()
          << " to node center " << EigenFormat::oneLine(node_center)
          << " and back should yield the same node index, but got "
          << roundtrip_node_index.toString() << " instead.";
    }
  }
}
}  // namespace wavemap