#include <bitset>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>

#include "wavemap/core/common.h"
#include "wavemap/core/indexing/index_conversions.h"
#include "wavemap/core/utils/print/eigen.h"
#include "wavemap/test/eigen_utils.h"
#include "wavemap/test/fixture_base.h"
#include "wavemap/test/geometry_generator.h"

namespace wavemap {
// TODO(victorr): Add tests for remaining index conversions:
//                - Index
//                - Point
template <typename TypeParamT>
class IndexConversionsTest : public FixtureBase, public GeometryGenerator {
 protected:
  static constexpr IndexElement kMaxHeight = 14;
  const typename NdtreeIndex<TypeParamT::kDim>::Position
      kMinNdtreePositionIndex = NdtreeIndex<TypeParamT::kDim>::Position::Zero();
  const typename NdtreeIndex<TypeParamT::kDim>::Position
      kMaxNdtreePositionIndex =
          NdtreeIndex<TypeParamT::kDim>::Position::Constant(
              int_math::exp2(kMaxHeight));
};

template <int dim>
struct TypeParamTemplate {
  static constexpr int kDim = dim;
};
using TypeParams = ::testing::Types<TypeParamTemplate<2>, TypeParamTemplate<3>>;
TYPED_TEST_SUITE(IndexConversionsTest, TypeParams, );

TYPED_TEST(IndexConversionsTest, LinearIndexConversions) {
  constexpr int kDim = TypeParam::kDim;
  constexpr int kCellsPerSide = 16;
  const std::vector<Index<kDim>> test_indices =
      GeometryGenerator::getRandomIndexVector<kDim>(
          1000, 2000, Index<kDim>::Zero(),
          Index<kDim>::Constant(kCellsPerSide - 1));
  for (const Index<kDim>& index : test_indices) {
    const LinearIndex linear_index =
        convert::indexToLinearIndex<kCellsPerSide>(index);
    const Index<kDim> round_trip_index =
        convert::linearIndexToIndex<kCellsPerSide, kDim>(linear_index);
    EXPECT_EQ(round_trip_index, index)
        << "Expected index " << print::eigen::oneLine(index) << " but got "
        << print::eigen::oneLine(round_trip_index)
        << ", from intermediate linear_index " << linear_index;
  }
}

TYPED_TEST(IndexConversionsTest, NodeIndexConversions) {
  constexpr int kDim = TypeParam::kDim;

  // Generate a combination of random and handpicked node indices for testing
  std::vector<NdtreeIndex<kDim>> random_indices =
      GeometryGenerator::getRandomNdtreeIndexVector<NdtreeIndex<kDim>>(
          TestFixture::kMinNdtreePositionIndex,
          TestFixture::kMaxNdtreePositionIndex, 1, TestFixture::kMaxHeight);
  random_indices.emplace_back(NdtreeIndex<kDim>{0, {0, 0}});
  for (IndexElement index_height = 0; index_height < TestFixture::kMaxHeight;
       ++index_height) {
    for (IndexElement index_x = -1; index_x <= 1; ++index_x) {
      for (IndexElement index_y = -1; index_y <= 1; ++index_y) {
        random_indices.emplace_back(
            NdtreeIndex<kDim>{index_height, {index_x, index_y}});
      }
    }
  }

  // Test conversions from node indices to other coordinate and index types
  for (const NdtreeIndex<kDim>& node_index : random_indices) {
    // Compare to coordinate convention
    {
      const Index<kDim> min_corner_index =
          convert::nodeIndexToMinCornerIndex(node_index);
      const Index<kDim> min_corner_index_from_convention =
          node_index.position * int_math::exp2(node_index.height);
      EXPECT_EQ(min_corner_index, min_corner_index_from_convention)
          << "Quadtree converts node index " << node_index.toString()
          << " to regular index " << print::eigen::oneLine(min_corner_index)
          << " does not match convention "
          << print::eigen::oneLine(min_corner_index_from_convention);
    }
    {
      const Index<kDim> max_corner_index =
          convert::nodeIndexToMaxCornerIndex(node_index);
      const Index<kDim> max_corner_index_from_convention =
          (node_index.position + Index<kDim>::Ones()) *
              int_math::exp2(node_index.height) -
          Index<kDim>::Ones();
      EXPECT_EQ(max_corner_index, max_corner_index_from_convention)
          << "Quadtree converts node index " << node_index.toString()
          << " to regular index " << print::eigen::oneLine(max_corner_index)
          << " does not match convention "
          << print::eigen::oneLine(max_corner_index_from_convention);
    }

    // Roundtrip through regular indices (integer coordinates)
    {
      const Index<kDim> min_corner_index =
          convert::nodeIndexToMinCornerIndex(node_index);
      const NdtreeIndex<kDim> roundtrip_node_index =
          convert::indexAndHeightToNodeIndex(min_corner_index,
                                             node_index.height);
      EXPECT_EQ(roundtrip_node_index, node_index)
          << "Going from node index " << node_index.toString()
          << " to min corner index " << print::eigen::oneLine(min_corner_index)
          << " and back should yield the same node index, but got "
          << roundtrip_node_index.toString() << " instead.";
    }
    {
      const Index<kDim> max_corner_index =
          convert::nodeIndexToMaxCornerIndex(node_index);
      const NdtreeIndex<kDim> roundtrip_node_index =
          convert::indexAndHeightToNodeIndex(max_corner_index,
                                             node_index.height);
      EXPECT_EQ(roundtrip_node_index, node_index)
          << "Going from node index " << node_index.toString()
          << " to max corner index " << print::eigen::oneLine(max_corner_index)
          << " and back should yield the same node index, but got "
          << roundtrip_node_index.toString() << " instead.";
    }

    // Roundtrip through real valued coordinates
    {
      const FloatingPoint random_min_cell_width =
          TestFixture::getRandomMinCellWidth();
      const Point<kDim> node_center =
          convert::nodeIndexToCenterPoint(node_index, random_min_cell_width);
      const NdtreeIndex<kDim> roundtrip_node_index = convert::pointToNodeIndex(
          node_center, random_min_cell_width, node_index.height);
      EXPECT_EQ(roundtrip_node_index, node_index)
          << "Going from node index " << node_index.toString()
          << " to node center " << print::eigen::oneLine(node_center)
          << " and back should yield the same node index, but got "
          << roundtrip_node_index.toString() << " instead.";
    }

    // Roundtrip through AABB
    {
      const FloatingPoint random_min_cell_width =
          TestFixture::getRandomMinCellWidth();
      const auto node_aabb =
          convert::nodeIndexToAABB(node_index, random_min_cell_width);
      const auto min_corner =
          convert::nodeIndexToMinCorner(node_index, random_min_cell_width);
      const auto max_corner =
          convert::nodeIndexToMaxCorner(node_index, random_min_cell_width);
      const FloatingPoint cell_width =
          convert::heightToCellWidth(random_min_cell_width, node_index.height);
      // Skip excessively large cells where numerical noise is too large
      if ((min_corner.array().abs() < 1e2f || max_corner.array().abs() < 1e2f)
              .all()) {
        EXPECT_EIGEN_NEAR(node_aabb.min, min_corner, kEpsilon);
        EXPECT_EIGEN_NEAR(node_aabb.max, max_corner, kEpsilon);
        EXPECT_NEAR(node_aabb.template width<0>(), cell_width, kEpsilon);

        const Point<kDim> aabb_center = 0.5f * (node_aabb.min + node_aabb.max);
        const FloatingPoint aabb_width = node_aabb.template width<0>();
        const IndexElement aabb_height =
            convert::cellWidthToHeight(aabb_width, 1.f / random_min_cell_width);
        const NdtreeIndex<kDim> roundtrip_node_index =
            convert::pointToNodeIndex(aabb_center, random_min_cell_width,
                                      aabb_height);
        EXPECT_EQ(roundtrip_node_index, node_index)
            << "Going from node index " << node_index.toString() << " to aabb "
            << node_aabb.toString()
            << " and back should yield the same node index, but got "
            << roundtrip_node_index.toString() << " instead.";
      }
    }
  }
}

TYPED_TEST(IndexConversionsTest, MortonCodes) {
  constexpr int kDim = TypeParam::kDim;
  constexpr IndexElement kMaxCoordinate = morton::kMaxSingleCoordinate<kDim>;
  auto bitset_printer = [](Index<kDim> index) -> std::string {
    std::ostringstream ss;
    for (int idx = 0; idx < kDim; ++idx) {
      ss << std::bitset<64>(index[idx]) << "\n";
    }
    return ss.str();
  };
  // Test fully random indices
  const auto random_indices = GeometryGenerator::getRandomIndexVector<kDim>(
      2000, 2000, Index<kDim>::Zero(), Index<kDim>::Constant(kMaxCoordinate));
  for (const auto& index : random_indices) {
    const MortonIndex morton_code = convert::indexToMorton(index);
    const Index<kDim> round_trip_index =
        convert::mortonToIndex<kDim>(morton_code);
    EXPECT_EQ(round_trip_index, index)
        << "For original index " << print::eigen::oneLine(index) << "(\n"
        << bitset_printer(index) << "), morton code " << morton_code << " (\n"
        << std::bitset<64>(morton_code) << "\n) and round trip index "
        << print::eigen::oneLine(round_trip_index) << "(\n"
        << bitset_printer(round_trip_index) << ")";
  }
  // Test the last coordinate over its full range
  Index<kDim> index = GeometryGenerator::getRandomIndex<kDim>(
      Index<kDim>::Zero(), Index<kDim>::Constant(kMaxCoordinate));
  const auto test_range_bounds = {
      std::pair<size_t, size_t>{0, 1024},
      {kMaxCoordinate / 2 - 1024, kMaxCoordinate / 2 + 1024},
      {kMaxCoordinate - 1024, kMaxCoordinate}};
  for (const auto& [lower, upper] : test_range_bounds) {
    for (size_t idx = lower; idx <= upper; ++idx) {
      index[kDim - 1] = idx;
      const MortonIndex morton_code = convert::indexToMorton(index);
      const Index<kDim> round_trip_index =
          convert::mortonToIndex<kDim>(morton_code);
      EXPECT_EQ(round_trip_index, index)
          << "For original index " << print::eigen::oneLine(index) << "(\n"
          << bitset_printer(index) << "), morton code " << morton_code << " (\n"
          << std::bitset<64>(morton_code) << "\n) and round trip index "
          << print::eigen::oneLine(round_trip_index) << "(\n"
          << bitset_printer(round_trip_index) << ")";
    }
  }
}
}  // namespace wavemap
