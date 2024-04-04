#include <gtest/gtest.h>

#include "wavemap/core/utils/neighbors/grid_neighborhood.h"
#include "wavemap/core/utils/print/eigen.h"
#include "wavemap/test/fixture_base.h"

namespace wavemap {
template <typename DimT>
class GridNeighborhoodTest : public FixtureBase {};

using Dimensions = ::testing::Types<std::integral_constant<int, 1>,
                                    std::integral_constant<int, 2>,
                                    std::integral_constant<int, 3>>;
TYPED_TEST_SUITE(GridNeighborhoodTest, Dimensions, );

TYPED_TEST(GridNeighborhoodTest, NumNeighborsForAdjacencyType) {
  constexpr int kDim = TypeParam::value;
  constexpr int kDimIdx = kDim - 1;
  const std::array num_vertices = {2, 4, 8};
  const std::array num_edges = {1, 4, 12};
  const std::array num_faces = {0, 1, 6};
  const std::array num_cubes = {0, 0, 1};
  EXPECT_EQ(GridNeighborhood<kDim>::numNeighbors(Adjacency::kSharedVertex),
            num_vertices[kDimIdx]);
  EXPECT_EQ(GridNeighborhood<kDim>::numNeighbors(Adjacency::kSharedEdge),
            num_edges[kDimIdx]);
  EXPECT_EQ(GridNeighborhood<kDim>::numNeighbors(Adjacency::kSharedFace),
            num_faces[kDimIdx]);
  EXPECT_EQ(GridNeighborhood<kDim>::numNeighbors(Adjacency::kSharedCube),
            num_cubes[kDimIdx]);
}

TYPED_TEST(GridNeighborhoodTest, NumNeighborsForAdjacencyMask) {
  constexpr int kDim = TypeParam::value;
  constexpr int kDimIdx = kDim - 1;
  const std::array num_no_adjacency_neighbors = {0, 0, 0};
  const std::array num_any_disjoint_adjacency_neighbors = {2, 8, 26};
  const std::array num_any_adjacency_neighbors = {3, 9, 27};
  EXPECT_EQ(GridNeighborhood<kDim>::numNeighbors(Adjacency::kNone),
            num_no_adjacency_neighbors[kDimIdx]);
  EXPECT_EQ(GridNeighborhood<kDim>::numNeighbors(Adjacency::kAnyDisjoint),
            num_any_disjoint_adjacency_neighbors[kDimIdx]);
  EXPECT_EQ(GridNeighborhood<kDim>::numNeighbors(Adjacency::kAny),
            num_any_adjacency_neighbors[kDimIdx]);
}

TYPED_TEST(GridNeighborhoodTest, IndexOffsets) {
  constexpr int kDim = TypeParam::value;

  // All neighbors with disjoint adjacency
  const auto all_disjoint_adjacent_offsets = GridNeighborhood<
      kDim>::template generateIndexOffsets<Adjacency::kAnyDisjoint>();
  EXPECT_EQ(all_disjoint_adjacent_offsets.size(),
            GridNeighborhood<kDim>::numNeighbors(Adjacency::kAnyDisjoint));
  for (const auto& offset :
       Grid<kDim>(Index<kDim>::Constant(-1), Index<kDim>::Constant(1))) {
    if (!offset.isZero()) {
      EXPECT_EQ(std::count(all_disjoint_adjacent_offsets.begin(),
                           all_disjoint_adjacent_offsets.end(), offset),
                1)
          << "For offset " << print::eigen::oneLine(offset);
    }
  }

  // All neighbors, including the node itself (i.e. no offset)
  const auto all_adjacent_offsets =
      GridNeighborhood<kDim>::template generateIndexOffsets<Adjacency::kAny>();
  EXPECT_EQ(all_adjacent_offsets.size(),
            GridNeighborhood<kDim>::numNeighbors(Adjacency::kAny));
  for (const auto& offset :
       Grid<kDim>(Index<kDim>::Constant(-1), Index<kDim>::Constant(1))) {
    EXPECT_EQ(std::count(all_adjacent_offsets.begin(),
                         all_adjacent_offsets.end(), offset),
              1)
        << "For offset " << print::eigen::oneLine(offset);
  }
}
}  // namespace wavemap
