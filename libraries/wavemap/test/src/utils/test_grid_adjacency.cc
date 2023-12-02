#include <gtest/gtest.h>

#include "wavemap/test/fixture_base.h"
#include "wavemap/test/geometry_generator.h"
#include "wavemap/utils/neighbors/grid_adjacency.h"
#include "wavemap/utils/neighbors/grid_neighborhood.h"

namespace wavemap {
template <typename DimT>
class GridAdjacencyTest : public FixtureBase, public GeometryGenerator {};

using Dimensions = ::testing::Types<std::integral_constant<int, 1>,
                                    std::integral_constant<int, 2>,
                                    std::integral_constant<int, 3>>;
TYPED_TEST_SUITE(GridAdjacencyTest, Dimensions, );

TYPED_TEST(GridAdjacencyTest, SharedVertexAdjacency) {
  constexpr int kDim = TypeParam::value;
  using IndexType = Index<kDim>;

  const auto vertex_adjacency_offsets = GridNeighborhood<
      kDim>::template generateIndexOffsets<Adjacency::kSharedVertex>();
  for (const IndexType& index :
       TestFixture::template getRandomIndexVector<kDim>(100, 100)) {
    for (const IndexType& offset : vertex_adjacency_offsets) {
      const IndexType neighbor = index + offset;
      const auto adjacency_type = Adjacency{computeAdjacency(index, neighbor)};
      EXPECT_TRUE(adjacency_type.isValid());
      EXPECT_EQ(adjacency_type.toTypeId(), Adjacency::kSharedVertex);
      EXPECT_TRUE(areAdjacent(index, neighbor, Adjacency::kSharedVertex));
    }
  }
}

TYPED_TEST(GridAdjacencyTest, SharedEdgeAdjacency) {
  constexpr int kDim = TypeParam::value;
  using IndexType = Index<kDim>;

  const auto edge_adjacency_offsets = GridNeighborhood<
      kDim>::template generateIndexOffsets<Adjacency::kSharedEdge>();
  for (const IndexType& index :
       TestFixture::template getRandomIndexVector<kDim>(100, 100)) {
    for (const IndexType& offset : edge_adjacency_offsets) {
      const IndexType neighbor = index + offset;
      const auto adjacency_type = Adjacency{computeAdjacency(index, neighbor)};
      EXPECT_TRUE(adjacency_type.isValid());
      EXPECT_EQ(adjacency_type.toTypeId(), Adjacency::kSharedEdge);
      EXPECT_TRUE(areAdjacent(index, neighbor, Adjacency::kSharedEdge));
    }
  }
}

TYPED_TEST(GridAdjacencyTest, SharedFaceAdjacency) {
  constexpr int kDim = TypeParam::value;
  using IndexType = Index<kDim>;

  const auto face_adjacency_offsets = GridNeighborhood<
      kDim>::template generateIndexOffsets<Adjacency::kSharedFace>();
  for (const IndexType& index :
       TestFixture::template getRandomIndexVector<kDim>(100, 100)) {
    for (const IndexType& offset : face_adjacency_offsets) {
      const IndexType neighbor = index + offset;
      const auto adjacency_type = Adjacency{computeAdjacency(index, neighbor)};
      EXPECT_TRUE(adjacency_type.isValid());
      EXPECT_EQ(adjacency_type.toTypeId(), Adjacency::kSharedFace);
      EXPECT_TRUE(areAdjacent(index, neighbor, Adjacency::kSharedFace));
    }
  }
}

TYPED_TEST(GridAdjacencyTest, SharedCubeAdjacency) {
  constexpr int kDim = TypeParam::value;
  using IndexType = Index<kDim>;

  const auto cube_adjacency_offsets = GridNeighborhood<
      kDim>::template generateIndexOffsets<Adjacency::kSharedCube>();
  for (const IndexType& index :
       TestFixture::template getRandomIndexVector<kDim>(100, 100)) {
    for (const IndexType& offset : cube_adjacency_offsets) {
      const IndexType neighbor = index + offset;
      const auto adjacency_type = Adjacency{computeAdjacency(index, neighbor)};
      EXPECT_TRUE(adjacency_type.isValid());
      EXPECT_EQ(adjacency_type.toTypeId(), Adjacency::kSharedCube);
      EXPECT_TRUE(areAdjacent(index, neighbor, Adjacency::kSharedCube));
    }
  }
}

TYPED_TEST(GridAdjacencyTest, AnyDisjointAdjacency) {
  constexpr int kDim = TypeParam::value;
  using IndexType = Index<kDim>;

  const auto disjoint_adjacency_offsets = GridNeighborhood<
      kDim>::template generateIndexOffsets<Adjacency::kAnyDisjoint>();
  for (const IndexType& index :
       TestFixture::template getRandomIndexVector<kDim>(100, 100)) {
    for (const IndexType& offset : disjoint_adjacency_offsets) {
      const IndexType neighbor = index + offset;
      const auto adjacency_type = Adjacency{computeAdjacency(index, neighbor)};
      EXPECT_TRUE(adjacency_type.isValid());
      EXPECT_TRUE(areAdjacent(index, neighbor, Adjacency::kAnyDisjoint));
    }
  }
}

TYPED_TEST(GridAdjacencyTest, NoAdjacency) {
  constexpr int kDim = TypeParam::value;
  using IndexType = Index<kDim>;

  const auto disjoint_adjacency_offsets = GridNeighborhood<
      kDim>::template generateIndexOffsets<Adjacency::kAnyDisjoint>();
  for (const IndexType& index :
       TestFixture::template getRandomIndexVector<kDim>(100, 100)) {
    for (const IndexType& offset : disjoint_adjacency_offsets) {
      const IndexType non_adjacent_index = index + 2 * offset;
      const auto adjacency_type =
          Adjacency{computeAdjacency(index, non_adjacent_index)};
      EXPECT_TRUE(adjacency_type.isValid());
      EXPECT_EQ(adjacency_type, Adjacency::kNone);
      EXPECT_FALSE(areAdjacent(index, non_adjacent_index, Adjacency::kAny));
    }
  }
}

TYPED_TEST(GridAdjacencyTest, AnyAdjacency) {
  constexpr int kDim = TypeParam::value;
  using IndexType = Index<kDim>;

  const auto any_adjacency_offsets =
      GridNeighborhood<kDim>::template generateIndexOffsets<Adjacency::kAny>();

  for (const IndexType& index :
       TestFixture::template getRandomIndexVector<kDim>(100, 100)) {
    for (const IndexType& offset : any_adjacency_offsets) {
      const IndexType neighbor = index + offset;
      const auto adjacency_type = Adjacency{computeAdjacency(index, neighbor)};
      EXPECT_TRUE(adjacency_type.isValid());
      EXPECT_TRUE(areAdjacent(index, neighbor, Adjacency::kAny));
    }
  }
}
}  // namespace wavemap
