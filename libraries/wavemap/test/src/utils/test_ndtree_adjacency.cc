#include "wavemap/test/fixture_base.h"
#include "wavemap/test/geometry_generator.h"
#include "wavemap/utils/neighbors/grid_adjacency.h"
#include "wavemap/utils/neighbors/grid_neighborhood.h"
#include "wavemap/utils/neighbors/ndtree_adjacency.h"

namespace wavemap {
template <typename DimT>
class NdtreeAdjacencyTest : public FixtureBase, public GeometryGenerator {};

using Dimensions = ::testing::Types<std::integral_constant<int, 1>,
                                    std::integral_constant<int, 2>,
                                    std::integral_constant<int, 3>>;
TYPED_TEST_SUITE(NdtreeAdjacencyTest, Dimensions, );

TYPED_TEST(NdtreeAdjacencyTest, AnyAdjacency) {
  constexpr int kDim = TypeParam::value;
  using IndexType = Index<kDim>;
  using NdtreeIndexType = NdtreeIndex<kDim>;

  const auto vertex_adjacency_offsets =
      GridNeighborhood<kDim>::template generateIndexOffsets<Adjacency::kAny>();
  for (const auto& index :
       TestFixture::template getRandomNdtreeIndexVector<NdtreeIndexType>(
           IndexType::Constant(-100), IndexType::Constant(100), 0, 6, 100,
           100)) {
    for (const IndexType& offset : vertex_adjacency_offsets) {
      const NdtreeIndexType neighbor{index.height, index.position + offset};
      EXPECT_TRUE(areAdjacent(index, neighbor));
      EXPECT_TRUE(
          areAdjacent(index.computeParentIndex(index.height + 1), neighbor));
    }
  }
}

TYPED_TEST(NdtreeAdjacencyTest, NoAdjacency) {
  constexpr int kDim = TypeParam::value;
  using IndexType = Index<kDim>;
  using NdtreeIndexType = NdtreeIndex<kDim>;

  const auto vertex_adjacency_offsets = GridNeighborhood<
      kDim>::template generateIndexOffsets<Adjacency::kAnyDisjoint>();
  for (const auto& index :
       TestFixture::template getRandomNdtreeIndexVector<NdtreeIndexType>(
           IndexType::Constant(-100), IndexType::Constant(100), 0, 6, 100,
           100)) {
    for (const IndexType& offset : vertex_adjacency_offsets) {
      const NdtreeIndexType neighbor{index.height, index.position + 2 * offset};
      EXPECT_FALSE(areAdjacent(index, neighbor));
    }
  }
}
}  // namespace wavemap
