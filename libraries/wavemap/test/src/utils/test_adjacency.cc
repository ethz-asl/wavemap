#include <gtest/gtest.h>

#include "wavemap/test/fixture_base.h"
#include "wavemap/utils/bits/bit_operations.h"
#include "wavemap/utils/neighbors/adjacency.h"

namespace wavemap {
template <typename DimT>
class AdjacencyTest : public FixtureBase {};

using Dimensions = ::testing::Types<std::integral_constant<int, 1>,
                                    std::integral_constant<int, 2>,
                                    std::integral_constant<int, 3>>;
TYPED_TEST_SUITE(AdjacencyTest, Dimensions, );

TYPED_TEST(AdjacencyTest, AdjacencyMask) {
  constexpr int kDim = TypeParam::value;

  // Check that the "no adjacency" mask tests negative for any adjacency type
  EXPECT_EQ(bit_ops::is_bit_set(Adjacency::toMask<kDim>(Adjacency::kNone),
                                static_cast<int>(Adjacency::kSharedVertex)),
            false);
  EXPECT_EQ(bit_ops::is_bit_set(Adjacency::toMask<kDim>(Adjacency::kNone),
                                static_cast<int>(Adjacency::kSharedEdge)),
            false);
  EXPECT_EQ(bit_ops::is_bit_set(Adjacency::toMask<kDim>(Adjacency::kNone),
                                static_cast<int>(Adjacency::kSharedFace)),
            false);
  EXPECT_EQ(bit_ops::is_bit_set(Adjacency::toMask<kDim>(Adjacency::kNone),
                                static_cast<int>(Adjacency::kSharedCube)),
            false);

  // Check that the "any adjacency" mask tests positive for any adjacency type
  EXPECT_EQ(bit_ops::is_bit_set(Adjacency::toMask<kDim>(Adjacency::kAny),
                                static_cast<int>(Adjacency::kSharedVertex)),
            true);
  EXPECT_EQ(bit_ops::is_bit_set(Adjacency::toMask<kDim>(Adjacency::kAny),
                                static_cast<int>(Adjacency::kSharedEdge)),
            true);
  EXPECT_EQ(bit_ops::is_bit_set(Adjacency::toMask<kDim>(Adjacency::kAny),
                                static_cast<int>(Adjacency::kSharedFace)),
            true);
  EXPECT_EQ(bit_ops::is_bit_set(Adjacency::toMask<kDim>(Adjacency::kAny),
                                static_cast<int>(Adjacency::kSharedCube)),
            true);

  // Check the "any disjoint adjacency" mask
  EXPECT_EQ(
      bit_ops::is_bit_set(Adjacency::toMask<kDim>(Adjacency::kAnyDisjoint),
                          static_cast<int>(Adjacency::kSharedVertex)),
      true);
  EXPECT_EQ(
      bit_ops::is_bit_set(Adjacency::toMask<kDim>(Adjacency::kAnyDisjoint),
                          static_cast<int>(Adjacency::kSharedEdge)),
      1 < kDim);
  EXPECT_EQ(
      bit_ops::is_bit_set(Adjacency::toMask<kDim>(Adjacency::kAnyDisjoint),
                          static_cast<int>(Adjacency::kSharedFace)),
      2 < kDim);
  EXPECT_EQ(
      bit_ops::is_bit_set(Adjacency::toMask<kDim>(Adjacency::kAnyDisjoint),
                          static_cast<int>(Adjacency::kSharedCube)),
      3 < kDim);
}
}  // namespace wavemap
