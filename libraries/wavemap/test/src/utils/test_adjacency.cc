#include <gtest/gtest.h>

#include "wavemap/utils/bits/bit_operations.h"
#include "wavemap/utils/neighbors/adjacency.h"

namespace wavemap::neighbors {
TEST(AdjacencyTest, AdjacencyMask) {
  // Check that the "no adjacency" mask tests negative for any adjacency type
  EXPECT_EQ(bit_ops::is_bit_set(kAdjacencyNone,
                                static_cast<int>(AdjacencyType::kVertex)),
            false);
  EXPECT_EQ(bit_ops::is_bit_set(kAdjacencyNone,
                                static_cast<int>(AdjacencyType::kEdge)),
            false);
  EXPECT_EQ(bit_ops::is_bit_set(kAdjacencyNone,
                                static_cast<int>(AdjacencyType::kFace)),
            false);
  EXPECT_EQ(bit_ops::is_bit_set(kAdjacencyNone,
                                static_cast<int>(AdjacencyType::kCube)),
            false);

  // Check that the "any adjacency" mask tests positive for any adjacency type
  EXPECT_EQ(bit_ops::is_bit_set(kAdjacencyAny,
                                static_cast<int>(AdjacencyType::kVertex)),
            true);
  EXPECT_EQ(bit_ops::is_bit_set(kAdjacencyAny,
                                static_cast<int>(AdjacencyType::kEdge)),
            true);
  EXPECT_EQ(bit_ops::is_bit_set(kAdjacencyAny,
                                static_cast<int>(AdjacencyType::kFace)),
            true);
  EXPECT_EQ(bit_ops::is_bit_set(kAdjacencyAny,
                                static_cast<int>(AdjacencyType::kCube)),
            true);

  // Check the "any disjoint adjacency" mask for 2D grids
  EXPECT_EQ(bit_ops::is_bit_set(kAdjacencyAnyDisjoint<2>,
                                static_cast<int>(AdjacencyType::kVertex)),
            true);
  EXPECT_EQ(bit_ops::is_bit_set(kAdjacencyAnyDisjoint<2>,
                                static_cast<int>(AdjacencyType::kEdge)),
            true);
  EXPECT_EQ(bit_ops::is_bit_set(kAdjacencyAnyDisjoint<2>,
                                static_cast<int>(AdjacencyType::kFace)),
            false);
  EXPECT_EQ(bit_ops::is_bit_set(kAdjacencyAnyDisjoint<2>,
                                static_cast<int>(AdjacencyType::kCube)),
            false);

  // Check the "any disjoint adjacency" mask for 3D grids
  EXPECT_EQ(bit_ops::is_bit_set(kAdjacencyAnyDisjoint<3>,
                                static_cast<int>(AdjacencyType::kVertex)),
            true);
  EXPECT_EQ(bit_ops::is_bit_set(kAdjacencyAnyDisjoint<3>,
                                static_cast<int>(AdjacencyType::kEdge)),
            true);
  EXPECT_EQ(bit_ops::is_bit_set(kAdjacencyAnyDisjoint<3>,
                                static_cast<int>(AdjacencyType::kFace)),
            true);
  EXPECT_EQ(bit_ops::is_bit_set(kAdjacencyAnyDisjoint<3>,
                                static_cast<int>(AdjacencyType::kCube)),
            false);
}
}  // namespace wavemap::neighbors
