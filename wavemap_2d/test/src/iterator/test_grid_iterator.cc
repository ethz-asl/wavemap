#include <gtest/gtest.h>

#include "wavemap_2d/common.h"
#include "wavemap_2d/indexing/index.h"
#include "wavemap_2d/iterator/grid_iterator.h"
#include "wavemap_2d/test/fixture_base.h"

namespace wavemap_2d {
using GridIteratorTest = FixtureBase;

TEST_F(GridIteratorTest, EquivalenceToNestedLoops) {
  constexpr int kNumTestGrids = 20;
  for (int i = 0; i < kNumTestGrids; ++i) {
    const Index bottom_left_idx = -getRandomIndex().cwiseAbs();
    const Index top_right_idx = getRandomIndex().cwiseAbs();

    Grid grid(bottom_left_idx, top_right_idx);
    auto grid_it = grid.begin();
    const auto grid_it_end = grid.end();

    size_t count = 0u;
    for (Index index = bottom_left_idx; index.x() <= top_right_idx.x();
         ++index.x()) {
      for (index.y() = bottom_left_idx.y(); index.y() <= top_right_idx.y();
           ++index.y()) {
        EXPECT_NE(grid_it, grid_it_end);
        EXPECT_EQ(*grid_it, index);
        ++grid_it;
        ++count;
      }
    }
    const size_t num_grid_indices =
        (top_right_idx - bottom_left_idx + Index::Ones()).array().prod();
    EXPECT_EQ(count, num_grid_indices);
    EXPECT_EQ(grid_it, grid_it_end);
  }
}
}  // namespace wavemap_2d
