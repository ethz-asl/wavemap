#include <gtest/gtest.h>

#include "wavemap/common.h"
#include "wavemap/iterator/grid_iterator.h"
#include "wavemap/test/fixture_base.h"
#include "wavemap/utils/eigen_format.h"

namespace wavemap {
template <typename TypeParamT>
using GridIteratorTest = FixtureBase;

template <int dim>
struct TypeParamTemplate {
  static constexpr int kDim = dim;
};
using TypeParams = ::testing::Types<TypeParamTemplate<2>, TypeParamTemplate<3>>;
TYPED_TEST_SUITE(GridIteratorTest, TypeParams, );

// Helper class that creates nested loops using template recursion
template <typename IndexT, typename LoopBodyFn,
          int dim_idx = IndexT::RowsAtCompileTime - 1>
struct NestedLoopGridVisitor {
  NestedLoopGridVisitor(const IndexT& min_index, const IndexT& max_index,
                        LoopBodyFn visitor_fn) {
    IndexT current_index = min_index;
    Nest(min_index, max_index, current_index, visitor_fn);
  }

  static void Nest(const IndexT& min_index, const IndexT& max_index,
                   IndexT& current_index, LoopBodyFn loop_body_fn) {
    for (current_index[dim_idx] = min_index[dim_idx];
         current_index[dim_idx] <= max_index[dim_idx];
         ++current_index[dim_idx]) {
      NestedLoopGridVisitor<IndexT, LoopBodyFn, dim_idx - 1>::Nest(
          min_index, max_index, current_index, loop_body_fn);
    }
  }
};
// Template specialize the base case s.t. the recursion terminates
template <typename IndexT, typename LoopBodyFn>
struct NestedLoopGridVisitor<IndexT, LoopBodyFn, -1> {
  // The base case should never be used directly, hence its ctor is deleted
  NestedLoopGridVisitor(const IndexT& min_index, const IndexT& max_index,
                        LoopBodyFn visitor_fn) = delete;

  static void Nest(const IndexT& /*min_index*/, const IndexT& /*max_index*/,
                   IndexT& current_index, LoopBodyFn loop_body_fn) {
    loop_body_fn(current_index);
  }
};

TYPED_TEST(GridIteratorTest, EquivalenceToNestedLoops) {
  constexpr int kDim = TypeParam::kDim;
  constexpr int kNumTestGrids = 20;

  for (int i = 0; i < kNumTestGrids; ++i) {
    const Index<kDim> bottom_left_idx =
        TestFixture::template getRandomIndex<kDim>(Index<kDim>::Constant(-4e1),
                                                   Index<kDim>::Zero());
    const Index<kDim> top_right_idx =
        TestFixture::template getRandomIndex<kDim>(Index<kDim>::Zero(),
                                                   Index<kDim>::Constant(4e1));

    Grid grid(bottom_left_idx, top_right_idx);
    auto grid_it = grid.begin();
    const auto grid_it_end = grid.end();

    size_t count = 0u;
    NestedLoopGridVisitor(
        bottom_left_idx, top_right_idx, [&](const Index<kDim>& index) {
          EXPECT_EQ(*grid_it, index)
              << "Got *grid_it " << EigenFormat::oneLine(*grid_it)
              << ", while expecting index " << EigenFormat::oneLine(index)
              << " between min index " << EigenFormat::oneLine(bottom_left_idx)
              << " and max index " << EigenFormat::oneLine(top_right_idx);
          EXPECT_NE(grid_it, grid_it_end)
              << "For *grid_it " << EigenFormat::oneLine(*grid_it)
              << ", *grid_it_end " << EigenFormat::oneLine(*grid_it_end)
              << " while at index " << EigenFormat::oneLine(index)
              << " between min index " << EigenFormat::oneLine(bottom_left_idx)
              << " and max index " << EigenFormat::oneLine(top_right_idx);
          ++grid_it;
          ++count;
        });

    const size_t num_grid_indices =
        (top_right_idx - bottom_left_idx + Index<kDim>::Ones()).array().prod();
    EXPECT_EQ(count, num_grid_indices)
        << "If this fails, the test itself is broken";
    EXPECT_EQ(grid_it, grid_it_end)
        << "For *grid_it " << EigenFormat::oneLine(*grid_it)
        << ", *grid_it_end " << EigenFormat::oneLine(*grid_it_end)
        << ", min index " << EigenFormat::oneLine(bottom_left_idx)
        << " and max index " << EigenFormat::oneLine(top_right_idx);
  }
}
}  // namespace wavemap
