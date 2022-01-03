#include <gtest/gtest.h>

#include "wavemap_2d/datastructure/cell.h"

namespace wavemap_2d {
TEST(CellTest, CellTraits) {
  {
    using UnboundedCell = CellTraits<FloatingPoint, FloatingPoint, int>;
    EXPECT_FALSE(UnboundedCell::hasLowerBound);
    EXPECT_FALSE(UnboundedCell::hasUpperBound);
    EXPECT_FALSE(UnboundedCell::isFullyBounded);
    EXPECT_FLOAT_EQ(UnboundedCell::kSpecializedToBaseIntScalingFactor, 1.f);
    EXPECT_FLOAT_EQ(UnboundedCell::add(0.f, -10.f), -10.f);
    EXPECT_FLOAT_EQ(UnboundedCell::add(-10.f, 0.f), -10.f);
    EXPECT_FLOAT_EQ(UnboundedCell::add(10.f, 0.f), 10.f);
    EXPECT_FLOAT_EQ(UnboundedCell::add(0.f, 10.f), 10.f);
  }

  {
    constexpr BoundType kLowerBound = -2;
    using LowerBoundedCell =
        CellTraits<FloatingPoint, FloatingPoint, int, kLowerBound>;
    EXPECT_TRUE(LowerBoundedCell::hasLowerBound);
    EXPECT_FALSE(LowerBoundedCell::hasUpperBound);
    EXPECT_FALSE(LowerBoundedCell::isFullyBounded);
    EXPECT_FLOAT_EQ(LowerBoundedCell::kSpecializedToBaseIntScalingFactor, 1.f);
    EXPECT_FLOAT_EQ(LowerBoundedCell::add(0.f, -10.f), kLowerBound);
    EXPECT_FLOAT_EQ(LowerBoundedCell::add(-10.f, 0.f), kLowerBound);
    EXPECT_FLOAT_EQ(LowerBoundedCell::add(0.f, 1.f), 1.f);
    EXPECT_FLOAT_EQ(LowerBoundedCell::add(1.f, 0.f), 1.f);
    EXPECT_FLOAT_EQ(LowerBoundedCell::add(10.f, 0.f), 10.f);
    EXPECT_FLOAT_EQ(LowerBoundedCell::add(0.f, 10.f), 10.f);
  }

  {
    constexpr BoundType kUpperBound = 4;
    using UpperBoundedCell =
        CellTraits<FloatingPoint, FloatingPoint, int,
                   std::numeric_limits<BoundType>::lowest(), kUpperBound>;
    EXPECT_FALSE(UpperBoundedCell::hasLowerBound);
    EXPECT_TRUE(UpperBoundedCell::hasUpperBound);
    EXPECT_FALSE(UpperBoundedCell::isFullyBounded);
    EXPECT_FLOAT_EQ(UpperBoundedCell::kSpecializedToBaseIntScalingFactor, 1.f);
    EXPECT_FLOAT_EQ(UpperBoundedCell::add(0.f, -10.f), -10.f);
    EXPECT_FLOAT_EQ(UpperBoundedCell::add(-10.f, 0.f), -10.f);
    EXPECT_FLOAT_EQ(UpperBoundedCell::add(0.f, 1.f), 1.f);
    EXPECT_FLOAT_EQ(UpperBoundedCell::add(1.f, 0.f), 1.f);
    EXPECT_FLOAT_EQ(UpperBoundedCell::add(10.f, 0.f), kUpperBound);
    EXPECT_FLOAT_EQ(UpperBoundedCell::add(0.f, 10.f), kUpperBound);
  }

  {
    using Specialized = FloatingPoint;
    using BaseInt = int;
    constexpr BoundType kLowerBound = -2;
    constexpr BoundType kUpperBound = 4;
    using FullyBoundedCell =
        CellTraits<Specialized, FloatingPoint, BaseInt, -2, 4>;
    EXPECT_TRUE(FullyBoundedCell::hasLowerBound);
    EXPECT_TRUE(FullyBoundedCell::hasUpperBound);
    EXPECT_TRUE(FullyBoundedCell::isFullyBounded);
    EXPECT_FLOAT_EQ(FullyBoundedCell::kLowerBound, kLowerBound);
    EXPECT_FLOAT_EQ(FullyBoundedCell::kUpperBound, kUpperBound);
    EXPECT_FLOAT_EQ(
        FullyBoundedCell::kSpecializedToBaseIntScalingFactor,
        (static_cast<Specialized>(std::numeric_limits<BaseInt>::max()) -
         static_cast<Specialized>(std::numeric_limits<BaseInt>::lowest())) /
            (kUpperBound - kLowerBound));
    EXPECT_FLOAT_EQ(FullyBoundedCell::add(0.f, -10.f), kLowerBound);
    EXPECT_FLOAT_EQ(FullyBoundedCell::add(-10.f, 0.f), kLowerBound);
    EXPECT_FLOAT_EQ(FullyBoundedCell::add(0.f, 1.f), 1.f);
    EXPECT_FLOAT_EQ(FullyBoundedCell::add(1.f, 0.f), 1.f);
    EXPECT_FLOAT_EQ(FullyBoundedCell::add(10.f, 0.f), kUpperBound);
    EXPECT_FLOAT_EQ(FullyBoundedCell::add(0.f, 10.f), kUpperBound);
  }
}
}  // namespace wavemap_2d
