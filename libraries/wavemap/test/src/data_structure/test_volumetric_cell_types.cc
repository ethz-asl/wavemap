#include <gtest/gtest.h>

#include "wavemap/data_structure/volumetric/cell_types/scalar_cell.h"

namespace wavemap {
TEST(CellTest, UnboundedScalar) {
  EXPECT_FALSE(UnboundedScalarCell::hasLowerBound);
  EXPECT_FALSE(UnboundedScalarCell::hasUpperBound);
  EXPECT_FALSE(UnboundedScalarCell::isFullyBounded);
  EXPECT_FLOAT_EQ(UnboundedScalarCell::kSpecializedToBaseIntScalingFactor, 1.f);
  EXPECT_FLOAT_EQ(UnboundedScalarCell::add(0.f, -10.f), -10.f);
  EXPECT_FLOAT_EQ(UnboundedScalarCell::add(-10.f, 0.f), -10.f);
  EXPECT_FLOAT_EQ(UnboundedScalarCell::add(10.f, 0.f), 10.f);
  EXPECT_FLOAT_EQ(UnboundedScalarCell::add(0.f, 10.f), 10.f);
}

TEST(CellTest, LowerBoundedScalar) {
  constexpr BoundType kLowerBound = -2;
  using LowerBoundedScalar = LowerBoundedScalarCell<kLowerBound>;
  EXPECT_TRUE(LowerBoundedScalar::hasLowerBound);
  EXPECT_FALSE(LowerBoundedScalar::hasUpperBound);
  EXPECT_FALSE(LowerBoundedScalar::isFullyBounded);
  EXPECT_FLOAT_EQ(LowerBoundedScalar::kSpecializedToBaseIntScalingFactor, 1.f);
  EXPECT_FLOAT_EQ(LowerBoundedScalar::add(0.f, -10.f), kLowerBound);
  EXPECT_FLOAT_EQ(LowerBoundedScalar::add(-10.f, 0.f), kLowerBound);
  EXPECT_FLOAT_EQ(LowerBoundedScalar::add(0.f, 1.f), 1.f);
  EXPECT_FLOAT_EQ(LowerBoundedScalar::add(1.f, 0.f), 1.f);
  EXPECT_FLOAT_EQ(LowerBoundedScalar::add(10.f, 0.f), 10.f);
  EXPECT_FLOAT_EQ(LowerBoundedScalar::add(0.f, 10.f), 10.f);
}

TEST(CellTest, UpperBoundedScalar) {
  constexpr BoundType kUpperBound = 4;
  using UpperBoundedScalar = UpperBoundedScalarCell<kUpperBound>;
  EXPECT_FALSE(UpperBoundedScalar::hasLowerBound);
  EXPECT_TRUE(UpperBoundedScalar::hasUpperBound);
  EXPECT_FALSE(UpperBoundedScalar::isFullyBounded);
  EXPECT_FLOAT_EQ(UpperBoundedScalar::kSpecializedToBaseIntScalingFactor, 1.f);
  EXPECT_FLOAT_EQ(UpperBoundedScalar::add(0.f, -10.f), -10.f);
  EXPECT_FLOAT_EQ(UpperBoundedScalar::add(-10.f, 0.f), -10.f);
  EXPECT_FLOAT_EQ(UpperBoundedScalar::add(0.f, 1.f), 1.f);
  EXPECT_FLOAT_EQ(UpperBoundedScalar::add(1.f, 0.f), 1.f);
  EXPECT_FLOAT_EQ(UpperBoundedScalar::add(10.f, 0.f), kUpperBound);
  EXPECT_FLOAT_EQ(UpperBoundedScalar::add(0.f, 10.f), kUpperBound);
}

TEST(CellTest, FullyBoundedScalar) {
  constexpr BoundType kLowerBound = -2;
  constexpr BoundType kUpperBound = 4;
  using FullyBoundedScalar = BoundedScalarCell<kLowerBound, kUpperBound>;
  EXPECT_TRUE(FullyBoundedScalar::hasLowerBound);
  EXPECT_TRUE(FullyBoundedScalar::hasUpperBound);
  EXPECT_TRUE(FullyBoundedScalar::isFullyBounded);
  EXPECT_FLOAT_EQ(FullyBoundedScalar::kLowerBound, kLowerBound);
  EXPECT_FLOAT_EQ(FullyBoundedScalar::kUpperBound, kUpperBound);
  EXPECT_FLOAT_EQ(
      FullyBoundedScalar::kSpecializedToBaseIntScalingFactor,
      (static_cast<FullyBoundedScalar::Specialized>(
           std::numeric_limits<FullyBoundedScalar::BaseInt>::max()) -
       static_cast<FullyBoundedScalar::Specialized>(
           std::numeric_limits<FullyBoundedScalar::BaseInt>::lowest())) /
          (kUpperBound - kLowerBound));
  EXPECT_FLOAT_EQ(FullyBoundedScalar::add(0.f, -10.f), kLowerBound);
  EXPECT_FLOAT_EQ(FullyBoundedScalar::add(-10.f, 0.f), kLowerBound);
  EXPECT_FLOAT_EQ(FullyBoundedScalar::add(0.f, 1.f), 1.f);
  EXPECT_FLOAT_EQ(FullyBoundedScalar::add(1.f, 0.f), 1.f);
  EXPECT_FLOAT_EQ(FullyBoundedScalar::add(10.f, 0.f), kUpperBound);
  EXPECT_FLOAT_EQ(FullyBoundedScalar::add(0.f, 10.f), kUpperBound);
}
}  // namespace wavemap
