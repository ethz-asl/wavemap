#include <gtest/gtest.h>

#include "wavemap/common.h"
#include "wavemap/utils/fill_utils.h"

namespace wavemap {
TEST(FillUtils, Constant) {
  EXPECT_EQ(fill::constant<bool>(false), false);
  EXPECT_EQ(fill::constant<bool>(true), true);

  EXPECT_EQ(fill::constant<int>(-1), -1);
  EXPECT_EQ(fill::constant<int>(0), 0);
  EXPECT_EQ(fill::constant<int>(3), 3);

  EXPECT_EQ(fill::constant<FloatingPoint>(0.f), 0.f);
  EXPECT_EQ(fill::constant<FloatingPoint>(-1.23f), -1.23f);
  EXPECT_EQ(fill::constant<FloatingPoint>(1.23f), 1.23f);

  EXPECT_EQ(fill::constant<Point3D>(0.f), Point3D::Constant(0.f));
  EXPECT_EQ(fill::constant<Point3D>(-4.56f), Point3D::Constant(-4.56f));
  EXPECT_EQ(fill::constant<Point3D>(4.56f), Point3D::Constant(4.56f));
}

TEST(FillUtils, Zero) {
  EXPECT_EQ(fill::zero<bool>(), false);
  EXPECT_EQ(fill::zero<int>(), 0);
  EXPECT_EQ(fill::zero<FloatingPoint>(), 0.f);
  EXPECT_EQ(fill::zero<Point3D>(), Point3D::Zero());
}
}  // namespace wavemap
