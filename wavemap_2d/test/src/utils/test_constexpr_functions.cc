#include <cmath>

#include <gtest/gtest.h>

#include "wavemap_2d/utils/constexpr_functions.h"

namespace wavemap_2d {
TEST(ConstexprFunctionsTest, Exp2) {
  constexpr int kMaxExponent = 31;
  for (int i = 0; i < kMaxExponent; ++i) {
    EXPECT_EQ(constexpr_functions::exp2(i), std::exp2(i));
  }
}
}  // namespace wavemap_2d
