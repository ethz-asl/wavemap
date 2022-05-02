#include <cmath>

#include <gtest/gtest.h>

#include "wavemap_2d/utils/int_math.h"

namespace wavemap_2d {
TEST(ConstexprFunctionsTest, Exp2) {
  constexpr int kMaxExponent = 31;
  for (int exponent = 0; exponent < kMaxExponent; ++exponent) {
    EXPECT_EQ(int_math::exp2(exponent), std::exp2(exponent))
        << "For exponent " << exponent;
  }
}

TEST(ConstexprFunctionsTest, Log2) {
  // NOTE: We want to test whether the result of our log2_floored|ceiled methods
  //       are truncated in the right direction for all values in the
  //       logarithm's valid range [1, max_int]. By bit shifting a base_value
  //       upward, we can quickly test the logarithms of all powers of 2 (where
  //       the floored and ceiled results values are equal) and by applying
  //       offsets of -1 and 1 we also check if their neighbors are
  //       floored|ceiled in the right direction. We start with base_value=2 and
  //       offset=-1, so that the first test value is 1.
  for (int base_value = 2; 0 < base_value; base_value <<= 1) {
    for (int offset : {-1, 0, 1}) {
      const int value = base_value + offset;
      EXPECT_EQ(int_math::log2_floor(value), std::floor(std::log2(value)))
          << "For value " << value;
      EXPECT_EQ(int_math::log2_ceil(value), std::ceil(std::log2(value)))
          << "For value " << value;
    }
  }
  std::cout << std::endl;
}
}  // namespace wavemap_2d
