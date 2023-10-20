#include <cmath>

#include <gtest/gtest.h>

#include "wavemap/utils/math/int_math.h"

namespace wavemap {
TEST(IntMathTest, Exp2) {
  constexpr int kMaxExponent = 31;
  for (int exponent = 0; exponent < kMaxExponent; ++exponent) {
    EXPECT_EQ(int_math::exp2(static_cast<int>(exponent)),
              std::exp2(static_cast<int>(exponent)))
        << "For exponent " << exponent;
    EXPECT_EQ(int_math::exp2(static_cast<unsigned>(exponent)),
              std::exp2(static_cast<unsigned>(exponent)))
        << "For exponent " << exponent;
  }
}

TEST(IntMathTest, Log2) {
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
      EXPECT_EQ(int_math::log2_floor(static_cast<int>(value)),
                std::floor(std::log2(static_cast<int>(value))))
          << "For value " << value;
      EXPECT_EQ(int_math::log2_ceil(static_cast<int>(value)),
                std::ceil(std::log2(static_cast<int>(value))))
          << "For value " << value;
      EXPECT_EQ(int_math::log2_floor(static_cast<unsigned>(value)),
                std::floor(std::log2(static_cast<unsigned>(value))))
          << "For value " << value;
      EXPECT_EQ(int_math::log2_ceil(static_cast<unsigned>(value)),
                std::ceil(std::log2(static_cast<unsigned>(value))))
          << "For value " << value;
    }
  }
}

TEST(IntMathTest, DivExp2) {
  constexpr int kMaxExp = 10;
  constexpr int kMaxValue = 2000;
  constexpr int kMinValue = -kMaxValue;
  for (int exp = 0; exp < kMaxExp; ++exp) {
    for (int value = kMinValue; value < kMaxValue; ++value) {
      const FloatingPoint result = static_cast<FloatingPoint>(value) /
                                   std::exp2f(static_cast<FloatingPoint>(exp));
      EXPECT_EQ(int_math::div_exp2_floor(value, exp), std::floor(result));
      EXPECT_EQ(int_math::div_exp2_ceil(value, exp), std::ceil(result));
    }
  }
}

TEST(IntMathTest, PowSequence) {
  // Test trivial (length 1) sequences
  constexpr std::array<IndexElement, 1> expected_pow_sequence_length_1 = {1};
  constexpr auto pow_sequence_base_1_length_1 =
      int_math::pow_sequence<IndexElement, 1, 1>();
  EXPECT_EQ(pow_sequence_base_1_length_1, expected_pow_sequence_length_1);
  constexpr auto pow_sequence_base_4_length_1 =
      int_math::pow_sequence<IndexElement, 4, 1>();
  EXPECT_EQ(pow_sequence_base_4_length_1, expected_pow_sequence_length_1);

  // Test power sequences for different bases
  constexpr std::array<IndexElement, 4> expected_pow_sequence_base_1_length_4 =
      {1, 1, 1, 1};
  constexpr auto pow_sequence_base_1_length_4(
      int_math::pow_sequence<IndexElement, 1, 4>());
  EXPECT_EQ(pow_sequence_base_1_length_4,
            expected_pow_sequence_base_1_length_4);

  constexpr std::array<IndexElement, 4> expected_pow_sequence_base_2_length_4 =
      {1, 2, 4, 8};
  constexpr auto pow_sequence_base_2_length_4 =
      int_math::pow_sequence<IndexElement, 2, 4>();
  EXPECT_EQ(pow_sequence_base_2_length_4,
            expected_pow_sequence_base_2_length_4);

  constexpr std::array<IndexElement, 4> expected_pow_sequence_base_3_length_4 =
      {1, 3, 9, 27};
  constexpr auto pow_sequence_base_3_length_4 =
      int_math::pow_sequence<IndexElement, 3, 4>();
  EXPECT_EQ(pow_sequence_base_3_length_4,
            expected_pow_sequence_base_3_length_4);

  constexpr std::array<IndexElement, 4>
      expected_pow_sequence_base_min_2_length_4 = {1, -2, 4, -8};
  constexpr auto pow_sequence_base_min_2_length_4 =
      int_math::pow_sequence<IndexElement, -2, 4>();
  EXPECT_EQ(pow_sequence_base_min_2_length_4,
            expected_pow_sequence_base_min_2_length_4);
}
}  // namespace wavemap
