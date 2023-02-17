#include <bitset>

#include <gtest/gtest.h>

#include "wavemap/utils/bit_manipulation.h"

namespace wavemap {
TEST(BitManipulationTest, RotateLeft) {
  // Test default template argument deduction
  EXPECT_EQ(bit_manip::rotate_left(0b1011, 3), 0b1011000);

  // Test with explicitly set widths
  constexpr int kWidth = 4;
  std::set<std::tuple<int, int, int>> test_pairs{
      // Simple patterns
      {0b0001, 0, 0b0001},
      {0b0001, 1, 0b0010},
      {0b0001, 2, 0b0100},
      {0b0001, 3, 0b1000},
      {0b0001, 4, 0b0001},
      {0b1000, 0, 0b1000},
      {0b1000, 1, 0b0001},
      {0b1000, 2, 0b0010},
      {0b1000, 3, 0b0100},
      {0b1000, 4, 0b1000},
      // Random pattern
      {0b1011, 0, 0b1011},
      {0b1011, 1, 0b0111},
      {0b1011, 2, 0b1110},
      {0b1011, 3, 0b1101},
      {0b1011, 4, 0b1011},
      // Check that bits beyond kWidth are correctly ignored
      {0b10001, 0, 0b0001},
      {0b10001, 1, 0b0010},
      {0b10001, 2, 0b0100},
      {0b10001, 3, 0b1000},
      {0b10001, 4, 0b0001},
      {0b11000, 0, 0b1000},
      {0b11000, 1, 0b0001},
      {0b11000, 2, 0b0010},
      {0b11000, 3, 0b0100},
      {0b11000, 4, 0b1000},
  };
  for (auto [input, shift, expected_result] : test_pairs) {
    auto result = bit_manip::rotate_left<int, kWidth>(input, shift);
    EXPECT_EQ(result, expected_result)
        << "Shifting input " << std::bitset<kWidth>(input) << " left by "
        << shift << " steps should yield "
        << std::bitset<kWidth>(expected_result) << " but got "
        << std::bitset<kWidth>(result) << " (raw bits "
        << std::bitset<8 * sizeof(int)>(result) << ")";
  }

  // Check that rotations still happen correctly when the left-most bit is 1
  // NOTE: This is mainly done to check for that we don't accidentally use
  //       sign-extending right shifts instead of regular right shifts.
  EXPECT_EQ(bit_manip::rotate_left(-1, 3), -1);
  EXPECT_EQ(bit_manip::rotate_left(std::numeric_limits<int>::lowest(), 3), 4);
}

TEST(BitManipulationTest, RotateRight) {
  // Test default template argument deduction
  EXPECT_EQ(bit_manip::rotate_right(0b1011000, 3), 0b1011);

  // Test with explicitly set widths
  constexpr int kWidth = 4;
  std::set<std::tuple<int, int, int>> test_pairs{
      // Simple patterns
      {0b0001, 0, 0b0001},
      {0b0001, 1, 0b1000},
      {0b0001, 2, 0b0100},
      {0b0001, 3, 0b0010},
      {0b0001, 4, 0b0001},
      {0b1000, 0, 0b1000},
      {0b1000, 1, 0b0100},
      {0b1000, 2, 0b0010},
      {0b1000, 3, 0b0001},
      {0b1000, 4, 0b1000},
      // Random pattern
      {0b1011, 0, 0b1011},
      {0b1011, 1, 0b1101},
      {0b1011, 2, 0b1110},
      {0b1011, 3, 0b0111},
      {0b1011, 4, 0b1011},
      // Check that bits beyond kWidth are correctly ignored
      {0b10001, 0, 0b0001},
      {0b10001, 1, 0b1000},
      {0b10001, 2, 0b0100},
      {0b10001, 3, 0b0010},
      {0b10001, 4, 0b0001},
      {0b11000, 0, 0b1000},
      {0b11000, 1, 0b0100},
      {0b11000, 2, 0b0010},
      {0b11000, 3, 0b0001},
      {0b11000, 4, 0b1000},
  };
  for (auto [input, shift, expected_result] : test_pairs) {
    auto result = bit_manip::rotate_right<int, kWidth>(input, shift);
    EXPECT_EQ(result, expected_result)
        << "Shifting input " << std::bitset<kWidth>(input) << " right by "
        << shift << " steps should yield "
        << std::bitset<kWidth>(expected_result) << " but got "
        << std::bitset<kWidth>(result) << " (raw bits "
        << std::bitset<8 * sizeof(int)>(result) << ")";
  }

  // Check that rotations still happen correctly when the left-most bit is 1
  // NOTE: This is mainly done to check for that we don't accidentally use
  //       sign-extending right shifts instead of regular right shifts.
  EXPECT_EQ(bit_manip::rotate_right(-1, 3), -1);
  EXPECT_EQ(bit_manip::rotate_right(std::numeric_limits<int>::lowest(), 3),
            static_cast<unsigned>(std::numeric_limits<int>::lowest()) >> 3);
}

TEST(BitManipulationTest, SqueezeIn) {
  EXPECT_EQ(bit_manip::squeeze_in(0b1001, false, 0), 0b10010);
  EXPECT_EQ(bit_manip::squeeze_in(0b1001, false, 1), 0b10001);
  EXPECT_EQ(bit_manip::squeeze_in(0b1001, false, 2), 0b10001);
  EXPECT_EQ(bit_manip::squeeze_in(0b1001, false, 3), 0b10001);
  EXPECT_EQ(bit_manip::squeeze_in(0b1001, false, 4), 0b01001);

  EXPECT_EQ(bit_manip::squeeze_in(0b1001, true, 0), 0b10011);
  EXPECT_EQ(bit_manip::squeeze_in(0b1001, true, 1), 0b10011);
  EXPECT_EQ(bit_manip::squeeze_in(0b1001, true, 2), 0b10101);
  EXPECT_EQ(bit_manip::squeeze_in(0b1001, true, 3), 0b11001);
  EXPECT_EQ(bit_manip::squeeze_in(0b1001, true, 4), 0b11001);
}

TEST(BitManipulationTest, Popcount) {
  EXPECT_EQ(bit_manip::popcount(0b0), 0);
  EXPECT_EQ(bit_manip::popcount(0b1), 1);
  EXPECT_EQ(bit_manip::popcount(0b10), 1);
  EXPECT_EQ(bit_manip::popcount(0b11), 2);

  EXPECT_EQ(bit_manip::popcount(0b00101010111000), 6);
  EXPECT_EQ(bit_manip::popcount(0b11101010010111), 9);
}

TEST(BitManipulationTest, Parity) {
  EXPECT_EQ(bit_manip::parity(0b0), 0);
  EXPECT_EQ(bit_manip::parity(0b1), 1);
  EXPECT_EQ(bit_manip::parity(0b10), 1);
  EXPECT_EQ(bit_manip::parity(0b11), 0);

  EXPECT_EQ(bit_manip::parity(0b00101010111000), 0);
  EXPECT_EQ(bit_manip::parity(0b11101010010111), 1);
}
}  // namespace wavemap
