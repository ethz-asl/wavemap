#include <bitset>

#include <gtest/gtest.h>

#include "wavemap_common/utils/bit_manipulation.h"

namespace wavemap {
TEST(BitManipulationTest, RotateLeft) {
  // Test default template argument deduction
  EXPECT_EQ(bit_manip::rotate_left(static_cast<unsigned>(0b1011), 3u),
            0b1011000);

  // Test with explicitly set widths
  constexpr unsigned kWidth = 4;
  std::set<std::tuple<unsigned, unsigned, unsigned>> test_pairs{
      {0b1011, 0, 0b1011},
      {0b1011, 1, 0b0111},
      {0b1011, 2, 0b1110},
      {0b1011, 3, 0b1101}};
  for (auto [input, shift, expected_result] : test_pairs) {
    auto result = bit_manip::rotate_left<unsigned, kWidth>(input, shift);
    EXPECT_EQ(result, expected_result)
        << "Shifting input " << std::bitset<kWidth>(input) << " left by "
        << shift << " steps should yield "
        << std::bitset<kWidth>(expected_result) << " but got "
        << std::bitset<kWidth>(result) << " (raw bits "
        << std::bitset<8 * sizeof(unsigned)>(result) << ")"
        << " mask "
        << std::bitset<8 * sizeof(unsigned)>(~unsigned{} >>
                                             (8 * sizeof(unsigned) - kWidth));
  }
}

TEST(BitManipulationTest, RotateRight) {
  // Test default template argument deduction
  EXPECT_EQ(bit_manip::rotate_right(static_cast<unsigned>(0b1011000), 3u),
            0b1011);

  // Test with explicitly set widths
  constexpr unsigned kWidth = 4;
  std::set<std::tuple<unsigned, unsigned, unsigned>> test_pairs{
      {0b1011, 0, 0b1011},
      {0b1011, 1, 0b1101},
      {0b1011, 2, 0b1110},
      {0b1011, 3, 0b0111},
  };
  for (auto [input, shift, expected_result] : test_pairs) {
    auto result = bit_manip::rotate_right<unsigned, kWidth>(input, shift);
    EXPECT_EQ(result, expected_result)
        << "Shifting input " << std::bitset<kWidth>(input) << " right by "
        << shift << " steps should yield "
        << std::bitset<kWidth>(expected_result) << " but got "
        << std::bitset<kWidth>(result) << " (raw bits "
        << std::bitset<8 * sizeof(unsigned)>(result) << ")";
  }
}
}  // namespace wavemap
