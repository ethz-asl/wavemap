#include <bitset>

#include <gtest/gtest.h>

#include "wavemap/test/fixture_base.h"
#include "wavemap/utils/bit_manipulation.h"

namespace wavemap {
using BitManipulationTest = FixtureBase;

TEST_F(BitManipulationTest, RotateLeft) {
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

TEST_F(BitManipulationTest, RotateRight) {
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

TEST_F(BitManipulationTest, SqueezeIn) {
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

TEST_F(BitManipulationTest, Popcount) {
  // Small numbers
  EXPECT_EQ(bit_manip::popcount(0b0), 0);
  EXPECT_EQ(bit_manip::popcount(0b1), 1);
  EXPECT_EQ(bit_manip::popcount(0b10), 1);
  EXPECT_EQ(bit_manip::popcount(0b11), 2);
  EXPECT_EQ(bit_manip::popcount(0b00101010111000), 6);
  EXPECT_EQ(bit_manip::popcount(0b11101010010111), 9);

  // 32-bit
  EXPECT_EQ(bit_manip::popcount(0b00000010010000100010000010100000), 6);
  EXPECT_EQ(bit_manip::popcount(0b01000000100001001000111000010011), 10);
  EXPECT_EQ(bit_manip::popcount(0b11100000001000100001000100010000), 8);
  EXPECT_EQ(bit_manip::popcount(0b11111111111111111111111111111111), 32);

  // 64-bit
  EXPECT_EQ(
      bit_manip::popcount(
          0b0000100000000100010100100010000100010101110000100010000000100001),
      16);
  EXPECT_EQ(
      bit_manip::popcount(
          0b1000101110000000000101000001100000111010010100011000000101010000),
      20);
  EXPECT_EQ(
      bit_manip::popcount(
          0b1111111111111111111111111111111111111111111111111111111111111111),
      64);
}

TEST_F(BitManipulationTest, Parity) {
  EXPECT_EQ(bit_manip::parity(0b0), 0);
  EXPECT_EQ(bit_manip::parity(0b1), 1);
  EXPECT_EQ(bit_manip::parity(0b10), 1);
  EXPECT_EQ(bit_manip::parity(0b11), 0);

  EXPECT_EQ(bit_manip::parity(0b00101010111000), 0);
  EXPECT_EQ(bit_manip::parity(0b11101010010111), 1);
}

TEST_F(BitManipulationTest, CountLeadingZeros) {
  EXPECT_EQ(bit_manip::clz(static_cast<uint32_t>(1) << 0), 31);
  EXPECT_EQ(bit_manip::clz(static_cast<uint32_t>(1) << 30), 1);
  EXPECT_EQ(bit_manip::clz(static_cast<uint32_t>(1) << 31), 0);

  EXPECT_EQ(bit_manip::clz(static_cast<int32_t>(1)) << 0, 31);
  EXPECT_EQ(bit_manip::clz(static_cast<int32_t>(1) << 30), 1);

  EXPECT_EQ(bit_manip::clz(static_cast<uint64_t>(1)) << 0, 63);
  EXPECT_EQ(bit_manip::clz(static_cast<uint64_t>(1) << 62), 1);
  EXPECT_EQ(bit_manip::clz(static_cast<uint64_t>(1) << 63), 0);

  EXPECT_EQ(bit_manip::clz(static_cast<int64_t>(1)) << 0, 63);
  EXPECT_EQ(bit_manip::clz(static_cast<int64_t>(1) << 62), 1);
}

TEST_F(BitManipulationTest, RepeatBlock) {
  EXPECT_EQ(bit_manip::repeat_block<uint32_t>(2, 0b01),
            0b01010101010101010101010101010101);
  EXPECT_EQ(bit_manip::repeat_block<uint32_t>(3, 0b001),
            0b01001001001001001001001001001001);
  EXPECT_EQ(bit_manip::repeat_block<uint32_t>(4, 0b0001),
            0b00010001000100010001000100010001);
  EXPECT_EQ(bit_manip::repeat_block<uint32_t>(5, 0b00001),
            0b01000010000100001000010000100001);

  EXPECT_EQ(bit_manip::repeat_block<uint64_t>(2, 0b01),
            0b0101010101010101010101010101010101010101010101010101010101010101);
  EXPECT_EQ(bit_manip::repeat_block<uint64_t>(3, 0b001),
            0b1001001001001001001001001001001001001001001001001001001001001001);
  EXPECT_EQ(bit_manip::repeat_block<uint64_t>(4, 0b0001),
            0b0001000100010001000100010001000100010001000100010001000100010001);
  EXPECT_EQ(bit_manip::repeat_block<uint64_t>(5, 0b00001),
            0b0001000010000100001000010000100001000010000100001000010000100001);

  // Magic numbers
  // for 32-bit 2D Morton Codes
  EXPECT_EQ(bit_manip::repeat_block(1, 1), 0xFFFFFFFF);
  EXPECT_EQ(bit_manip::repeat_block<uint32_t>(32, (1 << 16) - 1), 0x0000FFFF);
  EXPECT_EQ(bit_manip::repeat_block<uint32_t>(16, (1 << 8) - 1), 0x00FF00FF);
  EXPECT_EQ(bit_manip::repeat_block<uint32_t>(8, (1 << 4) - 1), 0x0F0F0F0F);
  EXPECT_EQ(bit_manip::repeat_block<uint32_t>(4, (1 << 2) - 1), 0x33333333);
  EXPECT_EQ(bit_manip::repeat_block<uint32_t>(2, 1), 0x55555555);
  // for 64-bit 2D Morton Codes
  EXPECT_EQ(bit_manip::repeat_block<uint64_t>(64, (1ull << 32) - 1),
            0x00000000FFFFFFFF);
  EXPECT_EQ(bit_manip::repeat_block<uint64_t>(32, (1 << 16) - 1),
            0x0000FFFF0000FFFF);
  EXPECT_EQ(bit_manip::repeat_block<uint64_t>(16, (1 << 8) - 1),
            0x00FF00FF00FF00FF);
  EXPECT_EQ(bit_manip::repeat_block<uint64_t>(8, (1 << 4) - 1),
            0x0F0F0F0F0F0F0F0F);
  EXPECT_EQ(bit_manip::repeat_block<uint64_t>(4, (1 << 2) - 1),
            0x3333333333333333);
  EXPECT_EQ(bit_manip::repeat_block<uint64_t>(2, 1), 0x5555555555555555);
  // for 32-bit 3D Morton Codes
  EXPECT_EQ(bit_manip::repeat_block<uint32_t>(32, (1 << 10) - 1), 0x000003FF);
  EXPECT_EQ(bit_manip::repeat_block<uint32_t>(32, 0), 0x00000000);
  EXPECT_EQ(bit_manip::repeat_block<uint32_t>(24, (1 << 8) - 1), 0xFF0000FF);
  EXPECT_EQ(bit_manip::repeat_block<uint32_t>(12, (1 << 4) - 1), 0x0F00F00F);
  EXPECT_EQ(bit_manip::repeat_block<uint32_t>(6, (1 << 2) - 1), 0xC30C30C3);
  EXPECT_EQ(bit_manip::repeat_block<uint32_t>(3, (1 << 1) - 1), 0x49249249);
  // for 64-bit 3D Morton Codes
  EXPECT_EQ(bit_manip::repeat_block<uint64_t>(96, (1ull << 32) - 1),
            0x00000000FFFFFFFF);
  EXPECT_EQ(bit_manip::repeat_block<uint64_t>(48, (1 << 16) - 1),
            0xFFFF00000000FFFF);
  EXPECT_EQ(bit_manip::repeat_block<uint64_t>(24, (1 << 8) - 1),
            0x00FF0000FF0000FF);
  EXPECT_EQ(bit_manip::repeat_block<uint64_t>(12, (1 << 4) - 1),
            0xF00F00F00F00F00F);
  EXPECT_EQ(bit_manip::repeat_block<uint64_t>(6, (1 << 2) - 1),
            0x30C30C30C30C30C3);
  EXPECT_EQ(bit_manip::repeat_block<uint64_t>(3, (1 << 1) - 1),
            0x9249249249249249);
}

#if defined(BIT_EXPAND_AVAILABLE) && defined(BIT_COMPRESS_AVAILABLE)
TEST_F(BitManipulationTest, ExpandCompress) {
  for (uint64_t idx = 0u; idx < (1 << 20); ++idx) {
    const uint64_t source =
        getRandomInteger(0ul, std::numeric_limits<uint64_t>::max());
    const uint64_t selector =
        getRandomInteger(0ul, std::numeric_limits<uint64_t>::max());
    const uint64_t num_bits = bit_manip::popcount(selector);
    const uint64_t source_truncated = source & ((1ull << num_bits) - 1);
    const uint64_t expanded = bit_manip::expand(source, selector);
    const uint64_t source_round_trip = bit_manip::compress(expanded, selector);
    EXPECT_EQ(source_round_trip, source_truncated)
        << "For source \n"
        << std::bitset<64>(source) << " with selector \n"
        << std::bitset<64>(selector) << " (" << num_bits << " bits) to \n"
        << std::bitset<64>(expanded) << " and compressed back to \n"
        << std::bitset<64>(source_round_trip) << " while expecting \n"
        << std::bitset<64>(source_truncated);
  }
}
#endif
}  // namespace wavemap
