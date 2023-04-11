#include <gtest/gtest.h>

#include "wavemap/utils/data_utils.h"

namespace wavemap {
TEST(DataUtilsTest, IsNonZero) {
  EXPECT_EQ(data_utils::is_non_zero(0), false);
  EXPECT_EQ(data_utils::is_non_zero(-1), true);
  EXPECT_EQ(data_utils::is_non_zero(1), true);

  EXPECT_EQ(data_utils::is_non_zero(0.f), false);
  EXPECT_EQ(data_utils::is_non_zero(-1e-4f), true);
  EXPECT_EQ(data_utils::is_non_zero(1e-4f), true);
  EXPECT_EQ(data_utils::is_non_zero(-1.f), true);
  EXPECT_EQ(data_utils::is_non_zero(1.f), true);

  EXPECT_EQ(data_utils::is_non_zero(0.f, 1e-3f), false);
  EXPECT_EQ(data_utils::is_non_zero(-1e-4f, 1e-3f), false);
  EXPECT_EQ(data_utils::is_non_zero(1e-4f, 1e-3f), false);
  EXPECT_EQ(data_utils::is_non_zero(-1.f, 1e-3f), true);
  EXPECT_EQ(data_utils::is_non_zero(1.f, 1e-3f), true);

  EXPECT_EQ(data_utils::is_non_zero(std::array{0.f}), false);
  EXPECT_EQ(data_utils::is_non_zero(std::array{-1e-4f}), true);
  EXPECT_EQ(data_utils::is_non_zero(std::array{1e-4f}), true);
  EXPECT_EQ(data_utils::is_non_zero(std::array{-1.f}), true);
  EXPECT_EQ(data_utils::is_non_zero(std::array{1.f}), true);

  EXPECT_EQ(data_utils::is_non_zero(std::array{0.f, 0.f}), false);
  EXPECT_EQ(data_utils::is_non_zero(std::array{0.f, -1e-4f}), true);
  EXPECT_EQ(data_utils::is_non_zero(std::array{0.f, 1e-4f}), true);
  EXPECT_EQ(data_utils::is_non_zero(std::array{0.f, -1.f}), true);
  EXPECT_EQ(data_utils::is_non_zero(std::array{0.f, 1.f}), true);

  EXPECT_EQ(data_utils::is_non_zero(std::array{0.f, 0.f}, 1e-3f), false);
  EXPECT_EQ(data_utils::is_non_zero(std::array{0.f, -1e-4f}, 1e-3f), false);
  EXPECT_EQ(data_utils::is_non_zero(std::array{0.f, 1e-4f}, 1e-3f), false);
  EXPECT_EQ(data_utils::is_non_zero(std::array{0.f, -1.f}, 1e-3f), true);
  EXPECT_EQ(data_utils::is_non_zero(std::array{0.f, 1.f}, 1e-3f), true);
}
}  // namespace wavemap
