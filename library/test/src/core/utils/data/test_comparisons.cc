#include <gtest/gtest.h>

#include "wavemap/core/utils/data/comparisons.h"

namespace wavemap {
TEST(DataComparisonUtilsTest, IsNonzero) {
  EXPECT_EQ(data::is_nonzero(0), false);
  EXPECT_EQ(data::is_nonzero(-1), true);
  EXPECT_EQ(data::is_nonzero(1), true);

  EXPECT_EQ(data::is_nonzero(0.f), false);
  EXPECT_EQ(data::is_nonzero(-1e-4f), true);
  EXPECT_EQ(data::is_nonzero(1e-4f), true);
  EXPECT_EQ(data::is_nonzero(-1.f), true);
  EXPECT_EQ(data::is_nonzero(1.f), true);

  EXPECT_EQ(data::is_nonzero(0.f, 1e-3f), false);
  EXPECT_EQ(data::is_nonzero(-1e-4f, 1e-3f), false);
  EXPECT_EQ(data::is_nonzero(1e-4f, 1e-3f), false);
  EXPECT_EQ(data::is_nonzero(-1.f, 1e-3f), true);
  EXPECT_EQ(data::is_nonzero(1.f, 1e-3f), true);

  EXPECT_EQ(data::is_nonzero(std::array{0.f}), false);
  EXPECT_EQ(data::is_nonzero(std::array{-1e-4f}), true);
  EXPECT_EQ(data::is_nonzero(std::array{1e-4f}), true);
  EXPECT_EQ(data::is_nonzero(std::array{-1.f}), true);
  EXPECT_EQ(data::is_nonzero(std::array{1.f}), true);

  EXPECT_EQ(data::is_nonzero(std::array{0.f, 0.f}), false);
  EXPECT_EQ(data::is_nonzero(std::array{0.f, -1e-4f}), true);
  EXPECT_EQ(data::is_nonzero(std::array{0.f, 1e-4f}), true);
  EXPECT_EQ(data::is_nonzero(std::array{0.f, -1.f}), true);
  EXPECT_EQ(data::is_nonzero(std::array{0.f, 1.f}), true);

  EXPECT_EQ(data::is_nonzero(std::array{0.f, 0.f}, 1e-3f), false);
  EXPECT_EQ(data::is_nonzero(std::array{0.f, -1e-4f}, 1e-3f), false);
  EXPECT_EQ(data::is_nonzero(std::array{0.f, 1e-4f}, 1e-3f), false);
  EXPECT_EQ(data::is_nonzero(std::array{0.f, -1.f}, 1e-3f), true);
  EXPECT_EQ(data::is_nonzero(std::array{0.f, 1.f}, 1e-3f), true);
}
}  // namespace wavemap
