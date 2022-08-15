#include <gtest/gtest.h>

#include "wavemap_common/common.h"
#include "wavemap_common/utils/approximate_trigonometry.h"

namespace wavemap {
TEST(ApproximateTrigonometryTest, ApproxAtan2) {
  constexpr int kNumAngles = 360 * 1000;
  for (int i = 0; i <= kNumAngles; ++i) {
    const FloatingPoint angle = kTwoPi * static_cast<FloatingPoint>(i) /
                                static_cast<FloatingPoint>(kNumAngles);
    const Vector2D bearing{std::cos(angle), std::sin(angle)};
    EXPECT_NEAR(approximate::atan2()(bearing.y(), bearing.x()),
                std::atan2(bearing.y(), bearing.x()),
                approximate::atan2::kWorstCaseError);
  }
}
}  // namespace wavemap
