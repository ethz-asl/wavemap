#include <cmath>

#include <gtest/gtest.h>

#include "wavemap/core/utils/query/probability_conversions.h"

namespace wavemap {
TEST(ProbabilityConversionsTest, LogOddsToProbability) {
  EXPECT_NEAR(convert::logOddsToProbability(convert::kLogOddsNumericalMin), 0.f,
              kEpsilon);
  EXPECT_NEAR(convert::logOddsToProbability(convert::kLogOddsNumericalMax), 1.f,
              kEpsilon);

  constexpr int kNumSteps = 1000;
  for (int step_idx = 1; step_idx < kNumSteps; ++step_idx) {
    const FloatingPoint probability = static_cast<FloatingPoint>(step_idx) /
                                      static_cast<FloatingPoint>(kNumSteps);
    const FloatingPoint log_odds = convert::probabilityToLogOdds(probability);
    const FloatingPoint probability_round_trip =
        convert::logOddsToProbability(log_odds);
    EXPECT_NEAR(probability_round_trip, probability, kEpsilon);
  }
}
}  // namespace wavemap
