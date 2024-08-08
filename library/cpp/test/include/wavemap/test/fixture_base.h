#ifndef WAVEMAP_TEST_FIXTURE_BASE_H_
#define WAVEMAP_TEST_FIXTURE_BASE_H_

#include <gtest/gtest.h>

#include "wavemap/core/common.h"
#include "wavemap/core/utils/random_number_generator.h"

namespace wavemap {
class FixtureBase : public ::testing::Test {
 protected:
  void SetUp() override {
    constexpr size_t kFixedRandomSeed = 0u;
    random_number_generator_ = RandomNumberGenerator(kFixedRandomSeed);
  }

  template <typename T = int>
  T getRandomInteger(T lower_bound, T upper_bound) {
    return random_number_generator_.getRandomInteger(lower_bound, upper_bound);
  }

  template <typename T = FloatingPoint>
  T getRandomFloat(T lower_bound, T upper_bound) {
    return random_number_generator_.getRandomRealNumber(lower_bound,
                                                        upper_bound);
  }

 private:
  RandomNumberGenerator random_number_generator_;
};
}  // namespace wavemap

#endif  // WAVEMAP_TEST_FIXTURE_BASE_H_
