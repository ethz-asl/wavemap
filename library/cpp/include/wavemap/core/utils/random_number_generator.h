#ifndef WAVEMAP_CORE_UTILS_RANDOM_NUMBER_GENERATOR_H_
#define WAVEMAP_CORE_UTILS_RANDOM_NUMBER_GENERATOR_H_

#include <random>

#include "wavemap/core/common.h"

namespace wavemap {
class RandomNumberGenerator {
 public:
  explicit RandomNumberGenerator(size_t random_seed = std::random_device()())
      : random_number_generator_(random_seed) {}

  void setRandomSeed(size_t random_seed) {
    random_number_generator_.seed(random_seed);
  }

  template <typename IntegerType>
  IntegerType getRandomInteger(IntegerType lower_bound,
                               IntegerType upper_bound) {
    std::uniform_int_distribution<IntegerType> uniform_distribution(
        lower_bound, upper_bound);
    return uniform_distribution(random_number_generator_);
  }

  template <typename RealNumberType>
  FloatingPoint getRandomRealNumber(RealNumberType lower_bound,
                                    RealNumberType upper_bound) {
    std::uniform_real_distribution<RealNumberType> uniform_distribution(
        lower_bound, upper_bound);
    return uniform_distribution(random_number_generator_);
  }

  bool getRandomBool(FloatingPoint p_true) {
    std::bernoulli_distribution bernoulli_distribution(p_true);
    return bernoulli_distribution(random_number_generator_);
  }

 private:
  std::mt19937 random_number_generator_;
};
}  // namespace wavemap

#endif  // WAVEMAP_CORE_UTILS_RANDOM_NUMBER_GENERATOR_H_
