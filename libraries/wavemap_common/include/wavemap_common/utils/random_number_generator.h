#ifndef WAVEMAP_COMMON_UTILS_RANDOM_NUMBER_GENERATOR_H_
#define WAVEMAP_COMMON_UTILS_RANDOM_NUMBER_GENERATOR_H_

#include <random>

#include "wavemap_common/common.h"

namespace wavemap {
class RandomNumberGenerator {
 public:
  RandomNumberGenerator() : random_number_generator_(std::random_device()()) {}
  explicit RandomNumberGenerator(const size_t random_seed)
      : random_number_generator_(random_seed) {}

  void setRandomSeed(const size_t random_seed) {
    random_number_generator_.seed(random_seed);
  }

  template <typename IntegerType>
  IntegerType getRandomInteger(const IntegerType lower_bound,
                               const IntegerType upper_bound) {
    std::uniform_int_distribution<IntegerType> uniform_distribution(
        lower_bound, upper_bound);
    return uniform_distribution(random_number_generator_);
  }

  template <typename RealNumberType>
  FloatingPoint getRandomRealNumber(const RealNumberType lower_bound,
                                    const RealNumberType upper_bound) {
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

#endif  // WAVEMAP_COMMON_UTILS_RANDOM_NUMBER_GENERATOR_H_
