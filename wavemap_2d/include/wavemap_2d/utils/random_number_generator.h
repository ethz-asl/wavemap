#ifndef WAVEMAP_2D_UTILS_RANDOM_NUMBER_GENERATOR_H_
#define WAVEMAP_2D_UTILS_RANDOM_NUMBER_GENERATOR_H_

#include <random>

#include "wavemap_2d/common.h"

namespace wavemap_2d {
class RandomNumberGenerator {
 public:
  RandomNumberGenerator() : random_number_generator_(std::random_device()()) {}

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

 protected:
  std::mt19937 random_number_generator_;
};
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_UTILS_RANDOM_NUMBER_GENERATOR_H_
