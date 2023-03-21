#ifndef WAVEMAP_UTILS_INT_MATH_H_
#define WAVEMAP_UTILS_INT_MATH_H_

#include <limits>

#include "wavemap/common.h"
#include "wavemap/utils/bit_manipulation.h"

namespace wavemap::int_math {
template <typename T>
constexpr T exp2(T exponent) {
  static_assert(std::is_integral_v<T>);
  return static_cast<T>(1) << exponent;
}

template <typename T>
constexpr T log2_floor(T value) {
  static_assert(std::is_integral_v<T>);
  DCHECK(value != static_cast<T>(0));
  return std::numeric_limits<T>::digits - bit_manip::clz(value);
}

template <typename T>
constexpr T log2_ceil(T value) {
  static_assert(std::is_integral_v<T>);
  const T log2_floored = log2_floor(value);
  if (bit_manip::popcount(value) == static_cast<T>(1)) {
    return log2_floored;
  } else {
    return log2_floored + static_cast<T>(1);
  }
}

template <typename T>
constexpr T mult_exp2(T value, int exp) {
  return value * exp2(exp);
}

constexpr int div_exp2_floor(int value, int exp) { return value >> exp; }

constexpr int div_exp2_ceil(int value, int exp) {
  return (value + exp2(exp) - 1) >> exp;
}

template <int dim>
Eigen::Matrix<int, dim, 1> div_exp2_floor(Eigen::Matrix<int, dim, 1> vector,
                                          int exp) {
  DCHECK_GE(exp, 0);
  for (int dim_idx = 0; dim_idx < dim; ++dim_idx) {
    vector[dim_idx] = div_exp2_floor(vector[dim_idx], exp);
  }
  return vector;
}

template <int dim>
Eigen::Matrix<int, dim, 1> div_exp2_ceil(Eigen::Matrix<int, dim, 1> vector,
                                         int exp) {
  DCHECK_GE(exp, 0);
  for (int dim_idx = 0; dim_idx < dim; ++dim_idx) {
    vector[dim_idx] = div_exp2_ceil(vector[dim_idx], exp);
  }
  return vector;
}

template <typename T, T pow_base, int dim>
constexpr std::array<T, dim> pow_sequence() {
  static_assert(0 < dim);
  std::array<T, dim> pow_sequence{};
  pow_sequence[0] = 1;
  for (int dim_idx = 1; dim_idx < dim; ++dim_idx) {
    pow_sequence[dim_idx] = pow_sequence[dim_idx - 1] * pow_base;
  }
  return pow_sequence;
}

template <typename T>
constexpr T div_round_up(T numerator, T denominator) {
  return numerator / denominator + (numerator % denominator != 0);
}
}  // namespace wavemap::int_math

#endif  // WAVEMAP_UTILS_INT_MATH_H_
