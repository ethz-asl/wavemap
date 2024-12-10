#ifndef WAVEMAP_CORE_UTILS_MATH_INT_MATH_H_
#define WAVEMAP_CORE_UTILS_MATH_INT_MATH_H_

#include <limits>

#include "wavemap/core/common.h"
#include "wavemap/core/utils/bits/bit_operations.h"

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
  if (std::is_signed_v<T>) {
    return std::numeric_limits<T>::digits - bit_ops::clz(value);
  } else {
    return std::numeric_limits<T>::digits - bit_ops::clz(value) - 1;
  }
}

template <typename T>
constexpr T log2_ceil(T value) {
  static_assert(std::is_integral_v<T>);
  const T log2_floored = log2_floor(value);
  if (bit_ops::popcount(value) == static_cast<T>(1)) {
    return log2_floored;
  } else {
    return log2_floored + static_cast<T>(1);
  }
}

template <typename T>
constexpr bool is_power_of_two(T x) {
  return x && ((x & (x - 1)) == 0);
}

template <typename T>
constexpr T mult_exp2(T value, int exp) {
  return value * exp2(exp);
}

constexpr int div_exp2_floor(int value, int exp) { return value >> exp; }

constexpr int div_exp2_ceil(int value, int exp) {
  return (value + exp2(exp) - 1) >> exp;
}

constexpr int div_exp2_floor_remainder(int value, int exp) {
  const IndexElement mask = (1 << exp) - 1;
  return value & mask;
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
Eigen::Matrix<int, dim, 1> div_exp2_floor_remainder(
    Eigen::Matrix<int, dim, 1> vector, int exp) {
  DCHECK_GE(exp, 0);
  for (int dim_idx = 0; dim_idx < dim; ++dim_idx) {
    vector[dim_idx] = div_exp2_floor_remainder(vector[dim_idx], exp);
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

template <typename T>
constexpr T pow(T base, int exponent) {
  T pow = 1;
  for (int iteration = 0; iteration < exponent; ++iteration) {
    pow *= base;
  }
  return pow;
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

template <int dim>
Eigen::Matrix<int, dim, 1> div_round_up(Eigen::Matrix<int, dim, 1> vector,
                                        int denominator) {
  DCHECK_GE(denominator, 0);
  for (int dim_idx = 0; dim_idx < dim; ++dim_idx) {
    vector[dim_idx] = div_round_up(vector[dim_idx], denominator);
  }
  return vector;
}

template <typename T>
constexpr T factorial(T x) {
  T result = 1;
  for (; 1 < x; --x) {
    result *= x;
  }
  return result;
}

template <typename T>
constexpr T binomial(T n, T m) {
  if (n < m) {
    return 0;
  } else if (n == m) {
    return 1;
  } else {
    return factorial(n) / (factorial(m) * factorial(n - m));
  }
}
}  // namespace wavemap::int_math

#endif  // WAVEMAP_CORE_UTILS_MATH_INT_MATH_H_
