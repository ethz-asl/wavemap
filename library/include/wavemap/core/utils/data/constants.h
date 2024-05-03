#ifndef WAVEMAP_CORE_UTILS_DATA_CONSTANTS_H_
#define WAVEMAP_CORE_UTILS_DATA_CONSTANTS_H_

namespace wavemap {
// NOTE: We define commonly used constants here instead of directly using the
//       mathematical constants from math.h in the code, mainly for consistency,
//       readability and to avoid unnecessary conversions at runtime. Such
//       conversions can for example be triggered when M_PI (double) is used
//       instead of M_PIf32 in an expression with floats, which reduces general
//       performance and also hinders things like auto-vectorization.
//       Defining our own constants here should also make things easier if we
//       ever want to evaluate different FloatingPoint precisions.
template <typename T>
struct constants {
  static constexpr auto kEpsilon = static_cast<T>(1e-6);
  static constexpr auto kPi = static_cast<T>(M_PI);
  static constexpr auto kTwoPi = static_cast<T>(2 * M_PI);
  static constexpr auto kHalfPi = static_cast<T>(M_PI_2);
  static constexpr auto kQuarterPi = static_cast<T>(M_PI_4);
  static constexpr auto kSqrt2 = static_cast<T>(M_SQRT2);
  static constexpr auto kSqrt2Inv = static_cast<T>(1 / M_SQRT2);
  static constexpr auto kSqrt3 = static_cast<T>(1.73205080757f);
  static constexpr auto kSqrt3Inv = static_cast<T>(1 / 1.73205080757f);
};
}  // namespace wavemap

#endif  // WAVEMAP_CORE_UTILS_DATA_CONSTANTS_H_
