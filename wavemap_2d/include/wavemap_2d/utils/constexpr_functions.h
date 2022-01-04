#ifndef WAVEMAP_2D_UTILS_CONSTEXPR_FUNCTIONS_H_
#define WAVEMAP_2D_UTILS_CONSTEXPR_FUNCTIONS_H_

namespace wavemap_2d::constexpr_functions {
constexpr int exp2(int exponent) {
  int value = 1;
  for (int i = 0; i < exponent; ++i) {
    value *= 2;
  }
  return value;
}
}  // namespace wavemap_2d::constexpr_functions

#endif  // WAVEMAP_2D_UTILS_CONSTEXPR_FUNCTIONS_H_
