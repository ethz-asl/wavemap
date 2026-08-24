#ifndef WAVEMAP_LAYERED_TYPES_RGB_H_
#define WAVEMAP_LAYERED_TYPES_RGB_H_

#include <algorithm>
#include <cmath>

#include <wavemap/core/common.h>

namespace wavemap::layered {

struct Rgb {
  float r = 0.f;
  float g = 0.f;
  float b = 0.f;

  bool operator==(const Rgb& other) const {
    return r == other.r && g == other.g && b == other.b;
  }
};

struct RgbArithmetic {
  static Rgb add(const Rgb& lhs, const Rgb& rhs) {
    return {lhs.r + rhs.r, lhs.g + rhs.g, lhs.b + rhs.b};
  }

  static Rgb subtract(const Rgb& lhs, const Rgb& rhs) {
    return {lhs.r - rhs.r, lhs.g - rhs.g, lhs.b - rhs.b};
  }

  static Rgb scale(const Rgb& color, FloatingPoint factor) {
    return {factor * color.r, factor * color.g, factor * color.b};
  }
};

inline Rgb clampRgb(const Rgb& color, const Rgb& min, const Rgb& max) {
  return {std::clamp(color.r, min.r, max.r),
          std::clamp(color.g, min.g, max.g),
          std::clamp(color.b, min.b, max.b)};
}

inline FloatingPoint maxAbs(const Rgb& color) {
  return std::max({std::abs(color.r), std::abs(color.g), std::abs(color.b)});
}

}  // namespace wavemap::layered

#endif  // WAVEMAP_LAYERED_TYPES_RGB_H_
