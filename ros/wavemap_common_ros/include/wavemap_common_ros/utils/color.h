#ifndef WAVEMAP_COMMON_ROS_UTILS_COLOR_H_
#define WAVEMAP_COMMON_ROS_UTILS_COLOR_H_

#include <wavemap_common/common.h>

namespace wavemap {
struct RGBAColor {
  FloatingPoint a;
  FloatingPoint r;
  FloatingPoint g;
  FloatingPoint b;

  static constexpr RGBAColor Transparent() { return {0.f, 0.f, 0.f, 0.f}; }
  static constexpr RGBAColor White() { return {1.f, 1.f, 1.f, 1.f}; }
  static constexpr RGBAColor Black() { return {1.f, 0.f, 0.f, 0.f}; }
};
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_ROS_UTILS_COLOR_H_
