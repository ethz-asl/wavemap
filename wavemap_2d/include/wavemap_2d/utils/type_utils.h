#ifndef WAVEMAP_2D_UTILS_TYPE_UTILS_H_
#define WAVEMAP_2D_UTILS_TYPE_UTILS_H_

#include <type_traits>

namespace wavemap_2d {
template <typename T>
constexpr auto to_underlying(T value) noexcept {
  return static_cast<std::underlying_type_t<T>>(value);
}
}  // namespace wavemap_2d

#endif  // WAVEMAP_2D_UTILS_TYPE_UTILS_H_
