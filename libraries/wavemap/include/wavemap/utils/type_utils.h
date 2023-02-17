#ifndef WAVEMAP_UTILS_TYPE_UTILS_H_
#define WAVEMAP_UTILS_TYPE_UTILS_H_

#include <type_traits>

namespace wavemap {
template <typename T>
constexpr auto to_underlying(T value) noexcept {
  return static_cast<std::underlying_type_t<T>>(value);
}
}  // namespace wavemap

#endif  // WAVEMAP_UTILS_TYPE_UTILS_H_
