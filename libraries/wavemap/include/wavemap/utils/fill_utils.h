#ifndef WAVEMAP_UTILS_FILL_UTILS_H_
#define WAVEMAP_UTILS_FILL_UTILS_H_

#include <type_traits>

namespace wavemap::fill {
// Fill POD types with a constant value
template <typename T, typename V>
auto constant(V value) -> std::enable_if_t<std::is_pod_v<T>, T> {
  return static_cast<T>(value);
}

// Fill Eigen types with a constant value
template <typename T, typename V>
auto constant(V value) -> decltype(T::Constant(std::declval<V>())) {
  return T::Constant(value);
}

// Convenience method for zero-filling
template <typename T>
T zero() {
  return constant<T>(0);
}
}  // namespace wavemap::fill

#endif  // WAVEMAP_UTILS_FILL_UTILS_H_
