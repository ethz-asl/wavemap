#ifndef WAVEMAP_CORE_UTILS_META_TYPE_UTILS_H_
#define WAVEMAP_CORE_UTILS_META_TYPE_UTILS_H_

#include <type_traits>

namespace wavemap::meta {
// Helpers for enum classes
template <typename T>
constexpr auto to_underlying(T value) noexcept {
  return static_cast<std::underlying_type_t<T>>(value);
}

// Helpers for member pointers
template <typename T>
struct member_type;

template <typename C, typename T>
struct member_type<T C::*> {
  using type = T;
};

template <typename T>
using member_type_t = typename member_type<T>::type;

// Helpers for parameter packs
// TODO(victorr): Clean this up and choose one consistent style.
template <typename... Args>
struct TypeList {
  template <typename... AdditionalArgs>
  using Append = TypeList<Args..., AdditionalArgs...>;

  template <typename T>
  struct contains {
    static constexpr bool value{(std::is_same_v<T, Args> || ...)};
  };

  template <typename T>
  static constexpr bool contains_t = contains<T>::value;
};

template <template <class... Args> class T, class... Args>
struct inject_type_list {
  using type = T<Args...>;
};

template <template <class... Args> class T, class... Args>
struct inject_type_list<T, TypeList<Args...>> {
  using type = typename inject_type_list<T, Args...>::type;
};

template <template <class... Args> class T, class... Args>
using inject_type_list_t = typename inject_type_list<T, Args...>::type;

template <template <class... Args> class T, class C, class... Args>
struct inject_type_list_as_member_ptrs {
  using type = T<Args C::*...>;
};

template <template <class... Args> class T, class C, class... Args>
struct inject_type_list_as_member_ptrs<T, C, TypeList<Args...>> {
  using type = typename inject_type_list_as_member_ptrs<T, C, Args...>::type;
};

template <template <class... Args> class T, class C, class... Args>
using inject_type_list_as_member_ptrs_t =
    typename inject_type_list_as_member_ptrs<T, C, Args...>::type;

template <typename T, typename = void>
struct has_to_str_member_fn : std::false_type {};

template <typename T>
struct has_to_str_member_fn<T, std::void_t<decltype(std::declval<T>().toStr())>>
    : std::true_type {};

template <typename T>
constexpr bool has_to_str_member_fn_v = has_to_str_member_fn<T>::value;
}  // namespace wavemap::meta

#endif  // WAVEMAP_CORE_UTILS_META_TYPE_UTILS_H_
