#ifndef WAVEMAP_CORE_CONFIG_PARAM_CHECKS_H_
#define WAVEMAP_CORE_CONFIG_PARAM_CHECKS_H_

#include <functional>
#include <string>

#include "wavemap/core/utils/meta/nameof.h"

#define IS_PARAM_EQ(value, threshold, verbose) \
  wavemap::is_param<std::equal_to<>>(value, threshold, verbose, #value, " == ")

#define IS_PARAM_NE(value, threshold, verbose)                              \
  wavemap::is_param<std::not_equal_to<>>(value, threshold, verbose, #value, \
                                         " != ")

#define IS_PARAM_LT(value, threshold, verbose) \
  wavemap::is_param<std::less<>>(value, threshold, verbose, #value, " < ")

#define IS_PARAM_LE(value, threshold, verbose)                            \
  wavemap::is_param<std::less_equal<>>(value, threshold, verbose, #value, \
                                       " <= ")

#define IS_PARAM_GT(value, threshold, verbose) \
  wavemap::is_param<std::greater<>>(value, threshold, verbose, #value, " > ")

#define IS_PARAM_GE(value, threshold, verbose)                               \
  wavemap::is_param<std::greater_equal<>>(value, threshold, verbose, #value, \
                                          " >= ")

#define IS_PARAM_TRUE(value, verbose) \
  wavemap::is_param<std::equal_to<>>(value, true, verbose, #value)

#define IS_PARAM_FALSE(value, verbose) \
  wavemap::is_param<std::equal_to<>>(value, false, verbose, #value)

namespace wavemap {
namespace print {
template <typename ValueT>
constexpr bool is_stringlike = std::is_constructible_v<std::string, ValueT>;

template <typename ValueT>
constexpr bool is_boolean = std::is_same_v<std::decay_t<ValueT>, bool>;

template <typename ValueT>
inline auto param_value(const ValueT& value)
    -> std::enable_if_t<is_boolean<ValueT>, std::string> {
  return value ? "true" : "false";
}

template <typename ValueT>
inline auto param_value(const ValueT& value)
    -> std::enable_if_t<is_stringlike<ValueT>, std::string> {
  return "\"" + std::string(value) + "\"";
}

template <typename ValueT>
inline auto param_value(const ValueT& value)
    -> std::enable_if_t<!is_boolean<ValueT> && !is_stringlike<ValueT>,
                        const ValueT&> {
  return value;
}
}  // namespace print

template <typename ComparisonOp, typename A, typename B>
bool is_param(A value, B threshold) {
  return ComparisonOp{}(value, threshold);
}

template <typename ComparisonOp, typename A, typename B>
bool is_param(A value, B threshold, bool verbose, const std::string& value_name,
              const std::string& comparison_op_string = " ") {
  if (is_param<ComparisonOp, A, B>(value, threshold)) {
    return true;
  } else {
    LOG_IF(WARNING, verbose)
        << "Param \"" << value_name << "\" is not" << comparison_op_string
        << print::param_value(threshold);
    return false;
  }
}
}  // namespace wavemap

#endif  // WAVEMAP_CORE_CONFIG_PARAM_CHECKS_H_
