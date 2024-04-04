#ifndef WAVEMAP_CONFIG_PARAM_CHECKS_H_
#define WAVEMAP_CONFIG_PARAM_CHECKS_H_

#include <functional>
#include <string>

#include "wavemap/core/utils/meta/nameof.h"

#define IS_PARAM_EQ(value, threshold, verbose) \
  wavemap::is_param<std::equal_to<>>(value, threshold, verbose, #value, "==")

#define IS_PARAM_NE(value, threshold, verbose)                              \
  wavemap::is_param<std::not_equal_to<>>(value, threshold, verbose, #value, \
                                         "!=")

#define IS_PARAM_LT(value, threshold, verbose) \
  wavemap::is_param<std::less<>>(value, threshold, verbose, #value, "<")

#define IS_PARAM_LE(value, threshold, verbose) \
  wavemap::is_param<std::less_equal<>>(value, threshold, verbose, #value, "<=")

#define IS_PARAM_GT(value, threshold, verbose) \
  wavemap::is_param<std::greater<>>(value, threshold, verbose, #value, ">")

#define IS_PARAM_GE(value, threshold, verbose)                               \
  wavemap::is_param<std::greater_equal<>>(value, threshold, verbose, #value, \
                                          ">=")

namespace wavemap {
template <typename ComparisonOp, typename A, typename B>
bool is_param(A value, B threshold) {
  return ComparisonOp{}(value, threshold);
}

template <typename ComparisonOp, typename A, typename B>
bool is_param(A value, B threshold, bool verbose, const std::string& value_name,
              const std::string& comparison_op_name) {
  if (is_param<ComparisonOp, A, B>(value, threshold)) {
    return true;
  } else {
    LOG_IF(WARNING, verbose) << "Param \"" << value_name << "\" is not "
                             << comparison_op_name << " " << threshold;
    return false;
  }
}
}  // namespace wavemap

#endif  // WAVEMAP_CONFIG_PARAM_CHECKS_H_
