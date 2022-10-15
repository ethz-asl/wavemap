#ifndef WAVEMAP_COMMON_UTILS_CONFIG_UTILS_H_
#define WAVEMAP_COMMON_UTILS_CONFIG_UTILS_H_

#include <functional>
#include <string>

#include "wavemap_common/utils/nameof.h"
#include "wavemap_common/utils/param_utils.h"

namespace wavemap {
template <typename ConfigDerivedT>
struct ConfigBase {
  // NOTE: Aggregate initialization will not be available for derived classes
  //       due to the restriction listed on:
  //       https://en.cppreference.com/w/cpp/language/aggregate_initialization
  //       But similar syntax can still be used by manually defining
  //       constructors in the derived class.

  virtual ~ConfigBase() {
    // Force the derived config classes to implement a
    // static method that lets users build them from param maps
    // NOTE: This static assert has to be placed in a method that's guaranteed
    //       to be evaluated by the compiler, making the dtor is a good option.
    static_assert(
        std::is_same_v<decltype(ConfigDerivedT::from(
                           std::declval<const param::Map&>())),
                       ConfigDerivedT>,
        "Derived config type must implement static method with signature \""
        "DerivedConfig::from(const param::Map&)\".");
  }

  virtual bool isValid(bool verbose) const = 0;
  const ConfigDerivedT& checkValid() const {
    CHECK(isValid(true));
    return *static_cast<const ConfigDerivedT*>(this);
  }

 private:
  // Force structs that use ConfigBase (by deriving from it) to pass the right
  // template argument (i.e. themselves) by making the base constructor private
  // and befriending the derived class passed as the template argument
  ConfigBase() = default;
  friend ConfigDerivedT;
};

#define IS_PARAM_EQ(value, threshold, verbose) \
  is_param<std::equal<>>(value, threshold, verbose, NAMEOF(value), "==")

#define IS_PARAM_LT(value, threshold, verbose) \
  is_param<std::less<>>(value, threshold, verbose, NAMEOF(value), "<")

#define IS_PARAM_LE(value, threshold, verbose) \
  is_param<std::less_equal<>>(value, threshold, verbose, NAMEOF(value), "<=")

#define IS_PARAM_GT(value, threshold, verbose) \
  is_param<std::greater<>>(value, threshold, verbose, NAMEOF(value), ">")

#define IS_PARAM_GE(value, threshold, verbose) \
  is_param<std::greater_equal<>>(value, threshold, verbose, NAMEOF(value), ">=")

template <typename ComparisonOp, typename T>
bool is_param(T value, T threshold) {
  return ComparisonOp{}(value, threshold);
}

template <typename ComparisonOp, typename T>
bool is_param(T value, T threshold, bool verbose, const std::string& value_name,
              const std::string& comparison_op_name) {
  if (is_param<ComparisonOp, T>(value, threshold)) {
    return true;
  } else {
    LOG_IF(WARNING, verbose) << "Param \"" << value_name << "\" is not "
                             << comparison_op_name << " " << threshold;
    return false;
  }
}
}  // namespace wavemap

#endif  // WAVEMAP_COMMON_UTILS_CONFIG_UTILS_H_
