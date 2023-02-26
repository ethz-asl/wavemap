#ifndef WAVEMAP_CONFIG_CONFIG_BASE_H_
#define WAVEMAP_CONFIG_CONFIG_BASE_H_

#include "wavemap/config/param.h"
#include "wavemap/config/param_checks.h"
#include "wavemap/config/param_conversions.h"

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
}  // namespace wavemap

#endif  // WAVEMAP_CONFIG_CONFIG_BASE_H_
